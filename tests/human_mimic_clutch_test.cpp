// Hardware-free clutch contract for the real HumanMimicController.
// Never opens CAN, a socket, or a MuJoCo window.
//
// Current behavior, confirmed with weiqiang: a Quest timeout or more than
// kSolveFailFreezeCount consecutive IK failures drops the clutch. If Grip is
// still held, the next fresh frame is treated as a rising edge, the clutch
// re-engages, and activate_clutch() clears the IK failure counter.

#include "safety_contract.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller/quest_types.hpp"
#include "controller/dynamics.hpp"
#include "controller/human_mimic_ik_solver.hpp"
#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"
#include "controller/joint_reference_generator.hpp"
#include "controller/reference_smoother.hpp"
#include "controller/virtual_arm_skeleton.hpp"
#include "openarm_constants.hpp"
#include "robot_state.hpp"

// Parsed after the headers above so include guards keep Eigen/KDL/URDF on
// their normal access specifiers. Layout is unchanged; this only lets the
// test read the clutch counters.
#define private public
#define protected public
#include "controller/human_mimic_controller.hpp"
#undef private
#undef protected

namespace {

void require(bool ok, const std::string& message) {
    if (!ok) {
        throw std::runtime_error(message);
    }
}

class Arm {
public:
    explicit Arm(const std::string& urdf, bool reanchor_to_commanded = true)
        : dynamics_(urdf, "openarm_body_link0", "openarm_right_link7") {
        require(dynamics_.Init(), "dynamics init failed for " + urdf);
        HumanMimicConfig defaults;
        require(std::abs(defaults.quest_timeout_sec - kQuestTimeoutSec) < 1e-12,
                "HumanMimicConfig::quest_timeout_sec drifted from safety_contract.hpp");
        require(defaults.solve_fail_freeze_count == kSolveFailFreezeCount,
                "HumanMimicConfig::solve_fail_freeze_count drifted from safety_contract.hpp");

        config_.urdf_path = urdf;
        config_.root_link = "openarm_body_link0";
        config_.shoulder_link = "openarm_right_link1";
        config_.elbow_link = "openarm_right_link4";
        config_.wrist_link = "openarm_right_link7";
        config_.outward_dir = Eigen::Vector3d(0.0, -1.0, 0.0);
        config_.debug_log = false;
        config_.gripper_buttons_enabled = false;
        config_.reanchor_to_commanded = reanchor_to_commanded;
        config_.clutch_settle_ticks = 0;
        config_.clutch_track_settle_ticks = 0;
        config_.quest_timeout_sec = kQuestTimeoutSec;
        config_.solve_fail_freeze_count = kSolveFailFreezeCount;
        config_.q_nominal.assign(TABLE_READY_POSITION.begin(), TABLE_READY_POSITION.begin() + 7);

        control_ = std::make_unique<HumanMimicController>(&dynamics_, config_, "clutch-test ");
        require(control_->Init(), "HumanMimicController::Init failed");
        joints_.resize(control_->arm_dof());
        for (size_t i = 0; i < joints_.size() && i < TABLE_READY_POSITION.size(); ++i) {
            joints_[i].position = TABLE_READY_POSITION[i];
        }
    }

    HumanMimicController& control() { return *control_; }

    QuestPoseSample sample(bool grip, double age_sec, bool valid, double x) const {
        QuestPoseSample pose;
        pose.valid = valid;
        pose.grip_active = grip;
        pose.position = Eigen::Vector3d(x, 0.0, 0.3);
        pose.orientation = Eigen::Quaterniond::Identity();
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
        pose.receive_timestamp_ns = now_ns - static_cast<int64_t>(age_sec * 1e9);
        return pose;
    }

    void step(const QuestPoseSample& pose) {
        require(control_->step(joints_, pose, 0.01, &reference_), "step() returned false");
    }

    // Grip at `x`, then move far enough to leave the idle gate and complete one solve.
    void engage_and_track(double x) {
        step(sample(true, 0.0, true, x));
        require(control_->clutch_active(), "grip rising edge did not engage the clutch");
        const double moved = x + 0.05;
        step(sample(true, 0.0, true, moved));
        step(sample(true, 0.0, true, moved));
        require(control_->clutch_active(), "clutch dropped while starting to track");
        require(!control_->clutch_idle_, "hand motion did not leave the idle gate");
        require(control_->q_target_.size() == control_->arm_dof_, "tracking did not seed q_target_");
        hand_x_ = moved;
    }

    double hand_x() const { return hand_x_; }
    const std::vector<JointState>& reference() const { return reference_; }
    void sag(double radians) { joints_[1].position += radians; }

private:
    Dynamics dynamics_;
    HumanMimicConfig config_;
    std::unique_ptr<HumanMimicController> control_;
    std::vector<JointState> joints_;
    std::vector<JointState> reference_;
    double hand_x_ = 0.2;
};

void test_timeout_reengages_while_grip_held(const std::string& urdf) {
    Arm arm(urdf);
    arm.step(arm.sample(false, 0.0, true, 0.2));
    require(!arm.control().clutch_active(), "clutch engaged without grip");

    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "grip did not engage the clutch");

    // Above the stale threshold (0.05 s) and still under the timeout.
    arm.step(arm.sample(true, kQuestTimeoutSec * 0.5, true, 0.2));
    require(arm.control().clutch_active(), "sub-timeout Quest age dropped the clutch");
    require(!arm.control().quest_timeout_active(), "sub-timeout age latched quest timeout");

    arm.control().solve_fail_count_ = 4;
    arm.step(arm.sample(true, kQuestTimeoutSec, true, 0.2));
    require(!arm.control().clutch_active(), "Quest timeout did not drop the clutch");
    require(arm.control().quest_timeout_active(), "timeout did not latch quest_timeout_active_");
    require(!arm.control().last_grip_, "timeout did not clear last_grip_ while Grip is held");
    require(arm.control().require_reclutch_, "timeout did not request a re-clutch");
    require(arm.control().solve_fail_count_ == 4, "timeout path cleared the IK failure counter");

    // Brief dropout recovery: the next fresh frame still has Grip held.
    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "held Grip did not re-engage after Quest recovery");
    require(!arm.control().quest_timeout_active(), "recovery left the timeout latch set");
    require(arm.control().solve_fail_count_ == 0,
            "synthetic rising edge did not clear the IK failure counter");
}

void test_invalid_packet_reengages_while_grip_held(const std::string& urdf) {
    Arm arm(urdf);
    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "grip did not engage");

    arm.step(arm.sample(true, 0.0, false, 0.2));
    require(!arm.control().clutch_active(), "invalid Quest sample did not drop the clutch");
    require(!arm.control().last_grip_, "invalid sample did not clear last_grip_");

    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "held Grip did not resume after an invalid packet");
}

void test_timeout_recovery_does_not_ratchet_to_sagged_measurement(const std::string& urdf) {
    // Unitree/PIKA use measured seeding on a physical re-grip. An automatic
    // reconnect while Grip stays held must instead retain the commanded hold.
    Arm arm(urdf, false);
    arm.step(arm.sample(true, 0.0, true, 0.2));
    const auto before = arm.reference();
    for (int dropout = 0; dropout < 3; ++dropout) {
        arm.sag(0.06);
        arm.step(arm.sample(true, kQuestTimeoutSec, true, 0.2));
        require(arm.control().quest_timeout_active(), "dropout did not freeze");
        arm.step(arm.sample(true, 0.0, true, 0.2));
        require(arm.control().clutch_active(), "held Grip did not resume");
        for (size_t i = 0; i < before.size(); ++i) {
            require(std::abs(arm.reference()[i].position - before[i].position) < 1e-9,
                    "automatic timeout recovery lowered the commanded hold pose");
        }
    }
    // A real release/re-grip keeps the profile's intentional measured anchoring.
    arm.step(arm.sample(false, 0.0, true, 0.2));
    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(std::abs(arm.reference()[1].position - before[1].position) > 0.1,
            "physical re-grip no longer anchors to measurement");
}

void test_physical_release_stays_open(const std::string& urdf) {
    Arm arm(urdf);
    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "grip did not engage");

    arm.step(arm.sample(false, 0.0, true, 0.2));
    require(!arm.control().clutch_active(), "releasing Grip did not drop the clutch");
    arm.step(arm.sample(false, 0.0, true, 0.2));
    require(!arm.control().clutch_active(), "clutch re-engaged while Grip stayed released");

    arm.step(arm.sample(true, 0.0, true, 0.2));
    require(arm.control().clutch_active(), "a real Grip rising edge did not engage");
}

void test_ik_failures_reengage_and_clear_count(const std::string& urdf) {
    Arm arm(urdf);
    arm.engage_and_track(0.2);

    // SolveDiffQp rejects a seed shorter than the arm DoF (ok stays false).
    // That is a real solver rejection, so step() increments solve_fail_count_.
    arm.control().q_target_.clear();
    arm.step(arm.sample(true, 0.0, true, arm.hand_x()));
    require(arm.control().clutch_active(), "the first IK failure froze the clutch immediately");
    require(arm.control().solve_fail_count_ == 1, "the first IK failure did not increment the counter");

    int steps = 1;
    while (arm.control().clutch_active() && steps < 80) {
        arm.control().q_target_.clear();
        arm.step(arm.sample(true, 0.0, true, arm.hand_x()));
        ++steps;
    }
    require(!arm.control().clutch_active(),
            "clutch stayed engaged after " + std::to_string(steps) +
                " IK failures, count=" + std::to_string(arm.control().solve_fail_count_));
    // `>` the threshold: 31 failures, then the next step freezes without another increment.
    require(steps == kSolveFailFreezeCount + 2,
            "freeze happened on step " + std::to_string(steps) + ", expected " +
                std::to_string(kSolveFailFreezeCount + 2));
    require(arm.control().solve_fail_count_ == kSolveFailFreezeCount + 1,
            "freeze left solve_fail_count_ at " + std::to_string(arm.control().solve_fail_count_));
    require(!arm.control().last_grip_, "IK freeze did not clear last_grip_ while Grip is held");
    require(arm.control().require_reclutch_, "IK freeze did not request a re-clutch");

    arm.step(arm.sample(true, 0.0, true, arm.hand_x()));
    require(arm.control().clutch_active(), "held Grip did not re-engage after the IK freeze");
    require(arm.control().solve_fail_count_ == 0,
            "re-engage left solve_fail_count_ at " + std::to_string(arm.control().solve_fail_count_));

    // One later failure must not freeze again. That only holds if the counter was cleared.
    const double further = arm.hand_x() + 0.05;
    arm.step(arm.sample(true, 0.0, true, further));
    arm.step(arm.sample(true, 0.0, true, further));
    require(arm.control().clutch_active(), "tracking after re-engage dropped the clutch");
    arm.control().q_target_.clear();
    arm.step(arm.sample(true, 0.0, true, further));
    require(arm.control().clutch_active(), "a single IK failure after re-engage froze the clutch");
    require(arm.control().solve_fail_count_ == 1,
            "post-recovery IK failure count is " + std::to_string(arm.control().solve_fail_count_));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: human_mimic_clutch_test <urdf>\n";
        return 2;
    }
    try {
        const std::string urdf = argv[1];
        test_timeout_reengages_while_grip_held(urdf);
        test_invalid_packet_reengages_while_grip_held(urdf);
        test_timeout_recovery_does_not_ratchet_to_sagged_measurement(urdf);
        test_physical_release_stays_open(urdf);
        test_ik_failures_reengage_and_clear_count(urdf);
    } catch (const std::exception& error) {
        std::cerr << "human_mimic_clutch_test: " << error.what() << "\n";
        return 1;
    }
    std::cout << "human_mimic_clutch_test: ok\n";
    return 0;
}
