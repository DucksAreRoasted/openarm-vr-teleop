#pragma once

#include <controller/dynamics.hpp>
#include <controller/follower_config.hpp>
#include <controller/gravity_comp.hpp>
#include <controller/replay_feedback.hpp>
#include <hardware/teleop_hardware.hpp>
#include <openarm_port/openarm_init.hpp>

#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

// One-thread replay hardware ownership: validate feedback before sending commands.
class ReplayArm {
public:
    using Pose = std::array<double, 8>;
    ReplayArm(const std::string& side, const std::string& can, const std::string& urdf,
              const GravityTune& tune, const GravityModel& model)
        : side_(side), can_name_(can), dynamics_(std::make_shared<Dynamics>(urdf, "openarm_body_link0", "openarm_" + side + "_link7")),
          tune_(tune), model_(model) {
        apply_model();
        if (!dynamics_->Init()) throw std::runtime_error("replay: gravity model init failed");
        apply_model();
        std::vector<double> lo, hi;
        if (!dynamics_->GetJointPositionLimits(&lo, &hi) || lo.size() != 7 || hi.size() != 7) {
            throw std::runtime_error("replay: missing URDF joint limits");
        }
        std::copy(lo.begin(), lo.end(), lower.begin());
        std::copy(hi.begin(), hi.end(), upper.begin());
        lower[7] = -0.98; upper[7] = 0.62;
        can_.reset(openarm_init::OpenArmInitializer::initialize_openarm(can, false));
        hardware_ = std::make_shared<CanTeleopHardware>(can_.get());
        try {
            if (can_) wait_for_feedback();
            read();
            configure_control();
        } catch (...) {
            hardware_->disable_all();
            throw;
        }
    }
    // In-memory/simulation backend for validating the real replay command path.
    ReplayArm(const std::string& side, std::shared_ptr<TeleopHardware> hardware,
              std::shared_ptr<Dynamics> dynamics, const GravityTune& tune, const GravityModel& model)
        : side_(side), dynamics_(std::move(dynamics)), tune_(tune), model_(model), hardware_(std::move(hardware)) {
        std::vector<double> lo, hi;
        if (!dynamics_->GetJointPositionLimits(&lo, &hi) || lo.size()!=7 || hi.size()!=7)
            throw std::runtime_error("replay: missing joint limits");
        std::copy(lo.begin(), lo.end(), lower.begin());
        std::copy(hi.begin(), hi.end(), upper.begin());
        lower[7]=-.98; upper[7]=.62;
        apply_model();
        read();
        configure_control();
    }
    ~ReplayArm() { stop(); }
    void stop() noexcept {
        if (hardware_) {
            try { hardware_->disable_all(); } catch (...) {}
        }
    }
    void read() {
        // These motors may report only in response to an explicit status request.
        if (can_) can_->refresh_all();
        hardware_->recv_all(1000);
        if (can_) {
            validate_feedback(can_->get_arm().get_motors());
            validate_feedback(can_->get_gripper().get_motors());
        }
        const auto arm = hardware_->read_arm();
        const auto grip = hardware_->read_gripper();
        if (arm.size() != 7 || grip.size() != 1) throw std::runtime_error("replay: expected 7+1 motors");
        measured.clear();
        for (const auto& m : arm) measured.push_back({m.position, m.velocity, m.effort});
        measured.push_back({grip[0].position, grip[0].velocity, grip[0].effort});
        for (size_t i = 0; i < 8; ++i) {
            const auto& m = measured[i];
            validate_replay_feedback(side_, i,
                                     m.position, m.velocity, m.effort, lower[i], upper[i]);
        }
    }
    Pose pose() const {
        Pose q{};
        for (size_t i = 0; i < 8; ++i) q[i] = measured[i].position;
        return q;
    }
    Pose velocity() const {
        Pose dq{};
        for (size_t i = 0; i < 8; ++i) dq[i] = measured[i].velocity;
        return dq;
    }
    void set_diagnostic_context(const Pose& source, int phase, double time) {
        std::array<double, 7> q{};
        std::copy_n(source.begin(), 7, q.begin());
        control_->SetDiagnosticContext(q, phase, time);
    }
    void send(const Pose& target, const Pose& velocity) {
        std::vector<JointState> reference(8);
        for (size_t i=0; i<8; ++i) {
            if (!std::isfinite(target[i]) || !std::isfinite(velocity[i]) ||
                target[i]<lower[i] || target[i]>upper[i] ||
                std::abs(target[i]-measured[i].position) > (i==7 ? .201 : .45)) {
                throw std::runtime_error("replay: invalid target/tracking error on " +
                    side_ + " joint " + std::to_string(i+1));
            }
            reference[i].position=target[i]; reference[i].velocity=velocity[i];
        }
        state_->set_all_references(reference);
        // Same MIT packing, gravity ramp/clamp, gripper hold and safety filter as
        // Quest. No replay-only low gains, friction FF or cancellation of PD.
        if (!control_->unilateral_step()) {
            throw std::runtime_error("replay: follower stopped: " + control_->OutputDisabledReason());
        }
    }
    Pose lower{}, upper{};
    std::vector<JointState> measured;
private:
    void configure_control() {
        state_ = std::make_shared<RobotSystemState>(7, 1);
        control_ = std::make_unique<Control>(hardware_, dynamics_.get(), dynamics_.get(),
            state_, .005, ROLE_FOLLOWER, side_+"_arm", 7, 1);
        configure_follower_control(control_.get(), load_follower_params(), 8, false, side_+"_arm");
        if (env_bool_or_common("OPENARM_QUEST_GRAVITY_FF", true)) {
            control_->SetFollowerGravity(dynamics_, tune_,
                env_double_or_common("OPENARM_QUEST_GRAVITY_GAIN", model_.tracking_ff_gain),
                env_double_or_common("OPENARM_QUEST_GRAVITY_RAMP_SEC", 1.0));
        }
        control_->SetGripperFriction(model_.gripper_friction_fc, model_.gripper_friction_fv,
                                     model_.gripper_friction_k);
        control_->BindCanBus(can_.get());
        if (!control_->SeedHoldMeasured()) throw std::runtime_error("replay: failed to seed follower hold");
    }
    void wait_for_feedback() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        std::string last_error;
        do {
            can_->refresh_all();
            // recv_all stops draining when the queue is momentarily empty. Allow
            // separately arriving replies from all eight motors before checking.
            for (int i = 0; i < 5; ++i) {
                can_->recv_all(2000);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            try {
                validate_feedback(can_->get_arm().get_motors());
                validate_feedback(can_->get_gripper().get_motors());
                std::cout << "[replay] " << side_ << " can=" << can_name_
                          << " all 8 motors have fresh feedback" << std::endl;
                return;
            } catch (const std::exception& error) {
                last_error = error.what();
            }
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("replay: startup feedback wait failed (1s): " + last_error);
    }
    template <class Motors> void validate_feedback(const Motors& motors) {
        for (const auto& m : motors) {
            const bool missing = m.last_state_received().time_since_epoch().count() == 0;
            const double age_ms = missing ? -1.0 : std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - m.last_state_received()).count();
            if (missing || age_ms > 150 || m.get_error_code() > 1) {
                std::ostringstream message;
                message << "replay: " << (missing ? "no CAN feedback" :
                    m.get_error_code() > 1 ? "motor fault" : "stale CAN feedback")
                        << " side=" << side_ << " can=" << can_name_
                        << " joint=" << m.get_send_can_id()
                        << " rx_id=0x" << std::hex << m.get_recv_can_id() << std::dec
                        << " age_ms=" << age_ms << " status_code=" << m.get_error_code();
                throw std::runtime_error(message.str());
            }
        }
    }
    void apply_model() {
        configure_follower_gravity_model(dynamics_.get(), model_, side_);
    }
    std::string side_;
    std::string can_name_;
    std::shared_ptr<Dynamics> dynamics_;
    GravityTune tune_;
    GravityModel model_;
    std::unique_ptr<openarm::can::socket::OpenArm> can_;
    std::shared_ptr<TeleopHardware> hardware_;
    std::shared_ptr<RobotSystemState> state_;
    std::unique_ptr<Control> control_;
};
