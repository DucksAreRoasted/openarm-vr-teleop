// Calls the SafetyFilter helpers added by PR #4. No CAN and no simulator.
// First-step limiting is turned off where the assertion is the URDF clamp,
// so a large step is not absorbed by the 0.35 rad first-step guard.

#include "safety_contract.hpp"

#include <safety/safety_filter.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

SafetyConfig quiet_config() {
    SafetyConfig config;
    config.max_velocity.assign(8, 0.0);
    config.max_effort.assign(8, 0.0);
    config.max_first_step_delta = 0.0;
    return config;
}

std::vector<JointState> eight(double elbow) {
    std::vector<JointState> joints(8);
    joints[static_cast<size_t>(kElbowJointIndex)].position = elbow;
    return joints;
}

int test_urdf_limits() {
    UrdfPositionLimit elbow;
    if (!urdf_arm_position_limit(std::nullopt, static_cast<size_t>(kElbowJointIndex), &elbow)) {
        return fail("elbow URDF limit was missing");
    }
    if (std::abs(elbow.upper - kElbowUrdfUpperRad) > 1e-9 || elbow.lower != 0.0) {
        return fail("elbow limit is not the URDF 0 .. 2.443461 window");
    }
    UrdfPositionLimit left;
    UrdfPositionLimit right;
    if (!urdf_arm_position_limit(UrdfArmSide::Left, 3, &left) ||
        !urdf_arm_position_limit(UrdfArmSide::Right, 3, &right)) {
        return fail("sided elbow limit was missing");
    }
    if (std::abs(left.upper - kElbowUrdfUpperRad) > 1e-9 ||
        std::abs(right.upper - kElbowUrdfUpperRad) > 1e-9) {
        return fail("left/right elbow upper is not the URDF value");
    }
    UrdfPositionLimit ignored;
    if (urdf_arm_position_limit(std::nullopt, 7, &ignored)) {
        return fail("gripper index 7 was treated as a URDF revolute");
    }
    if (parse_urdf_arm_side("left_arm") != UrdfArmSide::Left ||
        parse_urdf_arm_side("right") != UrdfArmSide::Right ||
        parse_urdf_arm_side("nope").has_value()) {
        return fail("arm side parser did not match left/right/unknown");
    }
    return 0;
}

int test_hold_previous() {
    JointState previous;
    previous.position = 1.2;
    previous.velocity = 0.25;
    previous.effort = 0.4;
    bool replaced = false;
    const JointState kept = hold_previous_joint(previous, previous, &replaced);
    if (replaced || kept.position != previous.position) {
        return fail("a finite joint was replaced");
    }

    const double bad_values[] = {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()};
    for (double bad : bad_values) {
        for (int field = 0; field < 3; ++field) {
            JointState command = previous;
            if (field == 0) {
                command.position = bad;
            } else if (field == 1) {
                command.velocity = bad;
            } else {
                command.effort = bad;
            }
            replaced = false;
            const JointState held = hold_previous_joint(command, previous, &replaced);
            if (!replaced || held.position != previous.position || held.velocity != previous.velocity ||
                held.effort != previous.effort) {
                return fail("non-finite command did not keep the whole previous joint");
            }
        }
    }

    JointState neither = previous;
    neither.position = std::numeric_limits<double>::quiet_NaN();
    replaced = false;
    const JointState zeros = hold_previous_joint(neither, neither, &replaced);
    if (!replaced || zeros.position != 0.0 || zeros.velocity != 0.0 || zeros.effort != 0.0) {
        return fail("two non-finite joints did not become zeros");
    }
    return 0;
}

int test_filter_clamps_to_urdf() {
    SafetyFilter filter(quiet_config());
    filter.set_arm_side("left");
    const auto seed = eight(1.0);
    const double between_urdf_and_pi = kElbowUrdfUpperRad + 0.3;
    const double commands[] = {between_urdf_and_pi, 4.0, -0.2};
    const double expected[] = {kElbowUrdfUpperRad, kElbowUrdfUpperRad, 0.0};
    for (int i = 0; i < 3; ++i) {
        filter.reset(seed);
        auto raw = eight(commands[i]);
        std::vector<JointState> out;
        filter.filter(raw, seed, 0.001, &out);
        const double position = out[static_cast<size_t>(kElbowJointIndex)].position;
        if (std::abs(position - expected[i]) > 1e-6) {
            return fail("elbow reference was not clamped to the URDF window");
        }
    }

    filter.reset(seed);
    auto raw = seed;
    raw[7].position = 3.0;
    std::vector<JointState> out;
    filter.filter(raw, seed, 0.001, &out);
    if (std::abs(out[7].position - 3.0) > 1e-12) {
        return fail("gripper index was clamped as if it were a URDF joint");
    }
    return 0;
}

int test_stop_and_size_mismatch() {
    SafetyFilter filter(quiet_config());
    filter.set_arm_side("left");
    auto measured = eight(4.0);
    measured[static_cast<size_t>(kElbowJointIndex)].velocity = 1.0;
    measured[static_cast<size_t>(kElbowJointIndex)].effort = 2.0;
    filter.set_mode(SafetyMode::Stop);
    std::vector<JointState> out;
    const SafetyStatus stopped = filter.filter(measured, measured, 0.001, &out);
    if (stopped.ok || !stopped.emergency_stop || filter.mode() != SafetyMode::Stop) {
        return fail("Stop mode did not latch an emergency stop");
    }
    const JointState& elbow = out[static_cast<size_t>(kElbowJointIndex)];
    if (std::abs(elbow.position - kElbowUrdfUpperRad) > 1e-6 || elbow.velocity != 0.0 ||
        elbow.effort != 0.0) {
        return fail("Stop command was not a URDF-clamped position with zero velocity and effort");
    }

    JointState missing;
    missing.position = std::numeric_limits<double>::quiet_NaN();
    std::vector<JointState> measured_nan(8, missing);
    std::vector<JointState> last = eight(1.1);
    const std::vector<JointState> explicit_stop =
        make_explicit_stop_command(measured_nan, last, UrdfArmSide::Left);
    if (std::abs(explicit_stop[static_cast<size_t>(kElbowJointIndex)].position - 1.1) > 1e-12 ||
        explicit_stop[static_cast<size_t>(kElbowJointIndex)].velocity != 0.0 ||
        explicit_stop[static_cast<size_t>(kElbowJointIndex)].effort != 0.0) {
        return fail("NaN measured pose did not fall back to the last finite position");
    }

    SafetyFilter mismatch(quiet_config());
    mismatch.set_arm_side("left");
    const auto state = eight(1.0);
    const std::vector<JointState> short_command(3);
    try {
        const SafetyStatus status = mismatch.filter(short_command, state, 0.001, &out);
        if (!status.emergency_stop || mismatch.mode() != SafetyMode::Stop) {
            return fail("size mismatch did not latch Stop");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return fail("size mismatch threw instead of latching Stop");
    }
    if (out.empty()) {
        return fail("size mismatch left the safe command empty");
    }
    for (const JointState& joint : out) {
        if (joint.velocity != 0.0 || joint.effort != 0.0) {
            return fail("size-mismatch stop command still has velocity or effort");
        }
    }
    const SafetyStatus still = mismatch.filter(state, state, 0.001, &out);
    if (!still.emergency_stop || mismatch.mode() != SafetyMode::Stop) {
        return fail("Stop did not stay latched after the sizes matched again");
    }

    bool clamped = false;
    if (clamp_symmetric_if_enabled(5.0, 0.0, &clamped) != 5.0 || clamped) {
        return fail("a non-positive velocity/effort limit clamped a value");
    }
    if (clamp_symmetric_if_enabled(5.0, 1.0, &clamped) != 1.0 || !clamped) {
        return fail("a positive limit did not clamp");
    }
    bool limited = false;
    if (std::abs(limit_first_step_delta(2.0, 1.0, 0.35, &limited) - 1.35) > 1e-12 || !limited) {
        return fail("first-step delta was not limited to 0.35");
    }
    limited = false;
    if (limit_first_step_delta(2.0, 1.0, 0.0, &limited) != 2.0 || limited) {
        return fail("a disabled first-step limit still changed the reference");
    }
    return 0;
}

}  // namespace

int main() {
    if (const int urdf = test_urdf_limits()) {
        return urdf;
    }
    if (const int held = test_hold_previous()) {
        return held;
    }
    if (const int clamped = test_filter_clamps_to_urdf()) {
        return clamped;
    }
    return test_stop_and_size_mismatch();
}
