#pragma once

// Joint-reference guard that runs before a follower MIT frame is packed.
// It is not a torque limiter for the motor-side Kp spring. Quest's calibrated
// gravity FF is separately bounded in Control; the spring lives in the drive.
// What this layer does:
//   - clamp arm joint position references to assets/v1.urdf
//     (openarm_*_joint4 upper is 2.443461, not the π elbow in
//     openarm_constants.hpp — that wider constant is not applied)
//   - keep the previous finite output when a command is NaN or inf
//   - limit the first position step after reset when max_first_step_delta > 0
//   - clamp velocity / feedforward effort only when that joint's limit is > 0
//
// velocity_limit and effort_limit: a value <= 0 means the limit is not enabled.
// Simulation and real hardware share that reading. The tables are 0, so those
// clamps stay off. make_sim_safety_config() rewrites a 0 velocity limit to 12;
// Control::SetSafetyFilter calls apply_shared_velocity_effort_limits() so the
// installed sim and real filters both keep the raw 0 ("not enabled").
//
// SafetyMode::Stop is latched by set_mode(Stop) and by a command/state size
// mismatch. filter() then writes an explicit safe command (measured or last
// position, velocity 0, effort 0) instead of leaving safe_cmd empty.

#include <robot_state.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

enum class SafetyMode {
    Normal,
    Stop,
};

enum class UrdfArmSide {
    Left,
    Right,
};

struct SafetyConfig {
    std::vector<double> min_position;
    std::vector<double> max_position;
    std::vector<double> max_velocity;
    std::vector<double> max_acceleration;
    std::vector<double> max_effort;
    // First filter step after reset: |q_out - anchor| is limited to this when
    // the value is > 0. A value <= 0 means the first-step limit is not enabled.
    // Later steps are not rate-limited here.
    double max_first_step_delta = 0.35;
};

struct SafetyStatus {
    bool ok = true;
    bool clamped = false;
    bool emergency_stop = false;
    std::string reason;
};

struct UrdfPositionLimit {
    double lower = 0.0;
    double upper = 0.0;
};

// "left" / "left_arm" / "right" / "right_arm". Anything else is unknown.
std::optional<UrdfArmSide> parse_urdf_arm_side(std::string_view name);

// Arm joints 0..6 from assets/v1.urdf. Index 3 is openarm_*_joint4,
// lower 0, upper 2.443461 on both sides. Returns false for the gripper motor
// (index >= 7) and any other index: that joint is not a URDF revolute here.
// side == nullopt uses the intersection of left and right, which never exceeds
// either URDF (elbow stays 0 .. 2.443461).
bool urdf_arm_position_limit(std::optional<UrdfArmSide> side, size_t joint_index,
                             UrdfPositionLimit* limit);

inline bool joint_state_finite(const JointState& joint) {
    return std::isfinite(joint.position) && std::isfinite(joint.velocity) &&
           std::isfinite(joint.effort);
}

// Non-finite command keeps the previous finite joint unchanged.
// If the previous joint is also non-finite, the result is zeros.
inline JointState hold_previous_joint(const JointState& commanded, const JointState& previous,
                                      bool* replaced) {
    if (joint_state_finite(commanded)) {
        return commanded;
    }
    if (replaced != nullptr) {
        *replaced = true;
    }
    if (joint_state_finite(previous)) {
        return previous;
    }
    return JointState{};
}

// max_delta <= 0 means the first-step limit is not enabled.
inline double limit_first_step_delta(double reference, double anchor, double max_delta,
                                     bool* limited) {
    if (!(max_delta > 0.0) || !std::isfinite(reference) || !std::isfinite(anchor)) {
        return reference;
    }
    const double delta = reference - anchor;
    if (std::abs(delta) <= max_delta) {
        return reference;
    }
    if (limited != nullptr) {
        *limited = true;
    }
    return anchor + std::copysign(max_delta, delta);
}

// limit <= 0 means this velocity or effort limit is not enabled.
// Simulation and real configs use this same rule.
inline double clamp_symmetric_if_enabled(double value, double limit, bool* clamped) {
    if (!(limit > 0.0) || !std::isfinite(value)) {
        return value;
    }
    const double next = std::clamp(value, -limit, limit);
    if (clamped != nullptr && next != value) {
        *clamped = true;
    }
    return next;
}

// Explicit stop command: position is the finite measured pose (else last finite
// position, else 0) clamped to the URDF arm limits; velocity and effort are 0.
inline std::vector<JointState> make_explicit_stop_command(
    const std::vector<JointState>& current_state, const std::vector<JointState>& last_cmd,
    std::optional<UrdfArmSide> side) {
    const std::vector<JointState>* source = !current_state.empty() ? &current_state : &last_cmd;
    std::vector<JointState> command(source->size());
    for (size_t index = 0; index < source->size(); ++index) {
        JointState joint;
        const JointState& measured = (*source)[index];
        const JointState previous = index < last_cmd.size() ? last_cmd[index] : JointState{};
        if (std::isfinite(measured.position)) {
            joint.position = measured.position;
        } else if (std::isfinite(previous.position)) {
            joint.position = previous.position;
        }
        UrdfPositionLimit limit;
        if (urdf_arm_position_limit(side, index, &limit) && std::isfinite(joint.position)) {
            joint.position = std::clamp(joint.position, limit.lower, limit.upper);
        }
        joint.velocity = 0.0;
        joint.effort = 0.0;
        command[index] = joint;
    }
    return command;
}

class SafetyFilter {
public:
    explicit SafetyFilter(SafetyConfig config);

    // dt is accepted for callers that already pass the control period. Velocity
    // and acceleration are not integrated from dt. A max_velocity / max_effort
    // entry <= 0 is not enabled (same rule for simulation and real hardware).
    SafetyStatus filter(const std::vector<JointState>& raw_cmd,
                        const std::vector<JointState>& current_state, double dt,
                        std::vector<JointState>* safe_cmd);

    void reset(const std::vector<JointState>& initial_cmd);
    void set_mode(SafetyMode mode);
    SafetyMode mode() const { return mode_; }
    void set_arm_side(std::string_view side);

    // Copy velocity_limit / effort_limit (0 = not enabled) over any rewritten
    // config values, so a sim config and a real config mean the same thing.
    void apply_shared_velocity_effort_limits();

private:
    SafetyConfig config_;
    SafetyMode mode_ = SafetyMode::Normal;
    bool initialized_ = false;
    bool first_step_pending_ = true;
    std::optional<UrdfArmSide> arm_side_;
    std::vector<JointState> last_cmd_;
};
