#include <safety/safety_filter.hpp>

#include <openarm_constants.hpp>

#include <sstream>
#include <utility>

namespace {

struct UrdfJointRow {
    double lower;
    double upper;
};

// assets/v1.urdf revolute limits. Index 3 is openarm_*_joint4 (upper 2.443461).
constexpr UrdfJointRow kLeftUrdf[7] = {
    {-3.490659, 1.3962629999999998},
    {-3.3161253267948965, 0.17453267320510335},
    {-1.570796, 1.570796},
    {0.0, 2.443461},
    {-2.792527, 2.792527},
    {-0.785398, 0.785398},
    {-1.570796, 1.570796},
};

constexpr UrdfJointRow kRightUrdf[7] = {
    {-1.396263, 3.490659},
    {-0.17453267320510335, 3.3161253267948965},
    {-1.570796, 1.570796},
    {0.0, 2.443461},
    {-2.792527, 2.792527},
    {-0.785398, 0.785398},
    {-1.570796, 1.570796},
};

}  // namespace

std::optional<UrdfArmSide> parse_urdf_arm_side(std::string_view name) {
    if (name == "left" || name == "left_arm") {
        return UrdfArmSide::Left;
    }
    if (name == "right" || name == "right_arm") {
        return UrdfArmSide::Right;
    }
    return std::nullopt;
}

bool urdf_arm_position_limit(std::optional<UrdfArmSide> side, size_t joint_index,
                             UrdfPositionLimit* limit) {
    if (limit == nullptr || joint_index >= 7) {
        return false;
    }
    if (side.has_value() && *side == UrdfArmSide::Left) {
        limit->lower = kLeftUrdf[joint_index].lower;
        limit->upper = kLeftUrdf[joint_index].upper;
        return true;
    }
    if (side.has_value() && *side == UrdfArmSide::Right) {
        limit->lower = kRightUrdf[joint_index].lower;
        limit->upper = kRightUrdf[joint_index].upper;
        return true;
    }
    limit->lower = std::max(kLeftUrdf[joint_index].lower, kRightUrdf[joint_index].lower);
    limit->upper = std::min(kLeftUrdf[joint_index].upper, kRightUrdf[joint_index].upper);
    return limit->lower <= limit->upper;
}

SafetyFilter::SafetyFilter(SafetyConfig config) : config_(std::move(config)) {}

void SafetyFilter::reset(const std::vector<JointState>& initial_cmd) {
    last_cmd_ = initial_cmd;
    initialized_ = true;
    first_step_pending_ = true;
}

void SafetyFilter::set_mode(SafetyMode mode) { mode_ = mode; }

void SafetyFilter::set_arm_side(std::string_view side) { arm_side_ = parse_urdf_arm_side(side); }

void SafetyFilter::apply_shared_velocity_effort_limits() {
    // 0 means not enabled. make_sim_safety_config() does max(velocity_limit, 12)
    // and would otherwise enable a limit the raw table left off. Real and sim
    // installed filters both keep the raw table.
    for (size_t index = 0; index < config_.max_velocity.size(); ++index) {
        const size_t table = index < velocity_limit.size() ? index : velocity_limit.size() - 1;
        config_.max_velocity[index] = velocity_limit[table];
    }
    for (size_t index = 0; index < config_.max_effort.size(); ++index) {
        const size_t table = index < effort_limit.size() ? index : effort_limit.size() - 1;
        config_.max_effort[index] = effort_limit[table];
    }
}

SafetyStatus SafetyFilter::filter(const std::vector<JointState>& raw_cmd,
                                  const std::vector<JointState>& current_state, double dt,
                                  std::vector<JointState>* safe_cmd) {
    // dt is not used to integrate velocity or acceleration. Those limits are
    // applied only as a symmetric clamp, and only when the configured value
    // is > 0. 0 means not enabled, for both simulation and real hardware.
    (void)dt;
    SafetyStatus status;
    if (safe_cmd == nullptr) {
        mode_ = SafetyMode::Stop;
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "safe_cmd is null";
        return status;
    }

    if (mode_ == SafetyMode::Stop) {
        *safe_cmd = make_explicit_stop_command(current_state, last_cmd_, arm_side_);
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "safety stop mode";
        if (!safe_cmd->empty()) {
            last_cmd_ = *safe_cmd;
            initialized_ = true;
        }
        return status;
    }

    if (raw_cmd.empty() || raw_cmd.size() != current_state.size()) {
        mode_ = SafetyMode::Stop;
        *safe_cmd = make_explicit_stop_command(current_state, last_cmd_, arm_side_);
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "command/state size mismatch";
        if (!safe_cmd->empty()) {
            last_cmd_ = *safe_cmd;
            initialized_ = true;
        }
        return status;
    }

    if (!initialized_ || last_cmd_.size() != raw_cmd.size()) {
        reset(current_state);
    }

    std::ostringstream reason;
    *safe_cmd = raw_cmd;
    const bool limit_first_step = first_step_pending_;

    for (size_t index = 0; index < safe_cmd->size(); ++index) {
        JointState& out = (*safe_cmd)[index];
        const JointState previous = index < last_cmd_.size() ? last_cmd_[index] : JointState{};
        bool replaced = false;
        out = hold_previous_joint(out, previous, &replaced);
        if (replaced) {
            status.ok = false;
            status.clamped = true;
            reason << "non-finite joint " << index << "; ";
        }

        if (limit_first_step) {
            const double anchor = std::isfinite(previous.position) ? previous.position
                                  : (index < current_state.size() &&
                                     std::isfinite(current_state[index].position))
                                      ? current_state[index].position
                                      : out.position;
            bool limited = false;
            const double original = out.position;
            out.position =
                limit_first_step_delta(out.position, anchor, config_.max_first_step_delta, &limited);
            if (limited) {
                status.clamped = true;
                reason << "first-step clamp joint " << index << " " << original << " -> "
                       << out.position << "; ";
            }
        }

        // URDF clamp is last so a first-step pull toward an out-of-range anchor
        // cannot command past openarm_*_joint* limits (elbow upper 2.443461).
        UrdfPositionLimit position_limit;
        if (urdf_arm_position_limit(arm_side_, index, &position_limit) &&
            std::isfinite(out.position)) {
            const double original = out.position;
            out.position = std::clamp(out.position, position_limit.lower, position_limit.upper);
            if (out.position != original) {
                status.clamped = true;
                reason << "position clamp joint " << index << " " << original << " -> "
                       << out.position << "; ";
            }
        }

        if (index < config_.max_velocity.size()) {
            bool limited = false;
            const double original = out.velocity;
            out.velocity =
                clamp_symmetric_if_enabled(out.velocity, config_.max_velocity[index], &limited);
            if (limited) {
                status.clamped = true;
                reason << "velocity clamp joint " << index << " " << original << " -> "
                       << out.velocity << "; ";
            }
        }

        if (index < config_.max_effort.size()) {
            bool limited = false;
            const double original = out.effort;
            out.effort = clamp_symmetric_if_enabled(out.effort, config_.max_effort[index], &limited);
            if (limited) {
                status.clamped = true;
                reason << "feedforward effort clamp joint " << index << " " << original << " -> "
                       << out.effort << "; ";
            }
        }
    }

    last_cmd_ = *safe_cmd;
    initialized_ = true;
    first_step_pending_ = false;
    status.reason = reason.str();
    return status;
}
