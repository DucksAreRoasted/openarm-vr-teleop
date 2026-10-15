#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

inline constexpr double kReplaySpeedGuardStart = 0.8;
inline constexpr double kReplayTrackingSpeed = 1.5;
inline constexpr double kReplayFollowerSpeedStop = 2.0;
// Historical policy, retained for offline comparisons only. ReplayArm now uses
// the same Control follower as Quest; these are NOT live replay torque limits.
inline constexpr std::array<double, 7> kReplayLegacyTotalEffortLimits{20, 20, 7, 7, 3, 3, 3};

// Shape the complete estimated MIT torque, relative to gravity holding torque.
// Position PD and friction compensation must not accelerate an already fast joint.
inline double replay_follower_torque(double total, double gravity_hold,
                                    double measured_velocity, double effort_limit) {
    if (!std::isfinite(total) || !std::isfinite(gravity_hold) ||
        !std::isfinite(measured_velocity) || !std::isfinite(effort_limit) || effort_limit <= 0) {
        throw std::runtime_error("replay: invalid speed guard input");
    }
    // Start from realizable motor torque; a large PD backlog must not defeat
    // attenuation by supplying arbitrarily large pre-saturation torque.
    total = std::clamp(total, -effort_limit, effort_limit);
    const double speed = std::abs(measured_velocity);
    if (speed > kReplaySpeedGuardStart) {
        const double direction = std::copysign(1.0, measured_velocity);
        double drive = direction * (total - gravity_hold);
        if (drive > 0) {
            drive *= std::clamp((kReplayTrackingSpeed - speed) /
                               (kReplayTrackingSpeed - kReplaySpeedGuardStart), 0.0, 1.0);
        }
        // Keep existing braking and add damping proportional to excess speed.
        // Torque limits remain authoritative, even when gravity uses the headroom.
        drive -= 0.5 * effort_limit * (speed - kReplaySpeedGuardStart) /
                 (kReplayFollowerSpeedStop - kReplaySpeedGuardStart);
        total = gravity_hold + direction * drive;
    }
    return std::clamp(total, -effort_limit, effort_limit);
}
