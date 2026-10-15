#pragma once

#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

// Live feedback stop thresholds, independent of the historical torque guard.
// J7's startup peak is 2.63 rad/s; retain a hard stop above 3 rad/s.
inline constexpr double kReplayArmFeedbackSpeedStop = 2.0;
inline constexpr double kReplayWristFeedbackSpeedStop = 3.0;
inline constexpr double kReplayGripperFeedbackSpeedStop = 10.0;

inline double replay_feedback_speed_limit(size_t joint) {
    if (joint >= 8) throw std::runtime_error("replay: invalid feedback joint index");
    if (joint == 7) return kReplayGripperFeedbackSpeedStop;
    return joint == 6 ? kReplayWristFeedbackSpeedStop : kReplayArmFeedbackSpeedStop;
}

inline void validate_replay_feedback(const std::string& side, size_t joint,
                                    double position, double velocity, double effort,
                                    double lower, double upper) {
    const double speed_limit = replay_feedback_speed_limit(joint);
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(effort) ||
        position < lower - 0.08 || position > upper + 0.08 ||
        std::abs(velocity) > speed_limit) {
        std::ostringstream error;
        error << "replay: invalid feedback/limit/speed on " << side << " joint " << joint+1
              << " q=" << position << " dq=" << velocity << " tau=" << effort
              << " limits=[" << lower << "," << upper << "]"
              << " speed_limit=" << speed_limit;
        throw std::runtime_error(error.str());
    }
}
