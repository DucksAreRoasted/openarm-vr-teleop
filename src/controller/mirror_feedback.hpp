#pragma once

#include <controller/mirror_speed_guard.hpp>

#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

inline double mirror_feedback_speed_limit(bool master, size_t joint) {
    // Hand-driven input and the powered follower need different envelopes.
    // This does not change the follower's commanded velocity/acceleration limits.
    return joint == 7 ? 10.0 : (master ? 4.0 : kMirrorFollowerSpeedStop);
}

inline void validate_mirror_feedback(const std::string& side, bool master, size_t joint,
                                    double position, double velocity, double effort,
                                    double lower, double upper) {
    const double speed_limit = mirror_feedback_speed_limit(master, joint);
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(effort) ||
        position < lower - 0.08 || position > upper + 0.08 ||
        std::abs(velocity) > speed_limit) {
        std::ostringstream error;
        error << "mirror: invalid feedback/limit/speed on " << side << " joint " << joint+1
              << " q=" << position << " dq=" << velocity << " tau=" << effort
              << " limits=[" << lower << "," << upper << "]"
              << " speed_limit=" << speed_limit << " role=" << (master ? "master" : "follower");
        throw std::runtime_error(error.str());
    }
}
