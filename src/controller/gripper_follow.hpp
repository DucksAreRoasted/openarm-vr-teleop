#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

struct GripperFollowCommand {
    double position;
    double velocity;
    bool limited;
    bool holding;
};

// Positive motor angle closes the gripper. Contact is inferred, not a force sensor.
class GripperFollow {
public:
    GripperFollowCommand step(double requested, double position, double velocity,
                              double measured, double measured_velocity,
                              double lower, double upper, double dt) {
        if (!std::isfinite(requested) || !std::isfinite(position) || !std::isfinite(velocity) ||
            !std::isfinite(measured) || !std::isfinite(measured_velocity) ||
            !std::isfinite(lower) || !std::isfinite(upper) || lower >= upper ||
            !std::isfinite(dt) || dt <= 0 || dt > .05) {
            throw std::runtime_error("gripper: invalid following input");
        }
        // Limit spring deflection instead of allowing a stalled target to run away.
        const double lo = std::max(lower, measured - .2);
        const double hi = std::min(upper, measured + .2);
        if (lo > hi) throw std::runtime_error("gripper: feedback outside stroke");
        if (holding_) {
            peak_request_ = std::max(peak_request_, requested);
            if (requested < peak_request_ - .03 || requested < hold_position_ - .02) {
                holding_ = false;
                stalled_time_ = 0;
                // An object may stop the left jaw well before the right jaw.
                // Follow relative opening immediately, without first waiting
                // for the absolute right angle to pass the contact position.
                releasing_ = true;
                release_offset_ = measured - peak_request_;
                minimum_request_ = requested;
                position = measured;
                velocity = 0;
            }
        }
        if (releasing_) {
            minimum_request_ = std::min(minimum_request_, requested);
            if (requested > minimum_request_ + .03 || requested <= lower + .01) {
                releasing_ = false;
            } else {
                position = std::min(position, requested + release_offset_);
                velocity = std::min(velocity, 0.0);
            }
        }
        if (!holding_ && !releasing_) {
            const bool closing_stall = requested - measured > .15 &&
                position - measured > .12 && std::abs(measured_velocity) < .05;
            stalled_time_ = closing_stall ? stalled_time_ + dt : 0;
            if (stalled_time_ >= .25) {
                holding_ = true;
                hold_position_ = measured;
                peak_request_ = requested;
            }
        }
        if (holding_) {
            // 6 Nm/rad * 0.05 rad = 0.3 Nm static spring, before the 1 Nm cap.
            position = std::clamp(hold_position_ + .05, lo, hi);
            velocity = 0;
        }
        const double limited_position = std::clamp(position, lo, hi);
        const bool limited = limited_position != position;
        if (limited && (position - limited_position)*velocity > 0) velocity = 0;
        return {limited_position, velocity, limited, holding_};
    }
    bool holding() const { return holding_; }
private:
    bool holding_ = false;
    double stalled_time_ = 0;
    double hold_position_ = 0;
    double peak_request_ = 0;
    bool releasing_ = false;
    double release_offset_ = 0;
    double minimum_request_ = 0;
};
