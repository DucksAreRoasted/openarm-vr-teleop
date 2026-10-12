#pragma once

#include <controller/gripper_follow.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// Body-frame same-direction following: j1 and j7 have opposite physical axes.
// The gripper is a motor angle, not the URDF finger translation.
class MirrorReference {
public:
    using Pose = std::array<double, 8>;
    static constexpr Pose signs{-1, 1, 1, 1, 1, 1, -1, 1};
    MirrorReference(Pose initial, Pose lower, Pose upper)
        : q_(initial), lower_(lower), upper_(upper) {}

    Pose step(const Pose& right, double dt) {
        return step(right, {}, dt);
    }

    Pose step(const Pose& right, const Pose& right_velocity, double dt) {
        return step_impl(right, right_velocity, nullptr, nullptr, dt);
    }

    Pose step(const Pose& right, const Pose& right_velocity, const Pose& left,
              const Pose& left_velocity, double dt) {
        return step_impl(right, right_velocity, &left, &left_velocity, dt);
    }

private:
    Pose step_impl(const Pose& right, const Pose& right_velocity, const Pose* left,
                   const Pose* left_velocity, double dt) {
        if (!std::isfinite(dt) || dt <= 0 || dt > 0.05) {
            throw std::runtime_error("mirror: servo deadline missed");
        }
        const Pose goal = target_pose(right);
        bool aligned = true;
        for (size_t i = 0; i < q_.size(); ++i) {
            if (!std::isfinite(right_velocity[i])) {
                throw std::runtime_error("mirror: non-finite master velocity");
            }
            // A gripper held against an object must not block arm readiness.
            if (i < 7) {
                aligned = aligned && std::abs(goal[i] - q_[i]) < 0.015 &&
                    std::abs(velocity_[i]) < 0.1 && std::abs(right_velocity[i]) < 0.1;
            }
        }
        tracking_ = tracking_ || aligned;
        for (size_t i = 0; i < q_.size(); ++i) {
            const double error = goal[i] - q_[i];
            const double vmax = tracking_ ? (i == 7 ? 2.0 : 1.5) : (i == 7 ? 0.5 : 0.35);
            const double amax = tracking_ ? (i == 7 ? 8.0 : 6.0) : (i == 7 ? 1.0 : 0.7);
            // Velocity feedforward removes the error required by a position-only
            // low-pass follower. Position correction closes drift without a 0.5s lag.
            // A clipped target is stationary until the master re-enters the
            // follower's stroke. Do not feed outward velocity through a limit.
            const double mapped_position = signs[i] * right[i];
            const double mapped_velocity = signs[i] * right_velocity[i];
            const bool inside = mapped_position >= lower_[i] && mapped_position <= upper_[i];
            const bool outward = (mapped_position <= lower_[i] && mapped_velocity < 0) ||
                                 (mapped_position >= upper_[i] && mapped_velocity > 0);
            const double desired = std::clamp(
                (tracking_ && inside && !outward ? mapped_velocity : 0.0) +
                (tracking_ ? 20.0 : 2.0) * error, -vmax, vmax);
            velocity_[i] += std::clamp(desired - velocity_[i], -amax * dt, amax * dt);
            const double next = q_[i] + velocity_[i] * dt;
            q_[i] = std::clamp(next, lower_[i], upper_[i]);
            if (q_[i] != next) velocity_[i] = 0;
        }
        if (left && left_velocity) {
            const auto command = gripper_.step(goal[7], q_[7], velocity_[7], (*left)[7],
                                               (*left_velocity)[7], lower_[7], upper_[7], dt);
            q_[7] = command.position;
            velocity_[7] = command.velocity;
            gripper_limited_ = command.limited;
        }
        return q_;
    }
public:
    const Pose& velocity() const { return velocity_; }
    bool tracking() const { return tracking_; }
    bool gripper_holding() const { return gripper_.holding(); }
    bool gripper_limited() const { return gripper_limited_; }
    Pose target_pose(const Pose& right) const {
        Pose target{};
        for (size_t i = 0; i < target.size(); ++i) {
            if (!std::isfinite(right[i])) throw std::runtime_error("mirror: non-finite master position");
            target[i] = std::clamp(signs[i] * right[i], lower_[i], upper_[i]);
        }
        return target;
    }
private:
    Pose q_, lower_, upper_;
    Pose velocity_{};
    bool tracking_ = false;
    GripperFollow gripper_;
    bool gripper_limited_ = false;
};
