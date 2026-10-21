// Reference check for MoveIt follow: one clamped cubic over the whole path.
// Intermediate knots must keep speed; the goal must stay inside P0 settle.
#include <controller/s_curve.hpp>

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr double kControlDt = 0.01;
// Tightest published settle, P0 in impedance_profiles.yaml.
constexpr double kP0SettleRad = 0.05;

std::array<double, 7> pose_at(double joint0) {
    std::array<double, 7> pose{};
    pose[0] = joint0;
    return pose;
}

size_t nearest(const std::vector<double>& times, double target) {
    size_t best = 0;
    double best_dt = std::abs(times[0] - target);
    for (size_t index = 1; index < times.size(); ++index) {
        const double dt = std::abs(times[index] - target);
        if (dt < best_dt) {
            best_dt = dt;
            best = index;
        }
    }
    return best;
}

void check_path(const std::vector<double>& times, const std::vector<double>& joint0,
                const std::vector<size_t>& moving_knots) {
    std::vector<std::array<double, 7>> points;
    points.reserve(joint0.size());
    for (double value : joint0) {
        points.push_back(pose_at(value));
    }

    const auto dense = openarm::scurve::densify_clamped_cubic(times, points, kControlDt);
    assert(dense.times.size() == dense.points.size());
    assert(dense.velocities.size() == dense.points.size());
    assert(dense.times.size() > times.size());

    assert(std::abs(dense.velocities.front()[0]) < 1e-9);
    assert(std::abs(dense.velocities.back()[0]) < 1e-9);
    for (size_t joint = 1; joint < 7; ++joint) {
        for (const auto& velocity : dense.velocities) {
            assert(std::abs(velocity[joint]) < 1e-9);
        }
    }

    for (size_t knot = 0; knot < times.size(); ++knot) {
        const size_t index = nearest(dense.times, times[knot]);
        assert(std::abs(dense.times[index] - times[knot]) < 1e-9);
        assert(std::abs(dense.points[index][0] - joint0[knot]) < 1e-9);
    }

    const double goal_error = std::abs(dense.points.back()[0] - joint0.back());
    assert(goal_error < kP0SettleRad);

    // Per-segment smootherstep is stopped at every knot (unit_smooth_dot(0/1) == 0).
    assert(std::abs(openarm::scurve::unit_smooth_dot(0.0)) < 1e-12);
    assert(std::abs(openarm::scurve::unit_smooth_dot(1.0)) < 1e-12);
    for (size_t knot : moving_knots) {
        const size_t index = nearest(dense.times, times[knot]);
        assert(std::abs(dense.velocities[index][0]) > 0.2);
    }

    double peak_step = 0.0;
    for (size_t index = 1; index < dense.velocities.size(); ++index) {
        const double dt = std::max(dense.times[index] - dense.times[index - 1], 1e-6);
        peak_step = std::max(peak_step, std::abs(dense.velocities[index][0] -
                                                 dense.velocities[index - 1][0]) /
                                             dt);
    }
    assert(peak_step < 20.0);
}

}  // namespace

int main() {
    check_path({0.0, 1.0, 2.0}, {0.0, 0.5, 1.0}, {1});
    check_path({0.0, 0.8, 1.6, 2.4}, {0.0, 0.2, 0.55, 1.0}, {1, 2});

    const auto single = openarm::scurve::densify_clamped_cubic({0.0, 1.0}, {pose_at(0.0), pose_at(0.4)},
                                                               kControlDt);
    assert(std::abs(single.velocities.front()[0]) < 1e-9);
    assert(std::abs(single.velocities.back()[0]) < 1e-9);
    assert(std::abs(single.points.back()[0] - 0.4) < kP0SettleRad);
    const size_t mid = nearest(single.times, 0.5);
    assert(std::abs(single.velocities[mid][0]) > 0.2);

    std::cout << "trajectory spline: interior speed kept, goal inside P0 settle\n";
    return 0;
}
