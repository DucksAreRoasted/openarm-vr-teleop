// SafetyFilter non-finite behavior that already holds on main.
// URDF clamping, Stop, and size mismatch are tested in safety_filter_api_test.cpp
// and run only after PR #4's helpers are present.

#include "safety_contract.hpp"

#include <common.hpp>
#include <safety/safety_filter.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

SafetyConfig wide_config() {
    SafetyConfig config;
    config.min_position.assign(8, -10.0);
    config.max_position.assign(8, 10.0);
    config.max_velocity.assign(8, 0.0);
    config.max_acceleration.assign(8, 0.0);
    config.max_effort.assign(8, 0.0);
    return config;
}

std::vector<JointState> pose_with_elbow(double elbow, double velocity, double effort) {
    std::vector<JointState> joints(8);
    joints[static_cast<size_t>(kElbowJointIndex)].position = elbow;
    joints[static_cast<size_t>(kElbowJointIndex)].velocity = velocity;
    joints[static_cast<size_t>(kElbowJointIndex)].effort = effort;
    return joints;
}

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

bool position_held(const std::vector<JointState>& out, double expected) {
    if (out.size() != 8) {
        return false;
    }
    const JointState& joint = out[static_cast<size_t>(kElbowJointIndex)];
    return std::isfinite(joint.position) && std::isfinite(joint.velocity) &&
           std::isfinite(joint.effort) && std::abs(joint.position - expected) < 1e-12;
}

int test_nonfinite() {
    const double held = 1.2;
    const double velocity = 0.25;
    const double effort = 0.4;
    const double nonfinite_values[] = {std::numeric_limits<double>::quiet_NaN(),
                                        std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity()};

    for (double bad : nonfinite_values) {
        for (int field = 0; field < 3; ++field) {
            SafetyFilter filter(wide_config());
            const auto seed = pose_with_elbow(held, velocity, effort);
            filter.reset(seed);
            auto raw = seed;
            JointState& joint = raw[static_cast<size_t>(kElbowJointIndex)];
            if (field == 0) {
                joint.position = bad;
            } else if (field == 1) {
                joint.velocity = bad;
            } else {
                joint.effort = bad;
            }
            std::vector<JointState> out;
            filter.filter(raw, seed, 0.001, &out);
            if (!position_held(out, held)) {
                return fail("non-finite command did not keep the previous elbow position");
            }
        }
    }

    SafetyFilter filter(wide_config());
    const auto seed = pose_with_elbow(held, velocity, effort);
    filter.reset(seed);
    auto raw = pose_with_elbow(1.5, 0.0, 0.0);
    std::vector<JointState> out;
    filter.filter(raw, seed, 0.001, &out);
    if (!position_held(out, 1.5)) {
        return fail("finite in-range elbow command was not passed through");
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "nonfinite") {
        return test_nonfinite();
    }
    std::cerr << "usage: safety_filter_contract_test nonfinite\n";
    return 2;
}
