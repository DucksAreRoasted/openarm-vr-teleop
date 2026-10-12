// SafetyFilter contract. Does not open CAN or a simulator.
// nonfinite: NaN/Inf must not pass through; position stays on the last finite command.
// clamp: out-of-range references must clamp to the URDF elbow limit. Not implemented
//        on main, so this mode returns 77 until the control/safety change lands.
// default-elbow: prints make_default_safety_config()'s elbow upper limit.

#include "safety_contract.hpp"

#include <common.hpp>
#include <safety/safety_filter.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kNotOnMain = 77;

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

int test_clamp() {
    SafetyConfig config = wide_config();
    config.min_position[static_cast<size_t>(kElbowJointIndex)] = 0.0;
    config.max_position[static_cast<size_t>(kElbowJointIndex)] = kElbowUrdfUpperRad;
    const auto seed = pose_with_elbow(1.0, 0.0, 0.0);

    auto run = [&](double command) {
        SafetyFilter filter(config);
        filter.reset(seed);
        auto raw = pose_with_elbow(command, 0.0, 0.0);
        std::vector<JointState> out;
        filter.filter(raw, seed, 0.001, &out);
        return out[static_cast<size_t>(kElbowJointIndex)].position;
    };

    // 4 rad is past both the URDF elbow stop and the pi constant.
    // URDF + 0.3 rad is past the URDF stop and still below pi, so a clamp to
    // position_limit_max would leave it unchanged.
    const double above_both = 4.0;
    const double between_urdf_and_pi = kElbowUrdfUpperRad + 0.3;
    const double high = run(above_both);
    const double mid = run(between_urdf_and_pi);
    const bool unchanged = std::abs(high - above_both) < 1e-9 &&
                           std::abs(mid - between_urdf_and_pi) < 1e-9;
    if (unchanged) {
        std::cerr << "SafetyFilter does not clamp position on main\n";
        return kNotOnMain;
    }
    const bool at_urdf = std::abs(high - kElbowUrdfUpperRad) < 1e-6 &&
                         std::abs(mid - kElbowUrdfUpperRad) < 1e-6;
    if (!at_urdf) {
        return fail("elbow command was clamped, but not to the URDF upper limit");
    }
    return 0;
}

int print_default_elbow() {
    const SafetyConfig config = make_default_safety_config(8);
    std::cout.setf(std::ios::fmtflags(0), std::ios::floatfield);
    std::cout.precision(17);
    std::cout << config.max_position.at(static_cast<size_t>(kElbowJointIndex)) << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "nonfinite") {
        return test_nonfinite();
    }
    if (mode == "clamp") {
        return test_clamp();
    }
    if (mode == "default-elbow") {
        return print_default_elbow();
    }
    std::cerr << "usage: safety_filter_contract_test nonfinite|clamp|default-elbow\n";
    return 2;
}
