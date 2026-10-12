#include <controller/gravity_comp.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

bool near(double value, double expected) { return std::abs(value - expected) < 1e-9; }

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        return fail("usage: no_gripper_gravity_smoke installed.yaml omitted.yaml");
    }
    GravityModel model;
    const double fingers = kOpenArmFingerMassKg;
    if (!near(end_effector_point_mass_kg(model, true), fingers + 0.12)) {
        return fail("default gripper mass must keep the 0.12 motor default");
    }
    model.gripper_motor_kg = 0.0;
    model.ee_payload_kg = 0.131;
    if (!near(end_effector_point_mass_kg(model, true), fingers + 0.131)) {
        return fail("explicit 0 gripper_motor_kg must not fall back to 0.12");
    }
    if (!near(end_effector_point_mass_kg(model, false), 0.131)) {
        return fail("removed gripper must keep only ee_payload");
    }
    if (!near(kOpenArmFingerComZ, 0.1475)) {
        return fail("finger COM drifted");
    }

    model.gripper_installed = false;
    model.gripper_motor_kg = 0.3;
    if (gripper_configured_installed(model)) {
        return fail("gripper_installed: false must win");
    }
    if (!near(end_effector_point_mass_kg(model, gripper_configured_installed(model)), 0.131)) {
        return fail("configured-off mass");
    }

    model.gripper_installed = true;
    if (setenv("OPENARM_NO_GRIPPER", "1", 1) != 0) {
        return fail("setenv");
    }
    if (gripper_configured_installed(model)) {
        return fail("OPENARM_NO_GRIPPER=1 must force the gripper off");
    }
    unsetenv("OPENARM_NO_GRIPPER");
    if (!gripper_configured_installed(model)) {
        return fail("clearing the env must restore the yaml flag");
    }

    GravityTune left;
    GravityTune right;
    GravityModel loaded;
    if (!load_gravity_yaml(argv[1], &left, &right, &loaded)) {
        return fail("load installed yaml");
    }
    if (loaded.gripper_installed || !near(loaded.gripper_motor_kg, 0.0) ||
        !near(loaded.ee_payload_kg, 0.05)) {
        return fail("failed to parse gripper_installed: false");
    }
    GravityModel omitted;
    if (!load_gravity_yaml(argv[2], &left, &right, &omitted)) {
        return fail("load omitted yaml");
    }
    if (!omitted.gripper_installed || !near(omitted.gripper_motor_kg, 0.12)) {
        return fail("omitted gripper keys must keep installed + 0.12 kg");
    }
    std::cout << "ok\n";
    return 0;
}
