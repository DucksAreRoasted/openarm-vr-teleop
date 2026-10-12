"""Gripper removal must not keep the 0.12 kg fallback or the finger point mass."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from lib.gravity_yaml_io import load_params  # noqa: E402


def compile_smoke() -> Path:
    binary = Path(tempfile.mkdtemp(prefix="no-gripper-gravity-")) / "no_gripper_gravity_smoke"
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            "-Isrc",
            str(ROOT / "tests" / "no_gripper_gravity_smoke.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        cwd=ROOT,
    )
    return binary


class NoGripperGravityTests(unittest.TestCase):
    def test_explicit_zero_and_missing_feedback_config_drop_gripper_mass(self):
        binary = compile_smoke()
        with tempfile.TemporaryDirectory() as directory:
            installed = Path(directory) / "installed.yaml"
            omitted = Path(directory) / "omitted.yaml"
            installed.write_text(
                "gripper_installed: false\ngripper_motor_kg: 0\nee_payload_kg: 0.05\n"
            )
            omitted.write_text("ee_payload_kg: 0\n")
            subprocess.run([str(binary), str(installed), str(omitted)], check=True, cwd=ROOT)

    def test_seed_no_gripper_writes_flag_and_skips_finger_mass(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "gravity_comp.yaml"
            env = os.environ.copy()
            env.pop("OPENARM_NO_GRIPPER", None)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "seed_gravity_comp.py"),
                    "--write",
                    str(target),
                    "--urdf",
                    str(ROOT / "assets" / "v1.urdf"),
                    "--ee-payload",
                    "0.131",
                    "--no-gripper",
                ],
                check=True,
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("无夹爪", completed.stdout)
            self.assertIn("ee_mass (无夹爪，仅额外负载) = 0.1310 kg", completed.stdout)
            params = load_params(target)
            self.assertIs(params["gripper_installed"], False)
            self.assertEqual(params["gripper_motor_kg"], 0.0)
            self.assertEqual(params["ee_payload_kg"], 0.131)

    def test_initializer_skips_gripper_when_env_set(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            header = temp / "openarm" / "can" / "socket" / "openarm.hpp"
            header.parent.mkdir(parents=True)
            header.write_text(
                r"""
#pragma once
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <string>
#include <vector>
namespace openarm::can::socket {
struct FakeComponent {
    std::vector<damiao_motor::Motor> motors;
    int neutral_frames = 0;
    const auto& get_motors() const { return motors; }
    void mit_control_all(const std::vector<damiao_motor::MITParam>& commands) {
        for (const auto& command : commands) {
            if (command.kp != 0 || command.kd != 0 || command.tau != 0)
                throw std::runtime_error("initializer sent non-neutral command");
        }
        ++neutral_frames;
    }
};
class OpenArm {
public:
    bool fd;
    bool enabled = false;
    FakeComponent arm, gripper;
    OpenArm(const std::string&, bool enable_fd) : fd(enable_fd) {}
    void init_arm_motors(const std::vector<damiao_motor::MotorType>& types,
                         const std::vector<uint32_t>& tx, const std::vector<uint32_t>& rx) {
        for (size_t i = 0; i < types.size(); ++i) arm.motors.emplace_back(types[i], tx[i], rx[i]);
    }
    void init_gripper_motor(damiao_motor::MotorType type, uint32_t tx, uint32_t rx) {
        gripper.motors.emplace_back(type, tx, rx);
    }
    void set_callback_mode_all(damiao_motor::CallbackMode) {}
    void enable_all() { enabled = true; }
    void disable_all() { enabled = false; }
    void recv_all(int = 500) {}
    FakeComponent& get_arm() { return arm; }
    FakeComponent& get_gripper() { return gripper; }
};
}
"""
            )
            source = temp / "test.cpp"
            source.write_text(
                r"""
#include <openarm_port/openarm_init.hpp>
#include <cassert>
#include <cstdlib>
#include <memory>
int main() {
    setenv("OPENARM_NO_GRIPPER", "1", 1);
    std::unique_ptr<openarm::can::socket::OpenArm> hardware(
        openarm_init::OpenArmInitializer::initialize_openarm("fake-can", false));
    assert(hardware->arm.motors.size() == 7);
    assert(hardware->gripper.motors.empty());
    assert(hardware->arm.neutral_frames == 20);
    unsetenv("OPENARM_NO_GRIPPER");
    hardware.reset(openarm_init::OpenArmInitializer::initialize_openarm("fake-can", false));
    assert(hardware->gripper.motors.size() == 1);
    assert(hardware->gripper.neutral_frames == 20);
}
"""
            )
            binary = temp / "test"
            subprocess.run(
                [
                    "g++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Wpedantic",
                    "-Werror",
                    "-I",
                    str(temp),
                    "-I",
                    str(ROOT / "src"),
                    "-I",
                    str(ROOT / "third_party" / "openarm_can" / "include"),
                    str(source),
                    str(ROOT / "src" / "openarm_port" / "openarm_init.cpp"),
                    str(ROOT / "third_party" / "openarm_can" / "src" / "openarm" / "damiao_motor" / "dm_motor.cpp"),
                    "-o",
                    str(binary),
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([str(binary)], check=True, cwd=ROOT)


if __name__ == "__main__":
    unittest.main()
