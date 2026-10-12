"""Exercise the real initializer with a fake hardware boundary; no CAN sockets."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CAN_ROOT = ROOT / "third_party" / "openarm_can"


class InitializerTransportTests(unittest.TestCase):
    def test_quiet_and_verbose_initialization_both_use_fd(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            header = temp / "openarm/can/socket/openarm.hpp"
            header.parent.mkdir(parents=True)
            header.write_text(r'''
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
        for (size_t i=0;i<types.size();++i) arm.motors.emplace_back(types[i], tx[i], rx[i]);
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
''')
            source = temp / "test.cpp"
            source.write_text(r'''
#include <openarm_port/openarm_init.hpp>
#include <cassert>
#include <memory>
int main() {
    for (bool debug : {false, true}) {
        std::unique_ptr<openarm::can::socket::OpenArm> hardware(
            openarm_init::OpenArmInitializer::initialize_openarm("fake-can", debug));
        assert(hardware->fd);
        assert(hardware->enabled);
        assert(hardware->arm.motors.size() == 7);
        assert(hardware->gripper.motors.size() == 1);
        assert(hardware->arm.neutral_frames == 20);
        assert(hardware->gripper.neutral_frames == 20);
    }
}
''')
            binary = temp / "test"
            subprocess.run([
                "c++", "-std=c++17", "-I", str(temp), "-I", str(ROOT / "src"),
                "-I", str(CAN_ROOT / "include"), str(source),
                str(ROOT / "src/openarm_port/openarm_init.cpp"),
                str(CAN_ROOT / "src/openarm/damiao_motor/dm_motor.cpp"),
                "-o", str(binary),
            ], check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
