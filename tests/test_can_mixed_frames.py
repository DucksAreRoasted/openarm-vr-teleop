"""CAN and CAN FD feedback decoding without opening sockets or enabling motors."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CAN_ROOT = ROOT / "third_party" / "openarm_can"


class MixedCanFramesTests(unittest.TestCase):
    def test_classic_reply_updates_fd_motor_state(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "mixed_frames.cpp"
            binary = Path(directory) / "mixed_frames"
            source.write_text(r'''
#include <openarm/canbus/can_socket.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <cassert>
#include <cstring>
int main() {
    using openarm::canbus::CANSocket;
    using namespace openarm::damiao_motor;
    Motor motor(MotorType::DM8009, 1, 0x11);
    DMCANDevice device(motor, CAN_SFF_MASK, true);
    assert(motor.last_state_received().time_since_epoch().count() == 0);

    can_frame classic{};
    classic.can_id = 0x11;
    classic.can_dlc = 8;
    const unsigned char payload[8] = {0x11, 0x80, 0, 0x80, 0x08, 0, 30, 31};
    std::memcpy(classic.data, payload, 8);
    canfd_frame normalized{};
    std::memcpy(&normalized, &classic, CAN_MTU);
    assert(CANSocket::normalize_received_frame(normalized, CAN_MTU));
    assert(normalized.can_id == 0x11 && normalized.len == 8 && normalized.flags == 0);
    assert(std::memcmp(normalized.data, payload, 8) == 0);
    device.callback(normalized);
    assert(motor.last_state_received().time_since_epoch().count() != 0);
    assert(motor.get_error_code() == 1);
    assert(motor.get_state_tmos() == 30 && motor.get_state_trotor() == 31);

    canfd_frame fd{};
    fd.can_id = 0x11; fd.len = 8; fd.flags = CANFD_BRS;
    std::memcpy(fd.data, payload, 8);
    fd.data[6] = 32;
    assert(CANSocket::normalize_received_frame(fd, CANFD_MTU));
    assert(fd.flags == CANFD_BRS);
    device.callback(fd);
    assert(motor.get_state_tmos() == 32);
    assert(!CANSocket::normalize_received_frame(fd, 0));
    assert(!CANSocket::normalize_received_frame(fd, CAN_MTU-1));
    fd.len = 9;
    assert(!CANSocket::normalize_received_frame(fd, CAN_MTU));
    fd.len = 65;
    assert(!CANSocket::normalize_received_frame(fd, CANFD_MTU));
}
''')
            sources = [
                "canbus/can_socket.cpp",
                "damiao_motor/dm_motor.cpp",
                "damiao_motor/dm_motor_control.cpp",
                "damiao_motor/dm_motor_device.cpp",
            ]
            subprocess.run([
                "c++", "-std=c++17", "-I", str(CAN_ROOT / "include"), str(source),
                *(str(CAN_ROOT / "src" / "openarm" / name) for name in sources),
                "-o", str(binary),
            ], check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
