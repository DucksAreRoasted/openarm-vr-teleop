#pragma once

#include <openarm/can/socket/openarm.hpp>

#include <chrono>
#include <thread>

namespace openarm_init {

inline bool motor_reported_state(const openarm::damiao_motor::Motor& motor) {
    return motor.last_state_received().time_since_epoch().count() != 0;
}

inline bool gripper_reported_state(openarm::can::socket::OpenArm* arm) {
    if (arm == nullptr) {
        return false;
    }
    const auto motors = arm->get_gripper().get_motors();
    return !motors.empty() && motor_reported_state(motors.front());
}

// Extra polls after the caller's own refresh window. False means the gripper
// motor (send 0x08 / recv 0x18) never answered, which is the removed-gripper case.
inline bool poll_gripper_state(openarm::can::socket::OpenArm* arm, int rounds, int sleep_ms = 5) {
    if (gripper_reported_state(arm)) {
        return true;
    }
    for (int i = 0; i < rounds; ++i) {
        arm->refresh_all();
        arm->recv_all(2000);
        if (gripper_reported_state(arm)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    return false;
}

}  // namespace openarm_init
