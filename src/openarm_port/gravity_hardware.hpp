#pragma once

#include <controller/gravity_comp.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <vector>

// Register only installed motors. All vendor enable/refresh/disable calls and
// the shared feedback guard then operate on exactly the same hardware set.
template <class Arm>
void initialize_gravity_motors(Arm& arm, const GravityModel& model, const std::string& side) {
    using openarm::damiao_motor::MotorType;
    const std::vector<MotorType> types{
        MotorType::DM8009, MotorType::DM8009, MotorType::DM4340, MotorType::DM4340,
        MotorType::DM4310, MotorType::DM4310, MotorType::DM4310};
    arm.init_arm_motors(types, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
    if (gripper_present_for(model, side)) {
        arm.init_gripper_motor(MotorType::DM4310, 0x08, 0x18);
    }
    arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
}
