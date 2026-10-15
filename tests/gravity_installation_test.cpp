#include <controller/gravity_comp.hpp>
#include <openarm_port/can_feedback.hpp>
#include <openarm_port/gravity_hardware.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>
#include <fstream>
#include <cstdio>

struct Motor {
    std::chrono::steady_clock::time_point received = std::chrono::steady_clock::now();
    int error = 0;
    uint32_t id;
    auto last_state_received() const { return received; }
    int get_error_code() const { return error; }
    uint32_t get_send_can_id() const { return id; }
    uint32_t get_recv_can_id() const { return id + 0x10; }
};
struct Part {
    std::vector<Motor> motors;
    const auto& get_motors() const { return motors; }
};
struct Arm {
    Part joints, gripper;
    Part& get_arm() { return joints; }
    Part& get_gripper() { return gripper; }
    void init_arm_motors(const std::vector<openarm::damiao_motor::MotorType>& types,
                         const std::vector<uint32_t>& ids, const std::vector<uint32_t>& rx) {
        assert(types.size() == 7 && ids.size() == 7 && rx.size() == 7);
        for (auto id : ids) joints.motors.push_back({std::chrono::steady_clock::now(), 0, id});
    }
    void init_gripper_motor(openarm::damiao_motor::MotorType, uint32_t id, uint32_t rx) {
        assert(id == 8 && rx == 0x18);
        gripper.motors.push_back({std::chrono::steady_clock::now(), 0, id});
    }
    void set_callback_mode_all(openarm::damiao_motor::CallbackMode) {}
};

int main(int argc, char** argv) {
    assert(argc == 2);
    // Zero friction is a valid way to isolate gravity while diagnosing drift.
    const std::string path = std::string(argv[1]) + "/zero_friction.yaml";
    {
        std::ofstream file(path);
        file << "left_friction_fc: [0,0,0,0,0,0,0]\n"
             << "left_friction_fv: [0,0,0,0,0,0,0]\n"
             << "right_friction_fc: [0,0,0,0,0,0,0]\n"
             << "right_friction_fv: [0,0,0,0,0,0,0]\n";
    }
    GravityTune lt, rt;
    GravityModel loaded;
    assert(load_gravity_yaml(path, &lt, &rt, &loaded));
    for (size_t j=0; j<7; ++j) {
        for (double dq : {-.5, -.01, 0., .01, .5}) {
            assert(std::abs(compose_tau_ff(.4, lt, j, dq, .95)-.38)<1e-12);
            assert(std::abs(compose_tau_ff(.4, rt, j, dq, .95)-.38)<1e-12);
        }
    }
    std::remove(path.c_str());
    GravityModel model;
    model.gripper_motor_kg = 0.0;
    model.ee_payload_kg = 0.0;
    assert(std::abs(end_effector_mass_for(model, "left") - 2 * 0.03602545343277134) < 1e-12);
    model.right_gripper_present = false;
    model.ee_payload_kg = 0.131;
    assert(end_effector_mass_for(model, "right") == 0.0);
    assert(wrist_camera_mass_for(model, "right") == model.wrist_camera_kg);
    model.right_wrist_camera_present = false;
    assert(wrist_camera_mass_for(model, "right") == 0.0);
    assert(wrist_camera_mass_for(model, "left") == model.wrist_camera_kg);
    Arm left, right;
    initialize_gravity_motors(left, model, "left");
    initialize_gravity_motors(right, model, "right");
    assert(left.gripper.motors.size() == 1);
    assert(right.gripper.motors.empty());
    auto now = std::chrono::steady_clock::now();
    assert(evaluate_openarm_feedback(right, now).ok);
    left.gripper.motors[0].received = {};
    assert(!evaluate_openarm_feedback(left, now).ok);
    // Removing the gripper never exempts any of the seven arm joints.
    right.joints.motors[6].received = {};
    assert(!evaluate_openarm_feedback(right, now).ok);
    right.joints.motors[6].received = now - std::chrono::seconds(1);
    assert(evaluate_openarm_feedback(right, now).fault == CanFeedbackFault::Stale);
    right.joints.motors[6].received = now;
    right.joints.motors[6].error = 2;
    assert(evaluate_openarm_feedback(right, now).fault == CanFeedbackFault::ErrorCode);
}
