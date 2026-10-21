// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// #include <sensor_msgs/msg/joint_state.hpp>
#include <controller/diff.hpp>
#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <io/control_diagnostics.hpp>
#include <io/joint_angle_stream.hpp>
#include <atomic>
#include <deque>
#include <fstream>
#include <hardware/teleop_hardware.hpp>
#include <joint_state_converter.hpp>
#include <memory>
#include <mutex>
#include <numeric>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_constants.hpp>
#include <robot_state.hpp>
#include <safety/safety_filter.hpp>
#include <string>
#include <utility>

class Control {
    std::shared_ptr<TeleopHardware> hardware_;
    std::unique_ptr<JointAngleStream> joint_angle_stream_;

    double Ts_;
    int role_;

    size_t arm_motor_num_;
    size_t hand_motor_num_;

    Differentiator *differentiator_;
    OpenArmJointConverter *openarmjointconverter_;
    OpenArmJointConverter *openarmgripperjointconverter_;

    std::shared_ptr<RobotSystemState> robot_state_;

    std::string arm_type_;

    Dynamics *dynamics_f_;
    Dynamics *dynamics_l_;
    std::shared_ptr<Dynamics> follower_gravity_;
    GravityTune follower_gravity_tune_;
    double follower_gravity_gain_ = 0.0;
    double follower_gravity_ramp_sec_ = 1.0;
    double follower_gravity_elapsed_ = 0.0;
    std::array<double, 7> diagnostic_gravity_{}, diagnostic_gravity_requested_{};
    std::unique_ptr<ControlDiagnostics> diagnostics_;
    std::array<double, 7> diagnostic_source_{};
    int diagnostic_phase_ = 0;
    double diagnostic_time_ = -1;
    std::unique_ptr<SafetyFilter> safety_filter_;
    openarm::can::socket::OpenArm* can_bus_ = nullptr;
    std::atomic<bool> output_disabled_{false};
    mutable std::mutex telemetry_mutex_;
    std::string output_disabled_reason_;
    std::vector<double> last_command_efforts_;
    // Exact MIT q/dq/tau that were last sent to motors (post safety / gripper hold).
    std::vector<JointState> last_mit_command_;

    double oblique_coordinates_force;
    double oblique_coordinates_position;

    // for easy logging
    // std::vector<std::pair<double, double>> velocity_log_;  // (differ_velocity, motor_velocity)
    // std::string log_file_path_ = "../data/velocity_comparison.csv";
    static constexpr int VEL_WINDOW_SIZE = 10;
    static constexpr double VIB_THRESHOLD = 0.7;  // [rad/s]
    std::deque<double> velocity_buffer_[NJOINTS];
    std::atomic_bool gripper_contact_hold_active_ {false};
    std::atomic_bool gripper_slip_boost_active_ {false};
    double gripper_hold_position_ = 0.0;
    double gripper_contact_timer_ = 0.0;
    double gripper_release_timer_ = 0.0;
    double gripper_slip_boost_timer_ = 0.0;
    double gripper_last_raw_target_ = 0.0;
    double gripper_close_sign_ = 0.0;

public:
    Control(std::shared_ptr<TeleopHardware> hardware, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            size_t arm_joint_num, size_t hand_motor_num);
    Control(std::shared_ptr<TeleopHardware> hardware, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            std::string arm_type, size_t arm_joint_num, size_t hand_motor_num);
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            size_t arm_joint_num, size_t hand_motor_num);
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            std::string arm_type, size_t arm_joint_num, size_t hand_motor_num);
    ~Control();

    std::shared_ptr<RobotSystemState> response_;
    std::shared_ptr<RobotSystemState> reference_;

    std::vector<double> Dn_, Kp_, Kd_, Fc_, k_, Fv_, Fo_;

    // bool Setup(void);
    void Setstate(int state);
    void Shutdown(void);

    void SetParameter(const std::vector<double> &Kp, const std::vector<double> &Kd,
                      const std::vector<double> &Fc, const std::vector<double> &k,
                      const std::vector<double> &Fv, const std::vector<double> &Fo);
    void SetSafetyFilter(std::unique_ptr<SafetyFilter> safety_filter);
    void ResetSafetyFilter(const std::vector<JointState> &initial_cmd);
    // Configure before starting the control thread. Uses measured q, gravity only.
    void SetFollowerGravity(std::shared_ptr<Dynamics> dynamics, const GravityTune& tune,
                            double gain, double ramp_sec = 1.0);
    // Real CAN bus used for feedback age / error_code checks. Simulation leaves this unset.
    void BindCanBus(openarm::can::socket::OpenArm* bus);
    // Set on the control thread; phase 0=teleop, 1=alignment, 2=play, 3=hold.
    void SetDiagnosticContext(const std::array<double, 7>& source, int phase, double time) {
        diagnostic_source_ = source; diagnostic_phase_ = phase; diagnostic_time_ = time;
    }
    bool OutputDisabled() const;
    std::string OutputDisabledReason() const;
    std::vector<double> GetLastCommandEfforts() const;
    // Joint-space copy of the last MIT frame actually sent (q, dq, tau).
    std::vector<JointState> GetLastMitCommand() const;
    bool GripperContactHoldActive() const;
    bool GripperSlipBoostActive() const;

    bool AdjustPosition(void);
    bool AdjustPositionTo(const std::vector<JointState> &goal);

    // Read motors and set references to the measured pose BEFORE any MIT step.
    // Prevents the default-zero reference from yanking the arm on first enable.
    bool SeedHoldMeasured();

    // Compute torque based on bilateral
    bool bilateral_step();
    bool unilateral_step();

private:
    // NOTE! Control() class operates on "joints", while the underlying
    // classes operates on "actuators". The following functions map
    // joints to actuators.

    void ComputeJointPosition(const double *motor_position, double *joint_position);
    void ComputeJointVelocity(const double *motor_velocity, double *joint_velocity);
    void ComputeMotorTorque(const double *joint_torque, double *motor_torque);

    // void ComputeFriction(const double *velocity, double *friction);
    void ComputeFriction(const double *velocity, double *friction, size_t index);
    void ComputeGravity(const double *position, double *gravity);
    bool DetectVibration(const double *velocity, bool *what_axis);

    // True when this step must not send another torque or Kp frame.
    bool feedback_blocks_output();
    bool apply_follower_gravity(const std::vector<JointState>& measured,
                                std::vector<JointState>* reference, double dt);
    void latch_output_disabled(const std::string& reason);
    void send_zero_gain_frame(const std::vector<JointState>& joints);
    bool latch_safety_stop(const SafetyStatus& status, const std::vector<JointState>& safe_ref);
    bool apply_safety_filter(double dt, const std::vector<JointState>& arm_state,
                             const std::vector<JointState>& hand_state,
                             std::vector<JointState>* arm_ref, std::vector<JointState>* hand_ref);
};
