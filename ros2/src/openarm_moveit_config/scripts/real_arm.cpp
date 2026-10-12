// FollowJointTrajectory on the real OpenArm CAN arms, and publish /joint_states.
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <controller/s_curve.hpp>
#include <openarm_port/gripper_probe.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

// Position gains for trajectory tracking (Nm/rad, Nm·s/rad).
constexpr std::array<double, 7> KP{55.0, 55.0, 55.0, 55.0, 16.0, 14.0, 14.0};
constexpr std::array<double, 7> KD{1.3, 1.3, 1.3, 1.3, 0.35, 0.28, 0.35};
// Hold-only integral (Nm/(rad·s)) — kills static droop that PD+G(q) leaves.
constexpr std::array<double, 7> KI{12.0, 12.0, 10.0, 10.0, 5.0, 4.0, 4.0};
constexpr std::array<double, 7> kIntegralTauLimit{6.0, 6.0, 4.0, 4.0, 1.5, 1.2, 1.2};
// MoveIt tracking τ_ff clamps (wider than zero-g defaults so gravity isn't clipped).
constexpr std::array<double, 7> kMoveItTauFfLimits{25.0, 25.0, 20.0, 20.0, 5.0, 5.0, 5.0};
constexpr double kControlDt = 0.01;
constexpr double kHoldVelThresh = 0.12;  // rad/s — only integrate when nearly still
constexpr double GRIPPER_KP = 10.0;
constexpr double GRIPPER_KD = 0.5;

using Follow = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

#ifndef OPENARM_URDF_PATH
#define OPENARM_URDF_PATH ""
#endif
#ifndef OPENARM_ROOT
#define OPENARM_ROOT ""
#endif

void apply_gravity_model(Dynamics* dynamics, const GravityModel& model, const std::string& side,
                         bool gripper_on, bool reinit_links = false) {
    if (dynamics == nullptr) {
        return;
    }
    const auto& scales = link_mass_scales_for(model, side);
    dynamics->SetLinkMassScales(scales);
    dynamics->SetLinkComDeltas(link_com_deltas_for(model, side));
    if (reinit_links) {
        if (!dynamics->ReinitWithMassScales(scales)) {
            RCLCPP_WARN(rclcpp::get_logger("real_arm"),
                        "ReinitWithMassScales failed for %s — still applying distal/camera knobs",
                        side.c_str());
        }
    }
    dynamics->SetUrdfGravityScale(model.urdf_scale);
    dynamics->SetEndEffectorPointMass(end_effector_point_mass_kg(model, gripper_on), 0.0, 0.0,
                                      kOpenArmFingerComZ);
    const auto& off =
        (side == "left") ? model.wrist_camera_offset_left : model.wrist_camera_offset_right;
    dynamics->SetWristCameraMass(model.wrist_camera_kg, side == "left", off[0], off[1], off[2]);
    dynamics->SetDistalMassGains(model.distal_ee_gain, model.distal_cam_gain);
}

void ensure_friction(GravityTune* tune) {
    double sum = 0.0;
    for (double v : tune->fc) {
        sum += std::abs(v);
    }
    if (sum < 1e-6) {
        seed_default_friction(tune);
    }
}

bool load_gravity_yaml_file(const std::string& path, GravityTune* left, GravityTune* right,
                            GravityModel* model) {
    if (!load_gravity_yaml(path, left, right, model)) {
        return false;
    }
    ensure_friction(left);
    ensure_friction(right);
    return true;
}

std::atomic<bool> g_reload_gravity{false};

void on_hup(int) { g_reload_gravity = true; }

double seconds_of(const builtin_interfaces::msg::Duration& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::array<double, 7> param_array7(rclcpp::Node& node, const std::string& name,
                                   const std::array<double, 7>& fallback) {
    std::vector<double> defaults(fallback.begin(), fallback.end());
    std::vector<double> values = defaults;
    try {
        values = node.declare_parameter<std::vector<double>>(name, defaults);
    } catch (const std::exception& error) {
        RCLCPP_WARN(node.get_logger(), "参数 %s 读取失败（%s），用默认值", name.c_str(),
                    error.what());
        values = defaults;
        try {
            if (!node.has_parameter(name)) {
                node.declare_parameter(name, defaults);
            }
        } catch (...) {
        }
    }
    std::array<double, 7> out = fallback;
    for (size_t index = 0; index < out.size() && index < values.size(); ++index) {
        out[index] = values[index];
    }
    return out;
}

struct Trajectory {
    bool active = false;
    // Virtual trajectory time (seconds). Advances slower / freezes when tracking
    // error is large so external force cannot make q_cmd race ahead of the arm.
    double virtual_time = 0.0;
    std::vector<double> times;
    std::vector<std::array<double, 7>> points;
    // Optional per-sample joint velocities (same length as points when filled).
    std::vector<std::array<double, 7>> velocities;
};

// Keep MoveIt knot times/poses; densify each segment with an S-shaped mix so
// acceleration eases in/out without retiming the whole path (avoids timeout /
// tracking abort from a longer schedule, and avoids stop-go rest-to-rest).
Trajectory densify_trajectory_scurve(const Trajectory& input, double sample_dt) {
    Trajectory out;
    out.active = input.active;
    if (input.points.size() < 2 || input.times.size() != input.points.size()) {
        return input;
    }

    sample_dt = std::clamp(sample_dt, 0.002, 0.05);
    out.times.push_back(input.times.front());
    out.points.push_back(input.points.front());

    for (size_t i = 1; i < input.points.size(); ++i) {
        const double t0 = input.times[i - 1];
        const double t1 = input.times[i];
        const double T = std::max(t1 - t0, 1e-4);
        const int steps = std::max(1, static_cast<int>(std::ceil(T / sample_dt)));
        for (int step = 1; step <= steps; ++step) {
            const double raw = static_cast<double>(step) / static_cast<double>(steps);
            const double mix = openarm::scurve::unit_smooth(raw);
            const double mix_dot = openarm::scurve::unit_smooth_dot(raw);  // d(mix)/d(raw)
            std::array<double, 7> q{};
            std::array<double, 7> dq{};
            for (size_t j = 0; j < 7; ++j) {
                const double delta = input.points[i][j] - input.points[i - 1][j];
                q[j] = input.points[i - 1][j] + mix * delta;
                // dq/dt = d(mix)/d(raw) * d(raw)/dt * delta
                dq[j] = mix_dot * (delta / T);
            }
            out.times.push_back(t0 + raw * T);
            out.points.push_back(q);
            out.velocities.push_back(dq);
        }
    }
    if (!out.velocities.empty()) {
        out.velocities.back().fill(0.0);
    }
    return out;
}

// Tracking / recovery thresholds (rad).
constexpr double kStallError = 0.18;     // freeze time progression
constexpr double kSlowError = 0.10;      // slow time progression
constexpr double kSettleError = 0.05;    // consider waypoint / goal reached
constexpr double kStartWarpError = 0.05; // rewrite traj start to measured
constexpr double kStartRejectError = 0.70; // plan too far from reality

class ErrorLogger {
public:
    explicit ErrorLogger(const std::string& directory) {
        if (directory.empty()) {
            return;
        }
        std::filesystem::create_directories(directory);
        const auto now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream name;
        name << directory << "/gravity_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        path_ = name.str();
        file_.open(path_);
        if (!file_) {
            path_.clear();
            return;
        }
        file_ << "t,side,traj";
        for (int joint = 1; joint <= 7; ++joint) {
            file_ << ",q_cmd" << joint << ",q_meas" << joint << ",e" << joint << ",dq" << joint
                  << ",dq_meas" << joint << ",g_model" << joint << ",tau_ff" << joint << ",mtau"
                  << joint;
        }
        file_ << "\n";
        file_.flush();
    }

    const std::string& path() const { return path_; }

    void write(double t, const std::string& side, bool traj, const std::array<double, 7>& q_cmd,
               const std::array<double, 7>& q_meas, const std::array<double, 7>& dq_cmd,
               const std::array<double, 7>& dq_meas, const std::array<double, 7>& g_model,
               const std::array<double, 7>& tau_ff, const std::array<double, 7>& mtau) {
        if (!file_) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        file_ << std::fixed << std::setprecision(6) << t << "," << side << "," << (traj ? 1 : 0);
        for (int joint = 0; joint < 7; ++joint) {
            file_ << "," << q_cmd[joint] << "," << q_meas[joint] << ","
                  << (q_cmd[joint] - q_meas[joint]) << "," << dq_cmd[joint] << "," << dq_meas[joint]
                  << "," << g_model[joint] << "," << tau_ff[joint] << "," << mtau[joint];
        }
        file_ << "\n";
    }

    void flush() {
        if (file_) {
            std::lock_guard<std::mutex> lock(mutex_);
            file_.flush();
        }
    }

private:
    std::mutex mutex_;
    std::ofstream file_;
    std::string path_;
};

class ArmBus {
public:
    ArmBus(const std::string& can, const std::string& side, const std::string& urdf_path,
           GravityTune tune, GravityModel model, double tracking_ff_gain,
           std::shared_ptr<ErrorLogger> logger)
        : side_(side),
          tune_(tune),
          model_(model),
          tracking_ff_gain_(tracking_ff_gain),
          logger_(std::move(logger)) {
        hardware_ = std::make_unique<openarm::can::socket::OpenArm>(can, true);
        const std::vector<openarm::damiao_motor::MotorType> types{
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310};
        hardware_->init_arm_motors(types, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
                                   {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
        const bool configured = gripper_configured_installed(model_);
        if (configured) {
            hardware_->init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);
        }
        hardware_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        hardware_->enable_all();
        enabled_ = true;
        bool saw_gripper = false;
        for (int cycle = 0; cycle < 20; ++cycle) {
            hardware_->refresh_all();
            hardware_->recv_all(2000);
            if (configured && openarm_init::gripper_reported_state(hardware_.get())) {
                saw_gripper = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (configured && !saw_gripper) {
            saw_gripper = openarm_init::poll_gripper_state(hardware_.get(), 180);
        }
        gripper_on_ = configured && saw_gripper;
        if (!gripper_on_) {
            RCLCPP_WARN(rclcpp::get_logger("real_arm"),
                        "%s 按无夹爪继续：不下发 0x08，重力不含指尖和夹爪电机质量。%s",
                        side_.c_str(),
                        configured ? "总线上没有 0x18 反馈。"
                                   : "OPENARM_NO_GRIPPER=1 或 gripper_installed: false。");
        }
        dynamics_ = std::make_unique<Dynamics>(urdf_path, "openarm_body_link0",
                                               "openarm_" + side_ + "_link7");
        dynamics_->SetLinkMassScales(link_mass_scales_for(model_, side_));
        dynamics_->SetLinkComDeltas(link_com_deltas_for(model_, side_));
        if (!dynamics_->Init()) {
            dynamics_.reset();
        } else {
            apply_gravity_model(dynamics_.get(), model_, side_, gripper_on_);
        }
        read();
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = measured_;
        velocity_.fill(0.0);
        gripper_hold_ = gripper_;
        send_locked(0.0);
    }

    ~ArmBus() {
        if (hardware_ && enabled_) {
            hardware_->disable_all();
            hardware_->recv_all(1000);
        }
    }

    bool has_gravity() const { return dynamics_ != nullptr; }

    void set_tune(const GravityTune& tune) {
        std::lock_guard<std::mutex> lock(mutex_);
        tune_ = tune;
    }

    void set_model(const GravityModel& model) {
        std::lock_guard<std::mutex> lock(mutex_);
        model_ = model;
        if (!gripper_configured_installed(model_)) {
            gripper_on_ = false;
        }
        apply_gravity_model(dynamics_.get(), model_, side_, gripper_on_, /*reinit_links=*/true);
    }

    void set_tracking_ff_gain(double gain) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracking_ff_gain_ = std::clamp(gain, 0.5, 1.5);
    }

    std::array<double, 7> measured() {
        std::lock_guard<std::mutex> lock(mutex_);
        return measured_;
    }

    void hold_here() {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = measured_;
        velocity_.fill(0.0);
        integral_tau_.fill(0.0);
    }

    void spin_once(double t_sec) {
        std::lock_guard<std::mutex> lock(mutex_);
        send_locked(t_sec);
        hardware_->recv_all(1000);
        read_locked();
    }

    void fill(sensor_msgs::msg::JointState* message) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int index = 0; index < 7; ++index) {
            message->name.push_back("openarm_" + side_ + "_joint" + std::to_string(index + 1));
            message->position.push_back(measured_[index]);
        }
        // Publish a planning-safe finger opening. Closed jaws self-collide in the
        // URDF meshes and used to make the whole start state invalid for MoveIt.
        // Gripper motor is still held via gripper_hold_; arm planning groups do
        // not command the fingers.
        constexpr double kPlanningFingerOpen = 0.02;
        for (const char* joint : {"finger_joint1", "finger_joint2"}) {
            message->name.push_back("openarm_" + side_ + "_" + joint);
            message->position.push_back(kPlanningFingerOpen);
        }
    }

    // Returns planned duration in seconds (after optional S-curve densify).
    double set_trajectory(Trajectory trajectory) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (openarm::scurve::env_flag_enabled("OPENARM_MOVEIT_SCURVE", true) &&
            trajectory.points.size() >= 2) {
            trajectory_ = densify_trajectory_scurve(trajectory, kControlDt);
        } else {
            trajectory_ = std::move(trajectory);
            trajectory_.velocities.clear();
        }
        trajectory_.virtual_time = 0.0;
        last_track_wall_ = std::chrono::steady_clock::now();
        if (!trajectory_.points.empty()) {
            command_ = trajectory_.points.front();
            velocity_.fill(0.0);
        }
        // Fresh move: clear integral so it doesn't fight the new path.
        integral_tau_.fill(0.0);
        if (trajectory_.times.size() < 2) {
            return 0.0;
        }
        return std::max(0.0, trajectory_.times.back() - trajectory_.times.front());
    }

    void clear_trajectory() {
        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_.active = false;
        velocity_.fill(0.0);
    }

    void track(const rclcpp::Time& /*now*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trajectory_.active || trajectory_.points.empty()) {
            return;
        }
        const auto wall_now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(wall_now - last_track_wall_).count();
        last_track_wall_ = wall_now;
        dt = std::clamp(dt, 0.0, 0.05);

        // Error vs the command we are already asking for.
        double err = 0.0;
        for (size_t i = 0; i < command_.size(); ++i) {
            err = std::max(err, std::abs(command_[i] - measured_[i]));
        }
        // Time-warp: freeze / slow when the human (or obstacle) holds the arm.
        double rate = 1.0;
        if (err > kStallError) {
            rate = 0.0;
        } else if (err > kSlowError) {
            rate = 0.25;
        }
        trajectory_.virtual_time += dt * rate;
        const double t_end = trajectory_.times.back();
        if (trajectory_.virtual_time > t_end) {
            trajectory_.virtual_time = t_end;
        }
        sample(trajectory_.virtual_time, &command_, &velocity_);

        if (trajectory_.virtual_time >= t_end) {
            velocity_.fill(0.0);
            // Only finish after the arm actually catches the final point.
            double final_err = 0.0;
            for (size_t i = 0; i < command_.size(); ++i) {
                final_err = std::max(final_err, std::abs(command_[i] - measured_[i]));
            }
            if (final_err < kSettleError) {
                trajectory_.active = false;
            }
        }
    }

    bool trajectory_done() {
        std::lock_guard<std::mutex> lock(mutex_);
        return !trajectory_.active;
    }

    double max_tracking_error() {
        std::lock_guard<std::mutex> lock(mutex_);
        double err = 0.0;
        for (size_t i = 0; i < command_.size(); ++i) {
            err = std::max(err, std::abs(command_[i] - measured_[i]));
        }
        return err;
    }

private:
    void read() {
        std::lock_guard<std::mutex> lock(mutex_);
        read_locked();
    }

    void read_locked() {
        const auto motors = hardware_->get_arm().get_motors();
        for (size_t index = 0; index < motors.size() && index < measured_.size(); ++index) {
            measured_[index] = motors[index].get_position();
            measured_vel_[index] = motors[index].get_velocity();
            measured_tau_[index] = motors[index].get_torque();
        }
        if (gripper_on_) {
            const auto gripper = hardware_->get_gripper().get_motors();
            if (!gripper.empty()) {
                gripper_ = gripper.front().get_position();
            }
        }
    }

    void send_locked(double t_sec) {
        std::array<double, 7> gravity{};
        std::array<double, 7> tau_ff{};
        if (dynamics_) {
            dynamics_->GetGravity(measured_.data(), gravity.data());
        }
        const bool holding = !trajectory_.active;
        std::vector<openarm::damiao_motor::MITParam> arm;
        arm.reserve(7);
        for (size_t index = 0; index < command_.size(); ++index) {
            // Tracking uses a dedicated gain (often >1) so residual gravity is
            // cancelled under PD — zero-g yaml ff_gain~0.95 is intentionally low.
            const double grav =
                tracking_ff_gain_ * (tune_.scale[index] * gravity[index] + tune_.bias[index]);
            const double fric = friction_torque(measured_vel_[index], tune_.fc[index],
                                               tune_.fv[index], tune_.fk[index]);
            double tau = grav + fric;

            const double err = command_[index] - measured_[index];
            const bool slow = std::abs(measured_vel_[index]) < kHoldVelThresh &&
                              std::abs(velocity_[index]) < kHoldVelThresh;
            if (holding && slow) {
                integral_tau_[index] += KI[index] * err * kControlDt;
                integral_tau_[index] = std::clamp(integral_tau_[index], -kIntegralTauLimit[index],
                                                  kIntegralTauLimit[index]);
            } else {
                // Leak while moving so integral does not accumulate on the way.
                integral_tau_[index] *= 0.92;
            }
            tau += integral_tau_[index];

            const double lim =
                (index < kMoveItTauFfLimits.size()) ? kMoveItTauFfLimits[index] : 20.0;
            tau_ff[index] = std::clamp(tau, -lim, lim);
            arm.push_back(
                {KP[index], KD[index], command_[index], velocity_[index], tau_ff[index]});
        }
        hardware_->get_arm().mit_control_all(arm);
        if (gripper_on_) {
            hardware_->get_gripper().mit_control_all(
                {{GRIPPER_KP, GRIPPER_KD, gripper_hold_, 0.0, 0.0}});
        }
        if (logger_ && (++log_div_ % 2) == 0) {
            logger_->write(t_sec, side_, trajectory_.active, command_, measured_, velocity_,
                           measured_vel_, gravity, tau_ff, measured_tau_);
        }
    }

    void sample(double elapsed, std::array<double, 7>* position,
                std::array<double, 7>* velocity) const {
        const auto& times = trajectory_.times;
        const auto& points = trajectory_.points;
        const auto& vels = trajectory_.velocities;
        const bool have_vel = vels.size() == points.size();
        if (elapsed <= times.front()) {
            *position = points.front();
            *velocity = have_vel ? vels.front() : std::array<double, 7>{};
            if (!have_vel) {
                velocity->fill(0.0);
            }
            return;
        }
        if (elapsed >= times.back()) {
            *position = points.back();
            velocity->fill(0.0);
            return;
        }
        auto upper = std::upper_bound(times.begin(), times.end(), elapsed);
        const size_t next = static_cast<size_t>(upper - times.begin());
        const size_t prev = next - 1;
        const double span = times[next] - times[prev];
        const double mix = span > 1e-6 ? (elapsed - times[prev]) / span : 1.0;
        for (size_t index = 0; index < position->size(); ++index) {
            (*position)[index] =
                points[prev][index] + mix * (points[next][index] - points[prev][index]);
            if (have_vel) {
                (*velocity)[index] =
                    vels[prev][index] + mix * (vels[next][index] - vels[prev][index]);
            } else {
                (*velocity)[index] =
                    span > 1e-6 ? (points[next][index] - points[prev][index]) / span : 0.0;
            }
        }
    }

    std::string side_;
    GravityTune tune_;
    GravityModel model_;
    double tracking_ff_gain_ = 1.08;
    std::shared_ptr<ErrorLogger> logger_;
    std::unique_ptr<openarm::can::socket::OpenArm> hardware_;
    std::unique_ptr<Dynamics> dynamics_;
    std::mutex mutex_;
    std::array<double, 7> measured_{};
    std::array<double, 7> measured_vel_{};
    std::array<double, 7> measured_tau_{};
    std::array<double, 7> command_{};
    std::array<double, 7> velocity_{};
    std::array<double, 7> integral_tau_{};
    double gripper_ = 0.0;
    double gripper_hold_ = 0.0;
    bool gripper_on_ = true;
    bool enabled_ = false;
    int log_div_ = 0;
    Trajectory trajectory_;
    std::chrono::steady_clock::time_point last_track_wall_{};
};

class RealArm : public rclcpp::Node {
public:
    RealArm() : Node("openarm_real_arm") {
        const auto left = declare_parameter<std::string>("left_can", "can1");
        const auto right = declare_parameter<std::string>("right_can", "can0");
        const auto urdf = declare_parameter<std::string>("urdf_path", OPENARM_URDF_PATH);
        const bool log_errors = declare_parameter<bool>("log_errors", true);
        std::string default_log =
            std::string(OPENARM_ROOT).empty()
                ? std::string()
                : (std::string(OPENARM_ROOT) + "/recordings/moveit_gravity");
        std::string log_dir = declare_parameter<std::string>("error_log_dir", default_log);
        if (log_dir.empty()) {
            log_dir = default_log;
        }
        GravityTune left_tune;
        GravityTune right_tune;
        left_tune.scale = param_array7(*this, "left_gravity_scale", left_tune.scale);
        left_tune.bias = param_array7(*this, "left_gravity_bias", left_tune.bias);
        right_tune.scale = param_array7(*this, "right_gravity_scale", right_tune.scale);
        right_tune.bias = param_array7(*this, "right_gravity_bias", right_tune.bias);
        seed_default_friction(&left_tune);
        seed_default_friction(&right_tune);
        left_tune.fc = param_array7(*this, "left_friction_fc", left_tune.fc);
        left_tune.fv = param_array7(*this, "left_friction_fv", left_tune.fv);
        left_tune.fk = param_array7(*this, "left_friction_k", left_tune.fk);
        right_tune.fc = param_array7(*this, "right_friction_fc", right_tune.fc);
        right_tune.fv = param_array7(*this, "right_friction_fv", right_tune.fv);
        right_tune.fk = param_array7(*this, "right_friction_k", right_tune.fk);
        GravityModel model;
        model.urdf_scale = declare_parameter<double>("urdf_gravity_scale", 1.0);
        model.ee_payload_kg = declare_parameter<double>("ee_payload_kg", 0.0);
        // Zero-g style gain (yaml often ~0.95). MoveIt uses tracking_g_scale instead.
        model.ff_gain = declare_parameter<double>("g_scale", 1.05);
        try {
            model.ff_gain = declare_parameter<double>("ff_gain", model.ff_gain);
        } catch (...) {
        }
        model.tracking_ff_gain = declare_parameter<double>("tracking_g_scale", 1.08);
        model.left_link_mass_scale =
            param_array7(*this, "left_link_mass_scale", model.left_link_mass_scale);
        model.right_link_mass_scale =
            param_array7(*this, "right_link_mass_scale", model.right_link_mass_scale);
        gravity_yaml_ = declare_parameter<std::string>(
            "gravity_yaml",
            std::string(OPENARM_ROOT).empty()
                ? std::string()
                : (std::string(OPENARM_ROOT) + "/config/gravity_comp.yaml"));

        // Load tuned yaml BEFORE constructing ArmBus. Previously the path was only
        // used for hot-reload, so distal/camera/urdf_scale never applied at startup.
        if (!gravity_yaml_.empty()) {
            if (load_gravity_yaml_file(gravity_yaml_, &left_tune, &right_tune, &model)) {
                RCLCPP_INFO(get_logger(),
                            "已加载重力 yaml：%s urdf_scale=%.3f gripper_motor=%.3f camera=%.3f "
                            "g_scale=%.3f distal_ee=[%.2f..%.2f]",
                            gravity_yaml_.c_str(), model.urdf_scale, model.gripper_motor_kg,
                            model.wrist_camera_kg, model.ff_gain, model.distal_ee_gain[0],
                            model.distal_ee_gain[6]);
                try {
                    yaml_mtime_ = std::filesystem::last_write_time(gravity_yaml_);
                } catch (...) {
                }
            } else {
                RCLCPP_WARN(get_logger(),
                            "读不了重力 yaml：%s — 使用 ROS 参数/默认值（补偿会明显偏弱）",
                            gravity_yaml_.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "未设置 gravity_yaml，重力补偿仅用默认参数");
        }

        std::shared_ptr<ErrorLogger> logger;
        if (log_errors) {
            logger = std::make_shared<ErrorLogger>(log_dir);
            if (!logger->path().empty()) {
                RCLCPP_INFO(get_logger(), "误差/重力日志：%s", logger->path().c_str());
            } else {
                RCLCPP_WARN(get_logger(), "打不开误差日志目录：%s", log_dir.c_str());
                logger.reset();
            }
        }

        RCLCPP_INFO(get_logger(), "连接真机 CAN：左 %s，右 %s", left.c_str(), right.c_str());
        RCLCPP_INFO(get_logger(),
                    "重力模型：urdf_scale=%.3f configured_ee=%.4f kg camera=%.3f kg "
                    "tracking_g_scale=%.3f (+ hold integral)。夹爪不在总线上时会去掉指尖和夹爪电机质量",
                    model.urdf_scale,
                    end_effector_point_mass_kg(model, gripper_configured_installed(model)),
                    model.wrist_camera_kg, model.tracking_ff_gain);
        arms_.emplace("left", std::make_unique<ArmBus>(left, "left", urdf, left_tune, model,
                                                       model.tracking_ff_gain, logger));
        arms_.emplace("right", std::make_unique<ArmBus>(right, "right", urdf, right_tune, model,
                                                        model.tracking_ff_gain, logger));
        logger_ = logger;
        if (arms_["left"]->has_gravity() && arms_["right"]->has_gravity()) {
            RCLCPP_INFO(get_logger(),
                        "已启用重力补偿+到位积分（消除静态误差；URDF=%s）", urdf.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "重力补偿初始化失败，抬臂会明显下坠/偏软");
        }
        publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        // Separate callback groups so left/right FollowJointTrajectory can run in parallel
        // under MultiThreadedExecutor (single-threaded spin would serialize them).
        for (const auto& side : {"left", "right"}) {
            action_groups_[side] =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            servers_[side] = rclcpp_action::create_server<Follow>(
                this, std::string(side) + "_arm_controller/follow_joint_trajectory",
                std::bind(&RealArm::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&RealArm::handle_cancel, this, std::placeholders::_1),
                std::bind(&RealArm::handle_execute, this, std::placeholders::_1),
                rcl_action_server_get_default_options(), action_groups_[side]);
        }
        std::signal(SIGHUP, on_hup);
        started_ = std::chrono::steady_clock::now();
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        RCLCPP_INFO(get_logger(), "真机已保持当前姿态，可以规划。精调后点窗口「根据日志精调重力」可热加载。");
    }

    ~RealArm() override {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        if (logger_) {
            logger_->flush();
        }
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const Follow::Goal> goal) {
        if (goal->trajectory.points.empty()) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute(const std::shared_ptr<GoalHandle> goal_handle) {
        const auto goal = goal_handle->get_goal();
        ArmBus* arm = arm_for(*goal);
        if (arm == nullptr) {
            goal_handle->abort(std::make_shared<Follow::Result>());
            return;
        }
        std::vector<int> map(7, -1);
        for (size_t index = 0; index < goal->trajectory.joint_names.size(); ++index) {
            const auto& joint = goal->trajectory.joint_names[index];
            const auto pos = joint.rfind("joint");
            if (pos == std::string::npos || pos + 5 >= joint.size()) {
                continue;
            }
            const int number = std::atoi(joint.c_str() + pos + 5);
            if (number >= 1 && number <= 7) {
                map[number - 1] = static_cast<int>(index);
            }
        }
        for (int slot : map) {
            if (slot < 0) {
                RCLCPP_ERROR(get_logger(), "轨迹里缺少 7 个手臂关节");
                goal_handle->abort(std::make_shared<Follow::Result>());
                return;
            }
        }
        Trajectory trajectory;
        trajectory.active = true;
        trajectory.virtual_time = 0.0;
        const auto measured = arm->measured();
        for (const auto& point : goal->trajectory.points) {
            if (static_cast<int>(point.positions.size()) <=
                *std::max_element(map.begin(), map.end())) {
                goal_handle->abort(std::make_shared<Follow::Result>());
                return;
            }
            std::array<double, 7> command{};
            for (int joint = 0; joint < 7; ++joint) {
                command[joint] = point.positions[map[joint]];
            }
            trajectory.times.push_back(std::max(0.0, seconds_of(point.time_from_start)));
            trajectory.points.push_back(command);
        }
        double worst = 0.0;
        for (int joint = 0; joint < 7; ++joint) {
            worst = std::max(worst, std::abs(trajectory.points.front()[joint] - measured[joint]));
        }
        if (worst > kStartRejectError) {
            RCLCPP_ERROR(get_logger(),
                         "轨迹起点离当前关节 %.2f rad（>%.2f），拒绝。请用当前状态重新规划",
                         worst, kStartRejectError);
            arm->hold_here();
            goal_handle->abort(std::make_shared<Follow::Result>());
            return;
        }
        if (worst > kStartWarpError) {
            // After a disturbance, MoveIt often plans from a stale "at goal" start.
            // Warp the first knot to the real measured pose so execution can start.
            RCLCPP_WARN(get_logger(),
                        "轨迹起点偏差 %.2f rad，已把首点改成当前关节角再执行", worst);
            trajectory.points.front() = measured;
        }
        const double duration = arm->set_trajectory(std::move(trajectory));
        // Extra time: stalls while the arm is held back by external force.
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(std::max(2.0, duration * 3.0 + 2.0));
        bool finished = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (goal_handle->is_canceling()) {
                arm->clear_trajectory();
                arm->hold_here();
                goal_handle->canceled(std::make_shared<Follow::Result>());
                return;
            }
            if (arm->trajectory_done()) {
                finished = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        arm->clear_trajectory();
        if (logger_) {
            logger_->flush();
        }
        if (goal_handle->is_canceling()) {
            arm->hold_here();
            goal_handle->canceled(std::make_shared<Follow::Result>());
            return;
        }
        const double final_err = arm->max_tracking_error();
        if (!finished || final_err > kSettleError) {
            // Sync q_cmd to measured so the next plan's start state matches reality.
            RCLCPP_ERROR(get_logger(),
                         "跟踪未到位（误差 %.2f rad%s），已保持当前姿态。请松手后重新规划",
                         final_err, finished ? "" : ", 执行超时");
            arm->hold_here();
            goal_handle->abort(std::make_shared<Follow::Result>());
            return;
        }
        goal_handle->succeed(std::make_shared<Follow::Result>());
    }

    ArmBus* arm_for(const Follow::Goal& goal) {
        bool left = false;
        bool right = false;
        for (const auto& joint : goal.trajectory.joint_names) {
            left = left || joint.find("left") != std::string::npos;
            right = right || joint.find("right") != std::string::npos;
        }
        if (left == right) {
            RCLCPP_ERROR(get_logger(), "一条轨迹只能包含一只手臂");
            return nullptr;
        }
        return arms_[left ? "left" : "right"].get();
    }

    void maybe_reload_gravity() {
        bool force = g_reload_gravity.exchange(false);
        if (!force && !gravity_yaml_.empty()) {
            try {
                const auto mtime = std::filesystem::last_write_time(gravity_yaml_);
                if (mtime != yaml_mtime_) {
                    yaml_mtime_ = mtime;
                    force = true;
                }
            } catch (...) {
            }
        }
        if (!force || gravity_yaml_.empty()) {
            return;
        }
        GravityTune left_tune;
        GravityTune right_tune;
        GravityModel model;
        if (!load_gravity_yaml_file(gravity_yaml_, &left_tune, &right_tune, &model)) {
            RCLCPP_WARN(get_logger(), "热加载失败：读不了 %s", gravity_yaml_.c_str());
            return;
        }
        arms_["left"]->set_tune(left_tune);
        arms_["right"]->set_tune(right_tune);
        arms_["left"]->set_model(model);
        arms_["right"]->set_model(model);
        arms_["left"]->set_tracking_ff_gain(model.tracking_ff_gain);
        arms_["right"]->set_tracking_ff_gain(model.tracking_ff_gain);
        RCLCPP_INFO(get_logger(),
                    "已热加载重力：urdf_scale=%.3f ee_payload=%.4f tracking_g_scale=%.3f",
                    model.urdf_scale, model.ee_payload_kg, model.tracking_ff_gain);
    }

    void loop() {
        auto next = std::chrono::steady_clock::now();
        int reload_div = 0;
        while (running_ && rclcpp::ok()) {
            if ((++reload_div % 50) == 0) {  // ~2 Hz
                maybe_reload_gravity();
            }
            const auto stamp = now();
            const double t_sec =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
            sensor_msgs::msg::JointState message;
            message.header.stamp = stamp;
            for (auto& [_, arm] : arms_) {
                arm->track(stamp);
                arm->spin_once(t_sec);
                arm->fill(&message);
            }
            publisher_->publish(message);
            next += std::chrono::milliseconds(10);
            std::this_thread::sleep_until(next);
        }
    }

    std::map<std::string, std::unique_ptr<ArmBus>> arms_;
    std::map<std::string, rclcpp::CallbackGroup::SharedPtr> action_groups_;
    std::map<std::string, rclcpp_action::Server<Follow>::SharedPtr> servers_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::shared_ptr<ErrorLogger> logger_;
    std::thread thread_;
    std::chrono::steady_clock::time_point started_;
    std::atomic<bool> running_{false};
    std::string gravity_yaml_;
    std::filesystem::file_time_type yaml_mtime_{};
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RealArm>();
        // At least two threads: left + right action execute callbacks can block together.
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        fprintf(stderr, "[openarm_real_arm] FATAL: %s\n", error.what());
        RCLCPP_FATAL(rclcpp::get_logger("openarm_real_arm"), "真机连接失败：%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
