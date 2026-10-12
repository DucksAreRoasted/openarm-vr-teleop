// Automatic gravity calibration: slow mirrored joint waypoints + dwells.
// Logs the same CSV as zero-g for tune_gravity_comp.py --residual.

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <openarm_port/can_feedback.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::array<double, 7> KP_TRACK{22.0, 22.0, 16.0, 16.0, 8.0, 8.0, 8.0};
constexpr std::array<double, 7> KD_TRACK{1.0, 1.0, 0.7, 0.7, 0.3, 0.25, 0.3};
// Soft hold during dwell (reBot-style low kp): sag reveals under-compensation.
constexpr std::array<double, 7> KP_SOFT{2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0};
constexpr std::array<double, 7> KD_SOFT{0.35, 0.35, 0.3, 0.3, 0.15, 0.12, 0.15};
constexpr double MAX_JOINT_SPEED = 0.10;  // rad/s per joint while ramping
constexpr double DWELL_S = 5.0;  // longer soft-hold so lateral sag shows in residual
constexpr double SETTLE_S = 2.0;
constexpr double HARD_SNAP_S = 0.7;
constexpr double GAIN_BLEND_S = 0.8;
constexpr double QUIET_SPEED = 0.05;    // rad/s from joint position, sustained before logging
constexpr double QUIET_HOLD_S = 0.40;
constexpr double QUIET_TIMEOUT_S = 3.0;
constexpr double kTrackErrAbort = 0.75;  // rad, hard tracking
constexpr double kSoftErrAbort = 1.35;   // rad, soft sag
constexpr double kSpeedAbort = 1.5;      // rad/s from joint position
constexpr double kFaultHoldS = 0.30;     // single-sample velocity glitches must not abort

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

constexpr double kFingerMassKg = 2.0 * 0.03602545343277134;
constexpr double kGripperMotorKgDefault = 0.12;
constexpr double kFingerComX = 0.0;
constexpr double kFingerComY = 0.0;
constexpr double kFingerComZ = 0.1025 + 0.045;

void apply_gravity_model(Dynamics* dynamics, const GravityModel& model, const std::string& side) {
    if (dynamics == nullptr) {
        return;
    }
    dynamics->SetLinkMassScales(link_mass_scales_for(model, side));
    dynamics->SetLinkComDeltas(link_com_deltas_for(model, side));
    dynamics->SetUrdfGravityScale(model.urdf_scale);
    const double motor = model.gripper_motor_kg > 0.0 ? model.gripper_motor_kg : kGripperMotorKgDefault;
    dynamics->SetEndEffectorPointMass(kFingerMassKg + motor + model.ee_payload_kg, kFingerComX,
                                      kFingerComY, kFingerComZ);
    const auto& off =
        (side == "left") ? model.wrist_camera_offset_left : model.wrist_camera_offset_right;
    dynamics->SetWristCameraMass(model.wrist_camera_kg, side == "left", off[0], off[1], off[2]);
    dynamics->SetDistalMassGains(model.distal_ee_gain, model.distal_cam_gain);
}

void ensure_friction(GravityTune* tune) {
    // Only seed when yaml has no friction at all. Do not override intentional low values
    // from the joint-tune UI (including 0 on proximal joints).
    double sum = 0.0;
    for (double v : tune->fc) {
        sum += std::abs(v);
    }
    if (sum < 1e-6) {
        seed_default_friction(tune);
        // Light DM4340 defaults when seeding from empty yaml.
        tune->fc[2] = 0.22;
        tune->fv[2] = 0.016;
        tune->fc[3] = 0.32;
        tune->fv[3] = 0.022;
        tune->fk[2] = 20.0;
        tune->fk[3] = 20.0;
    }
    // Wrist DM4310 (j5–j7): gravity torque is often ≪ static friction (~0.2 Nm vs
    // stick). Without FF, zero-g can never feel free on forearm roll / wrist.
    // Only fill zeros so joint-tune / yaml overrides stay respected.
    constexpr double kWristFc = 0.06;
    constexpr double kWristFv = 0.004;
    for (size_t i : {4u, 5u, 6u}) {
        if (std::abs(tune->fc[i]) < 1e-9) {
            tune->fc[i] = kWristFc;
        }
        if (std::abs(tune->fv[i]) < 1e-9) {
            tune->fv[i] = kWristFv;
        }
        if (tune->fk[i] < 1.0) {
            tune->fk[i] = 25.0;
        }
    }
}

class CsvLogger {
public:
    explicit CsvLogger(const std::string& directory) {
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
        file_ << "t,side,traj,ff_gain";
        for (int joint = 1; joint <= 7; ++joint) {
            file_ << ",q_cmd" << joint << ",q_meas" << joint << ",e" << joint << ",dq" << joint
                  << ",dq_meas" << joint << ",g_model" << joint << ",tau_g" << joint
                  << ",tau_ff" << joint << ",mtau" << joint;
        }
        file_ << "\n";
    }

    const std::string& path() const { return path_; }

    void write(double t, const std::string& side, int traj, const std::array<double, 7>& q_cmd,
               const std::array<double, 7>& q_meas, const std::array<double, 7>& dq_cmd,
               const std::array<double, 7>& dq_meas, const std::array<double, 7>& g_model,
               const std::array<double, 7>& tau_g, const std::array<double, 7>& tau_ff,
               const std::array<double, 7>& mtau, double ff_gain) {
        if (!file_) {
            return;
        }
        file_ << t << "," << side << "," << traj << "," << ff_gain;
        for (int joint = 0; joint < 7; ++joint) {
            file_ << "," << q_cmd[joint] << "," << q_meas[joint] << ","
                  << (q_cmd[joint] - q_meas[joint]) << "," << dq_cmd[joint] << "," << dq_meas[joint]
                  << "," << g_model[joint] << "," << tau_g[joint] << "," << tau_ff[joint] << ","
                  << mtau[joint];
        }
        file_ << "\n";
    }

    void flush() {
        if (file_) {
            file_.flush();
        }
    }

private:
    std::string path_;
    std::ofstream file_;
};

using Pose = std::array<double, 7>;

// Distal→proximal factory excitation. traj_id bands for sequential ID:
//   100–199 wrist (j5–j7 + EE), proximal held
//   200–299 elbow (j3–j4), wrist mostly fixed
//   300–399 shoulder (j1–j2) lateral/sagittal
// Right = mirror_right(left). Limits: left j1∈[-3.49,1.40], j2∈[-3.32,0.17].
struct Waypoint {
    const char* name;
    int traj_id;
    Pose q;
};

const std::vector<Waypoint> kLeftWaypoints = {
    // —— Stage wrist: mid-reach base, sweep wrist ——
    {"w_base", 100, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, 0.0}},
    {"w_roll_p", 110, Pose{0.0, -1.10, 0.0, 1.45, 0.85, 0.0, 0.0}},
    {"w_roll_n", 111, Pose{0.0, -1.10, 0.0, 1.45, -0.85, 0.0, 0.0}},
    {"w_pitch_p", 120, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.55, 0.0}},
    {"w_pitch_n", 121, Pose{0.0, -1.10, 0.0, 1.45, 0.0, -0.45, 0.0}},
    {"w_yaw_p", 130, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, 0.70}},
    {"w_yaw_n", 131, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, -0.70}},
    {"w_combo", 140, Pose{0.0, -1.10, 0.0, 1.45, 0.60, -0.35, 0.50}},
    {"w_combo2", 141, Pose{0.0, -1.10, 0.0, 1.45, -0.55, 0.30, -0.45}},
    // —— Stage elbow: vary j3/j4, wrist near zero ——
    {"e_base", 200, Pose{0.0, -1.15, 0.0, 1.20, 0.0, 0.0, 0.0}},
    {"e_flex", 210, Pose{0.0, -1.15, 0.0, 1.75, 0.0, 0.15, 0.0}},
    {"e_ext", 211, Pose{0.0, -1.15, 0.0, 0.95, 0.0, 0.0, 0.0}},
    {"e_roll_p", 220, Pose{0.0, -1.15, 0.55, 1.45, 0.0, 0.0, 0.0}},
    {"e_roll_n", 221, Pose{0.0, -1.15, -0.55, 1.45, 0.0, 0.0, 0.0}},
    {"e_high", 230, Pose{0.0, -1.40, 0.20, 1.80, 0.0, 0.20, 0.0}},
    // —— Stage shoulder: lateral + sagittal ——
    {"s_home", 300, Pose{0.0, -0.70, 0.0, 1.20, 0.0, 0.0, 0.0}},
    {"s_fwd", 310, Pose{0.0, -1.25, 0.0, 1.65, 0.0, 0.15, 0.0}},
    {"s_high", 311, Pose{0.0, -1.55, 0.0, 1.85, 0.0, 0.25, 0.0}},
    {"s_side45", 320, Pose{-0.85, -1.15, 0.0, 1.50, 0.0, 0.0, 0.0}},
    {"s_side90", 321, Pose{-1.35, -1.05, 0.0, 1.30, 0.0, 0.0, 0.0}},
    {"s_side90e", 322, Pose{-1.45, -1.10, 0.0, 0.95, 0.0, 0.0, 0.0}},
    {"s_side_hi", 323, Pose{-1.25, -1.35, 0.20, 1.50, 0.0, 0.0, 0.0}},
    {"s_home2", 330, Pose{0.0, -0.70, 0.0, 1.20, 0.0, 0.0, 0.0}},
};

// Visual left↔right mirror in joint space for OpenArm V1:
// - j1/j2: negate (shoulder); j3/j4: same (elbow)
// - j5/j6/j7: negate (wrist); URDF j7 axis flip alone is not enough for visual mirror
Pose mirror_right(const Pose& left) {
    Pose right = left;
    right[0] = -left[0];
    right[1] = -left[1];
    right[4] = -left[4];
    right[5] = -left[5];
    right[6] = -left[6];
    return right;
}

double ramp_duration(const Pose& a, const Pose& b) {
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, std::abs(b[i] - a[i]));
    }
    return std::max(1.0, 1.5 * worst / MAX_JOINT_SPEED);
}

Pose lerp(const Pose& a, const Pose& b, double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    Pose out{};
    for (size_t i = 0; i < a.size(); ++i) {
        out[i] = a[i] + alpha * (b[i] - a[i]);
    }
    return out;
}

class ArmCalib {
public:
    ArmCalib(const std::string& can, const std::string& side, const std::string& urdf,
             GravityTune tune, GravityModel model, std::shared_ptr<CsvLogger> logger)
        : side_(side), tune_(tune), ff_gain_(model.ff_gain), logger_(std::move(logger)) {
        hardware_ = std::make_unique<openarm::can::socket::OpenArm>(can, true);
        const std::vector<openarm::damiao_motor::MotorType> types{
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310};
        hardware_->init_arm_motors(types, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
                                   {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
        hardware_->init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);
        hardware_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        hardware_->enable_all();
        enabled_ = true;
        for (int cycle = 0; cycle < 20; ++cycle) {
            hardware_->refresh_all();
            hardware_->recv_all(2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const CanFeedbackVerdict verdict = refresh_until_fresh(hardware_.get());
        if (!verdict.ok) {
            const std::string detail = format_can_feedback_verdict(verdict, "[calib] " + side_);
            std::cerr << detail << std::endl;
            std::cerr << "[calib] " << side_
                      << " 没有新鲜有效反馈，拒绝进入运动出力（不按 q=0 计算重力）" << std::endl;
            try {
                hardware_->disable_all();
            } catch (...) {
            }
            enabled_ = false;
            throw std::runtime_error(detail);
        }
        dynamics_ = std::make_unique<Dynamics>(urdf, "openarm_body_link0",
                                               "openarm_" + side_ + "_link7");
        dynamics_->SetLinkMassScales(link_mass_scales_for(model, side_));
        dynamics_->SetLinkComDeltas(link_com_deltas_for(model, side_));
        if (!dynamics_->Init()) {
            dynamics_.reset();
            throw std::runtime_error(side_ + " 重力模型初始化失败");
        }
        apply_gravity_model(dynamics_.get(), model, side_);
        read();
        gripper_hold_ = gripper_;
        command_ = measured_;
        std::cout << "[calib] " << side_ << " 就绪 can=" << can << std::endl;
    }

    ~ArmCalib() {
        if (hardware_ && enabled_) {
            try {
                for (int cycle = 0; cycle < 30; ++cycle) {
                    std::vector<openarm::damiao_motor::MITParam> arm;
                    arm.reserve(7);
                    for (size_t index = 0; index < measured_.size(); ++index) {
                        arm.push_back({0.0, KD_TRACK[index], measured_[index], 0.0, 0.0});
                    }
                    hardware_->get_arm().mit_control_all(arm);
                    hardware_->recv_all(1000);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                hardware_->disable_all();
                hardware_->recv_all(1000);
            } catch (...) {
            }
        }
    }

    Pose measured() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return measured_;
    }

    void set_command(const Pose& q, const Pose& dq = Pose{}) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = q;
        command_vel_ = dq;
    }

    void set_soft_hold(bool soft) { set_compliance(soft ? 1.0 : 0.0); }

    void set_compliance(double compliance) {
        std::lock_guard<std::mutex> lock(mutex_);
        compliance_ = std::clamp(compliance, 0.0, 1.0);
        soft_hold_ = compliance_ >= 0.999;
    }

    double compliance() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return compliance_;
    }

    Pose command() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return command_;
    }

    void set_logging(bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        logging_ = enabled;
    }

    bool quiet(double speed_max) const {
        std::lock_guard<std::mutex> lock(mutex_);
        // Position difference, not the velocity register. That register spikes
        // while the joint is still, which would block every dwell.
        return have_pos_speed_ && pos_speed_ <= speed_max;
    }

    bool step(double t_sec, int traj_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fault_ || !enabled_) {
            return false;
        }
        std::string detail;
        if (!can_feedback_allows_torque(hardware_.get(), "[calib] " + side_, &detail)) {
            std::cerr << detail << std::endl;
            std::cerr << "[calib] " << side_ << " 反馈过期，停止下发并 disable_all" << std::endl;
            try {
                hardware_->disable_all();
            } catch (...) {
            }
            enabled_ = false;
            fault_ = true;
            return false;
        }
        std::array<double, 7> gravity{};
        std::array<double, 7> tau_g{};
        std::array<double, 7> tau_ff{};
        std::array<double, 7> kp{};
        std::array<double, 7> kd{};
        dynamics_->GetGravity(measured_.data(), gravity.data());
        for (size_t index = 0; index < kp.size(); ++index) {
            kp[index] = KP_TRACK[index] + compliance_ * (KP_SOFT[index] - KP_TRACK[index]);
            kd[index] = KD_TRACK[index] + compliance_ * (KD_SOFT[index] - KD_TRACK[index]);
        }
        std::vector<openarm::damiao_motor::MITParam> arm;
        arm.reserve(7);
        for (size_t index = 0; index < measured_.size(); ++index) {
            tau_g[index] = ff_gain_ * (tune_.scale[index] * gravity[index] + tune_.bias[index]);
            tau_ff[index] =
                compose_tau_ff(gravity[index], tune_, index, measured_vel_[index], ff_gain_);
            arm.push_back(
                {kp[index], kd[index], command_[index], command_vel_[index], tau_ff[index]});
        }
        hardware_->get_arm().mit_control_all(arm);
        hardware_->get_gripper().mit_control_all({{0.0, 0.2, gripper_hold_, 0.0, 0.0}});
        hardware_->recv_all(1000);
        read_locked();
        const double dt = have_sample_ ? (t_sec - prev_t_) : 0.0;
        pos_speed_ = 0.0;
        if (dt > 1e-4) {
            for (size_t index = 0; index < measured_.size(); ++index) {
                pos_speed_ = std::max(
                    pos_speed_, std::abs(measured_[index] - prev_q_[index]) / dt);
            }
            have_pos_speed_ = true;
        }
        prev_q_ = measured_;
        prev_t_ = t_sec;
        have_sample_ = true;
        double max_err = 0.0;
        for (size_t index = 0; index < measured_.size(); ++index) {
            max_err = std::max(max_err, std::abs(command_[index] - measured_[index]));
        }
        const double err_limit = compliance_ > 0.8 ? kSoftErrAbort : kTrackErrAbort;
        const double step_dt = std::clamp(dt, 0.0, 0.05);
        if (max_err > err_limit) {
            over_err_s_ += step_dt;
        } else {
            over_err_s_ = 0.0;
        }
        if (have_pos_speed_ && pos_speed_ > kSpeedAbort) {
            over_speed_s_ += step_dt;
        } else {
            over_speed_s_ = 0.0;
        }
        if (over_err_s_ >= kFaultHoldS || over_speed_s_ >= kFaultHoldS) {
            fault_ = true;
            std::cerr << "[calib] " << side_ << " 中止：跟踪误差 " << std::fixed
                      << std::setprecision(3) << max_err << " rad 持续 " << over_err_s_
                      << " s，位移速度 " << pos_speed_ << " rad/s 持续 " << over_speed_s_
                      << " s" << std::endl;
            return false;
        }
        // Log only fully settled soft hold. τ_g is scale·g+bias, without friction.
        if (logging_ && compliance_ >= 0.999 && logger_ && (++log_div_ % 2) == 0) {
            // Match g_model to the refreshed q_meas in this CSV row. tau_g/tau_ff
            // still record the command actually sent before receiving feedback.
            dynamics_->GetGravity(measured_.data(), gravity.data());
            logger_->write(t_sec, side_, traj_id, command_, measured_, command_vel_, measured_vel_,
                           gravity, tau_g, tau_ff, measured_tau_, ff_gain_);
        }
        return true;
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
        const auto gripper = hardware_->get_gripper().get_motors();
        if (!gripper.empty()) {
            gripper_ = gripper.front().get_position();
        }
    }

    std::string side_;
    GravityTune tune_;
    double ff_gain_ = 1.0;
    std::shared_ptr<CsvLogger> logger_;
    std::unique_ptr<openarm::can::socket::OpenArm> hardware_;
    std::unique_ptr<Dynamics> dynamics_;
    mutable std::mutex mutex_;
    Pose measured_{};
    Pose measured_vel_{};
    Pose measured_tau_{};
    Pose command_{};
    Pose command_vel_{};
    double gripper_ = 0.0;
    double gripper_hold_ = 0.0;
    bool enabled_ = false;
    bool soft_hold_ = false;
    bool logging_ = false;
    bool fault_ = false;
    double compliance_ = 0.0;
    int log_div_ = 0;
    bool have_sample_ = false;
    bool have_pos_speed_ = false;
    double prev_t_ = 0.0;
    double pos_speed_ = 0.0;
    double over_err_s_ = 0.0;
    double over_speed_s_ = 0.0;
    Pose prev_q_{};
};

std::string env_or(const char* key, const std::string& fallback) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : fallback;
}

}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string root = env_or("OPENARM_ROOT", "");
    if (root.empty()) {
        std::cerr << "[calib] 需要 OPENARM_ROOT" << std::endl;
        return 1;
    }
    const std::string urdf = env_or("OPENARM_URDF", root + "/assets/v1.urdf");
    const std::string yaml =
        env_or("OPENARM_GRAVITY_YAML", root + "/config/gravity_comp.yaml");
    const std::string left_can = env_or("OPENARM_LEFT_FOLLOWER_CAN", "can1");
    const std::string right_can = env_or("OPENARM_RIGHT_FOLLOWER_CAN", "can0");
    const std::string log_dir =
        env_or("OPENARM_ZERO_G_LOG_DIR", root + "/recordings/gravity");

    std::string trajectory = env_or("OPENARM_CALIB_TRAJECTORY", "waypoints");
    if (trajectory != "waypoints" && trajectory != "fourier") {
        std::cerr << "[calib] 无效轨迹方法：" << trajectory << std::endl;
        return 2;
    }
    if (trajectory == "fourier") {
        std::cout << "[calib] 均匀傅里叶相位的列归一化条件数差于航点"
                     "（腕 14.1>8.7，肘 10.9>8.8，肩 14.8>7.0），改为航点软持"
                  << std::endl;
        trajectory = "waypoints";
    }
    const auto waypoints = kLeftWaypoints;
    std::cout << "[calib] trajectory=" << trajectory << std::endl;

    GravityTune left_tune;
    GravityTune right_tune;
    GravityModel model;
    if (!load_gravity_yaml(yaml, &left_tune, &right_tune, &model)) {
        std::cerr << "[calib] 读不到 " << yaml << "，用解析默认" << std::endl;
        seed_default_friction(&left_tune);
        seed_default_friction(&right_tune);
    } else {
        ensure_friction(&left_tune);
        ensure_friction(&right_tune);
        std::cout << "[calib] 已加载 " << yaml << " urdf_scale=" << model.urdf_scale << std::endl;
    }

    auto logger = std::make_shared<CsvLogger>(log_dir);
    if (logger->path().empty()) {
        std::cerr << "[calib] 打不开日志目录 " << log_dir << std::endl;
        return 1;
    }
    std::cout << "LOG_PATH=" << logger->path() << std::endl;
    std::cout << "[calib] 左右严格镜像同步；移动硬跟踪，停留低 kp 软持采下垂残差。" << std::endl;

    std::unique_ptr<ArmCalib> left;
    std::unique_ptr<ArmCalib> right;
    try {
        left = std::make_unique<ArmCalib>(left_can, "left", urdf, left_tune, model, logger);
        right = std::make_unique<ArmCalib>(right_can, "right", urdf, right_tune, model, logger);
    } catch (const std::exception& error) {
        std::cerr << "[calib] 启动失败：" << error.what() << std::endl;
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    auto sleep_step = [&](double t, int traj) {
        if (!left->step(t, traj) || !right->step(t, traj)) {
            g_running = false;
        }
        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    };

    auto smooth_dq = [](const Pose& from, const Pose& goal, double alpha, double duration) {
        const double ds_dt =
            duration > 1e-6 ? (6.0 * alpha * (1.0 - alpha) / duration) : 0.0;
        Pose dq{};
        for (size_t i = 0; i < from.size(); ++i) {
            dq[i] = (goal[i] - from[i]) * ds_dt;
        }
        return dq;
    };

    auto blend_compliance = [&](double target, double seconds, int traj) {
        const double c0 = left->compliance();
        if (seconds <= 0.0 || std::abs(target - c0) < 1e-3) {
            left->set_compliance(target);
            right->set_compliance(target);
            return;
        }
        const Pose hold_l = left->command();
        const Pose hold_r = right->command();
        left->set_command(hold_l);
        right->set_command(hold_r);
        const auto blend_start = std::chrono::steady_clock::now();
        while (g_running) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - blend_start)
                    .count();
            if (elapsed >= seconds) {
                break;
            }
            const double alpha = elapsed / seconds;
            const double s = alpha * alpha * (3.0 - 2.0 * alpha);
            const double compliance = c0 + (target - c0) * s;
            left->set_compliance(compliance);
            right->set_compliance(compliance);
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj);
        }
        left->set_compliance(target);
        right->set_compliance(target);
    };

    auto hard_hold = [&](const Pose& ql, const Pose& qr, double seconds, int traj) {
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        left->set_logging(false);
        right->set_logging(false);
        left->set_command(ql);
        right->set_command(qr);
        const auto until =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (g_running && std::chrono::steady_clock::now() < until) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj);
        }
    };

    auto soft_dwell = [&](int traj_id, double seconds, const char* label) {
        blend_compliance(1.0, GAIN_BLEND_S, traj_id);
        std::cout << "[calib] 等待停稳 " << label << std::endl;
        double quiet_for = 0.0;
        auto mark = std::chrono::steady_clock::now();
        const auto settle_start = mark;
        bool settled = false;
        while (g_running) {
            const auto now = std::chrono::steady_clock::now();
            const double waited = std::chrono::duration<double>(now - settle_start).count();
            if (waited >= QUIET_TIMEOUT_S) {
                break;
            }
            const double dt = std::chrono::duration<double>(now - mark).count();
            mark = now;
            const double t = std::chrono::duration<double>(now - t0).count();
            sleep_step(t, traj_id);
            if (left->quiet(QUIET_SPEED) && right->quiet(QUIET_SPEED)) {
                quiet_for += dt;
            } else {
                quiet_for = 0.0;
            }
            if (quiet_for >= QUIET_HOLD_S) {
                settled = true;
                break;
            }
        }
        if (!g_running) {
            return;
        }
        if (!settled) {
            std::cout << "[calib] 未停稳，跳过 " << label << std::endl;
            return;
        }
        left->set_logging(true);
        right->set_logging(true);
        std::cout << "[calib] 软持采数 " << label << " " << seconds << " s（左右同时）" << std::endl;
        const auto until =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (g_running && std::chrono::steady_clock::now() < until) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj_id);
        }
        left->set_logging(false);
        right->set_logging(false);
    };

    // Sync both arms to the first excitation pose (mirrored).
    const Pose home_l = waypoints.front().q;
    const Pose home_r = mirror_right(home_l);
    const int home_traj = waypoints.front().traj_id;
    {
        const Pose from_l = left->measured();
        const Pose from_r = right->measured();
        const double duration =
            std::max(ramp_duration(from_l, home_l), ramp_duration(from_r, home_r));
        std::cout << "[calib] 分阶段激励 j7→j1：wrist(1xx) → elbow(2xx) → shoulder(3xx)"
                  << std::endl;
        std::cout << "[calib] 同步到起始姿态"
                  << "，ramp=" << std::fixed << std::setprecision(1) << duration << "s"
                  << std::endl;
        const auto ramp_start = std::chrono::steady_clock::now();
        while (g_running) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_start)
                    .count();
            if (elapsed >= duration) {
                break;
            }
            const double alpha = elapsed / duration;
            const double s = alpha * alpha * (3.0 - 2.0 * alpha);
            left->set_command(lerp(from_l, home_l, s), smooth_dq(from_l, home_l, alpha, duration));
            right->set_command(lerp(from_r, home_r, s), smooth_dq(from_r, home_r, alpha, duration));
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, home_traj);
        }
        hard_hold(home_l, home_r, HARD_SNAP_S, home_traj);
        soft_dwell(home_traj, SETTLE_S, waypoints.front().name);
        hard_hold(home_l, home_r, HARD_SNAP_S, home_traj);
    }

    Pose from_l = home_l;
    Pose from_r = home_r;
    for (size_t index = 1; index < waypoints.size() && g_running; ++index) {
        const auto& wp = waypoints[index];
        const Pose& left_goal = wp.q;
        const Pose right_goal = mirror_right(left_goal);
        const int traj_id = wp.traj_id;
        const double duration = std::max(ramp_duration(from_l, left_goal),
                                         ramp_duration(from_r, right_goal));
        std::cout << "[calib] " << index << "/" << (waypoints.size() - 1) << " → " << wp.name
                  << " traj=" << traj_id
                  << "  L=[" << left_goal[0] << "," << left_goal[1] << ",…," << left_goal[4]
                  << "," << left_goal[5] << "," << left_goal[6] << "]"
                  << "  R=[" << right_goal[0] << "," << right_goal[1] << ",…," << right_goal[4]
                  << "," << right_goal[5] << "," << right_goal[6] << "]"
                  << "  ramp=" << std::fixed << std::setprecision(1) << duration << "s"
                  << " dwell=" << DWELL_S << "s" << std::endl;
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        left->set_logging(false);
        right->set_logging(false);
        const auto ramp_start = std::chrono::steady_clock::now();
        while (g_running) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_start)
                    .count();
            if (elapsed >= duration) {
                break;
            }
            const double alpha = elapsed / duration;
            const double s = alpha * alpha * (3.0 - 2.0 * alpha);
            left->set_command(lerp(from_l, left_goal, s),
                              smooth_dq(from_l, left_goal, alpha, duration));
            right->set_command(lerp(from_r, right_goal, s),
                               smooth_dq(from_r, right_goal, alpha, duration));
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj_id);
        }
        hard_hold(left_goal, right_goal, HARD_SNAP_S, traj_id);
        soft_dwell(traj_id, DWELL_S, wp.name);
        blend_compliance(0.0, GAIN_BLEND_S, traj_id);
        hard_hold(left_goal, right_goal, HARD_SNAP_S, traj_id);
        from_l = left_goal;
        from_r = right_goal;
    }

    logger->flush();
    if (!g_running) {
        std::cout << "[calib] 已中止。日志：" << logger->path() << std::endl;
        return 130;
    }
    std::cout << "[calib] 完成。LOG_PATH=" << logger->path() << std::endl;
    return 0;
}
