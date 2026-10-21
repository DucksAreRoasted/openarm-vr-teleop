// Automatic gravity calibration: slow mirrored joint waypoints + dwells.
// Logs the same CSV as zero-g for tune_gravity_comp.py --residual.

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <controller/gravity_calibration_plan.hpp>
#include <openarm_port/can_feedback.hpp>
#include <openarm_port/gravity_hardware.hpp>

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

using namespace gravity_calibration;

constexpr std::array<double, 7> KP_TRACK{22.0, 22.0, 16.0, 16.0, 8.0, 8.0, 8.0};
constexpr std::array<double, 7> KD_TRACK{1.0, 1.0, 0.7, 0.7, 0.3, 0.25, 0.3};
// Soft hold during dwell (reBot-style low kp): sag reveals under-compensation.
constexpr std::array<double, 7> KP_SOFT{2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0};
constexpr std::array<double, 7> KD_SOFT{0.35, 0.35, 0.3, 0.3, 0.15, 0.12, 0.15};
constexpr double DWELL_S = 5.0;  // longer soft-hold so lateral sag shows in residual
constexpr double SETTLE_S = 2.0;
constexpr double HARD_SNAP_S = 0.7;
constexpr double GAIN_BLEND_S = 0.8;
constexpr double QUIET_SPEED = 0.05;    // rad/s over SettleSpeed's window, then held
constexpr double QUIET_HOLD_S = 0.40;
constexpr double QUIET_TIMEOUT_S = 6.0;
constexpr double kTrackErrAbort = 0.75;  // rad, hard tracking
constexpr double kSoftErrAbort = 1.35;   // rad, soft sag
constexpr double kSpeedAbort = 1.5;      // rad/s from joint position
constexpr double kFaultHoldS = 0.30;     // single-sample velocity glitches must not abort
// Friction sweep: hard tracking at ≤ 0.30 rad/s, so lag stays a few hundredths.
constexpr double kSweepErrAbort = 0.15;
constexpr double kSweepSpeedAbort = 0.60;
constexpr std::array<double, 3> kSweepSpeeds{0.05, 0.15, 0.30};
constexpr double kSweepAccel = 0.6;      // rad/s², trapezoid ends
constexpr double kSweepSettleS = 0.3;    // skip lag transient after accel and before decel
// Real gripper travel is −0.98 (open) to 0.62 (closed). Stay inside both stops.
constexpr double kGripperLo = -0.80;
constexpr double kGripperHi = 0.40;
constexpr double kGripperKp = 10.0;
constexpr double kGripperKd = 0.40;
constexpr double kGripperErrAbort = 0.28;
constexpr double kGripperSpeedAbort = 1.2;
constexpr double kGripperAccel = 1.5;
constexpr std::array<double, 3> kGripperSpeeds{0.10, 0.25, 0.50};
struct FrictionSweep {
    int joint;  // 0-based
    Pose base;  // left frame, swept joint overwritten
    double lo;
    double hi;
};
// Poses and ranges stay inside the gravity calibration envelope above.
const std::array<FrictionSweep, 7> kFrictionSweeps{{
    {0, Pose{0.0, -0.15, 0.0, 0.45, 0.0, 0.0, 0.0}, -0.60, 0.10},
    {1, Pose{0.0, -0.40, 0.0, 1.20, 0.0, 0.0, 0.0}, -0.80, -0.15},
    {2, Pose{0.0, -1.15, 0.0, 1.45, 0.0, 0.0, 0.0}, -0.45, 0.45},
    {3, Pose{0.0, -1.15, 0.0, 1.45, 0.0, 0.0, 0.0}, 0.90, 1.90},
    {4, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, 0.0}, -0.80, 0.80},
    {5, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, 0.0}, -0.40, 0.50},
    {6, Pose{0.0, -1.10, 0.0, 1.45, 0.0, 0.0, 0.0}, -0.65, 0.65},
}};
// Two-sided approach: each pose is held twice, released once from goal+δ and once
// from goal−δ under the soft spring. Static friction then opposes the final motion
// in opposite directions, so the mean hold torque cancels it and half the
// difference estimates it. kp_soft·δ must exceed breakaway for the joint to move.
constexpr double APPROACH_RAMP_S = 1.5;  // soft command glide; avoids overshooting past rest
constexpr double APPROACH_DWELL_S = 3.0;

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

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
    dynamics->SetEndEffectorPointMass(end_effector_mass_for(model, side), kFingerComX,
                                      kFingerComY, kFingerComZ);
    const auto& off =
        (side == "left") ? model.wrist_camera_offset_left : model.wrist_camera_offset_right;
    dynamics->SetWristCameraMass(wrist_camera_mass_for(model, side), side == "left", off[0], off[1], off[2]);
    dynamics->SetDistalMassGains(model.distal_ee_gain, model.distal_cam_gain);
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
        file_ << "t,side,traj,approach,ff_gain";
        for (int joint = 1; joint <= 7; ++joint) {
            file_ << ",q_cmd" << joint << ",q_meas" << joint << ",e" << joint << ",dq" << joint
                  << ",dq_meas" << joint << ",g_model" << joint << ",tau_g" << joint
                  << ",tau_ff" << joint << ",mtau" << joint;
        }
        file_ << "\n";
    }

    const std::string& path() const { return path_; }

    void write(double t, const std::string& side, int traj, int approach,
               const std::array<double, 7>& q_cmd, const std::array<double, 7>& q_meas,
               const std::array<double, 7>& dq_cmd, const std::array<double, 7>& dq_meas,
               const std::array<double, 7>& g_model, const std::array<double, 7>& tau_g,
               const std::array<double, 7>& tau_ff, const std::array<double, 7>& mtau,
               double ff_gain) {
        if (!file_) {
            return;
        }
        file_ << t << "," << side << "," << traj << "," << approach << "," << ff_gain;
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

// One row per control step for the swept joint only. tau_cmd is the MIT torque
// actually requested (kp·e + kd·ė + τ_ff), with τ_ff = gravity only.
class FrictionLogger {
public:
    explicit FrictionLogger(const std::string& directory) {
        std::filesystem::create_directories(directory);
        const std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream name;
        name << directory << "/friction_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        path_ = name.str();
        file_.open(path_);
        if (!file_) {
            path_.clear();
            return;
        }
        file_ << "t,side,joint,speed,dir,q_cmd,q,dq_cmd,dq,tau_g,tau_cmd,mtau\n";
    }

    const std::string& path() const { return path_; }

    void write(double t, const std::string& side, int joint, double speed, int dir, double q_cmd,
               double q, double dq_cmd, double dq, double tau_g, double tau_cmd, double mtau) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!file_) {
            return;
        }
        file_ << t << "," << side << "," << joint << "," << speed << "," << dir << "," << q_cmd
              << "," << q << "," << dq_cmd << "," << dq << "," << tau_g << "," << tau_cmd << ","
              << mtau << "\n";
    }

    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_) {
            file_.flush();
        }
    }

private:
    std::string path_;
    std::ofstream file_;
    std::mutex mutex_;
};


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
        : side_(side), tune_(tune), ff_gain_(model.ff_gain), logger_(std::move(logger)),
          gripper_present_(gripper_present_for(model, side)) {
        hardware_ = std::make_unique<openarm::can::socket::OpenArm>(can, true);
        initialize_gravity_motors(*hardware_, model, side_);
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
        std::cout << "[calib] " << side_ << " 就绪 can=" << can
                  << " gripper=" << (gripper_present_ ? "installed" : "removed") << std::endl;
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

    // +1 / −1: released from goal±δ; 0: single-sided hold.
    void set_approach(int approach) {
        std::lock_guard<std::mutex> lock(mutex_);
        approach_ = approach;
    }

    // Friction sweep: gravity-only feedforward, tight abort limits, and per-step
    // rows for `joint` while `logging` is set. joint < 0 leaves sweep mode.
    void set_sweep(std::shared_ptr<FrictionLogger> logger, int joint, double speed, bool logging) {
        std::lock_guard<std::mutex> lock(mutex_);
        friction_logger_ = std::move(logger);
        sweep_joint_ = joint;
        sweep_speed_ = speed;
        sweep_logging_ = logging && joint >= 0;
    }

    bool gripper_installed() const { return gripper_present_; }

    double gripper_position() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return gripper_;
    }

    double gripper_velocity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return gripper_vel_;
    }

    double gripper_tracking_error() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return gripper_cmd_ - gripper_;
    }

    // Gripper sweep uses position tracking and no friction feedforward, so the
    // tracking torque is the friction being measured. q/dq are ignored when
    // this gripper is not installed.
    void set_gripper_sweep(std::shared_ptr<FrictionLogger> logger, double q, double dq,
                           double speed, bool logging) {
        std::lock_guard<std::mutex> lock(mutex_);
        friction_logger_ = std::move(logger);
        gripper_sweep_ = gripper_present_;
        gripper_cmd_ = q;
        gripper_dq_cmd_ = dq;
        sweep_speed_ = speed;
        sweep_logging_ = logging && gripper_present_;
    }

    void stop_gripper_sweep() {
        std::lock_guard<std::mutex> lock(mutex_);
        gripper_sweep_ = false;
        sweep_logging_ = false;
        gripper_hold_ = gripper_;
    }

    bool quiet(double speed_max) const {
        std::lock_guard<std::mutex> lock(mutex_);
        // Windowed position change, not the velocity register and not one 10 ms
        // sample. Both of those spike while the joint is stopped.
        return settle_speed_.speed() <= speed_max;
    }

    double motion_speed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return settle_speed_.speed();
    }

    Pose hold_torque() {
        std::lock_guard<std::mutex> lock(mutex_);
        Pose gravity{}, torque{};
        dynamics_->GetGravity(measured_.data(), gravity.data());
        for (size_t index = 0; index < torque.size(); ++index) {
            torque[index] = ff_gain_ * (tune_.scale[index] * gravity[index] + tune_.bias[index]) +
                            KP_SOFT[index] * (command_[index] - measured_[index]);
        }
        return torque;
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
        const bool sweeping = sweep_joint_ >= 0;
        std::vector<openarm::damiao_motor::MITParam> arm;
        arm.reserve(7);
        for (size_t index = 0; index < measured_.size(); ++index) {
            tau_g[index] = ff_gain_ * (tune_.scale[index] * gravity[index] + tune_.bias[index]);
            tau_ff[index] = sweeping ? std::clamp(tau_g[index], -kTauFfLimits[index], kTauFfLimits[index])
                                     : compose_tau_ff(gravity[index], tune_, index,
                                                      measured_vel_[index], ff_gain_);
            arm.push_back(
                {kp[index], kd[index], command_[index], command_vel_[index], tau_ff[index]});
        }
        double sweep_q = 0.0, sweep_dq = 0.0, sweep_tau = 0.0;
        if (sweep_logging_) {
            const size_t j = static_cast<size_t>(sweep_joint_);
            sweep_q = measured_[j];
            sweep_dq = measured_vel_[j];
            sweep_tau = kp[j] * (command_[j] - measured_[j]) +
                        kd[j] * (command_vel_[j] - measured_vel_[j]) + tau_ff[j];
        }
        hardware_->get_arm().mit_control_all(arm);
        double grip_q = 0.0, grip_dq = 0.0, grip_tau = 0.0;
        const bool log_grip = gripper_sweep_ && sweep_logging_;
        if (gripper_present_) {
            if (gripper_sweep_) {
                grip_q = gripper_;
                grip_dq = gripper_vel_;
                grip_tau = kGripperKp * (gripper_cmd_ - gripper_) +
                           kGripperKd * (gripper_dq_cmd_ - gripper_vel_);
                hardware_->get_gripper().mit_control_all(
                    {{kGripperKp, kGripperKd, gripper_cmd_, gripper_dq_cmd_, 0.0}});
            } else {
                hardware_->get_gripper().mit_control_all({{0.0, 0.2, gripper_hold_, 0.0, 0.0}});
            }
        }
        hardware_->recv_all(1000);
        read_locked();
        if (log_grip && friction_logger_) {
            const int dir = gripper_dq_cmd_ > 0.0 ? 1 : -1;
            friction_logger_->write(t_sec, side_, 8, sweep_speed_, dir, gripper_cmd_, grip_q,
                                    gripper_dq_cmd_, grip_dq, 0.0, grip_tau, gripper_tau_);
        }
        if (sweeping && sweep_logging_ && friction_logger_) {
            const size_t j = static_cast<size_t>(sweep_joint_);
            const int dir = command_vel_[j] > 0 ? 1 : -1;
            friction_logger_->write(t_sec, side_, sweep_joint_ + 1, sweep_speed_, dir, command_[j],
                                    sweep_q, command_vel_[j], sweep_dq, tau_g[j], sweep_tau,
                                    measured_tau_[j]);
        }
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
        settle_speed_.push(t_sec, measured_);
        double max_err = 0.0;
        for (size_t index = 0; index < measured_.size(); ++index) {
            max_err = std::max(max_err, std::abs(command_[index] - measured_[index]));
        }
        // Interpolate with the gain blend: when soft hold ends the arm is still at its
        // sag, and a step back to the hard limit would abort before kp pulls it in.
        const double err_limit = sweeping ? kSweepErrAbort
                                          : kTrackErrAbort + compliance_ * (kSoftErrAbort - kTrackErrAbort);
        const double speed_limit = sweeping ? kSweepSpeedAbort : kSpeedAbort;
        const double step_dt = std::clamp(dt, 0.0, 0.05);
        if (max_err > err_limit) {
            over_err_s_ += step_dt;
        } else {
            over_err_s_ = 0.0;
        }
        if (have_pos_speed_ && pos_speed_ > speed_limit) {
            over_speed_s_ += step_dt;
        } else {
            over_speed_s_ = 0.0;
        }
        if (gripper_sweep_) {
            const double grip_err = std::abs(gripper_cmd_ - gripper_);
            const double grip_dt = std::clamp(dt, 0.0, 0.05);
            if (grip_err > kGripperErrAbort || std::abs(gripper_vel_) > kGripperSpeedAbort) {
                gripper_over_s_ += grip_dt;
            } else {
                gripper_over_s_ = 0.0;
            }
            if (gripper_over_s_ >= kFaultHoldS) {
                fault_ = true;
                std::cerr << "[friction] " << side_ << " 夹爪中止：跟踪误差 " << std::fixed
                          << std::setprecision(3) << grip_err << " rad，速度 " << gripper_vel_
                          << " rad/s" << std::endl;
                return false;
            }
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
            logger_->write(t_sec, side_, traj_id, approach_, command_, measured_, command_vel_,
                           measured_vel_, gravity, tau_g, tau_ff, measured_tau_, ff_gain_);
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
            gripper_vel_ = gripper.front().get_velocity();
            gripper_tau_ = gripper.front().get_torque();
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
    double gripper_vel_ = 0.0;
    double gripper_tau_ = 0.0;
    double gripper_hold_ = 0.0;
    double gripper_cmd_ = 0.0;
    double gripper_dq_cmd_ = 0.0;
    double gripper_over_s_ = 0.0;
    bool gripper_sweep_ = false;
    bool gripper_present_ = true;
    bool enabled_ = false;
    bool soft_hold_ = false;
    bool logging_ = false;
    int approach_ = 0;
    bool fault_ = false;
    double compliance_ = 0.0;
    int log_div_ = 0;
    bool have_sample_ = false;
    bool have_pos_speed_ = false;
    double prev_t_ = 0.0;
    double pos_speed_ = 0.0;
    SettleSpeed settle_speed_;
    double over_err_s_ = 0.0;
    double over_speed_s_ = 0.0;
    Pose prev_q_{};
    std::shared_ptr<FrictionLogger> friction_logger_;
    int sweep_joint_ = -1;
    double sweep_speed_ = 0.0;
    bool sweep_logging_ = false;
};

std::string env_or(const char* key, const std::string& fallback) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : fallback;
}

}  // namespace

int main(int argc, char** argv) {
    // Preview exits before configuration loading, CAN initialization or logging.
    if (argc == 2 && std::strcmp(argv[1], "--preview-plan") == 0) {
        std::cout << "{\"transit_speed\":" << TRANSIT_JOINT_SPEED
                  << ",\"initial_speed\":" << MAX_JOINT_SPEED
                  << ",\"dwell_min\":1.2,\"dwell_max\":" << APPROACH_DWELL_S
                  << ",\"poses\":[";
        for (size_t index = 0; index < kLeftWaypoints.size(); ++index) {
            const auto& waypoint = kLeftWaypoints[index];
            std::cout << (index ? "," : "") << "{\"name\":\"" << waypoint.name
                      << "\",\"traj\":" << waypoint.traj_id << ",\"q\":[";
            for (size_t joint = 0; joint < 7; ++joint) {
                std::cout << (joint ? "," : "") << waypoint.q[joint];
            }
            std::cout << "]}";
        }
        std::cout << "]}\n";
        return 0;
    }
    const bool friction_mode = argc == 2 && std::strcmp(argv[1], "--friction") == 0;
    const bool gripper_mode = argc == 2 && std::strcmp(argv[1], "--friction-gripper") == 0;
    if (argc != 1 && !friction_mode && !gripper_mode) {
        std::cerr << "Usage: openarm_gravity_calib [--preview-plan|--friction|--friction-gripper]"
                  << std::endl;
        return 2;
    }
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
    if (friction_mode || gripper_mode) {
        trajectory = "waypoints";
    } else if (trajectory != "waypoints" && trajectory != "fourier") {
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
    const std::string approach_mode = env_or("OPENARM_CALIB_APPROACH", "both");
    if (!friction_mode && !gripper_mode && approach_mode != "both" && approach_mode != "single") {
        std::cerr << "[calib] 无效接近方式：" << approach_mode << "（both|single）" << std::endl;
        return 2;
    }
    const bool two_sided = approach_mode == "both";
    if (!friction_mode && !gripper_mode) {
        std::cout << "[calib] trajectory=" << trajectory << std::endl;
        std::cout << "[calib] " << waypoints.size()
                  << " 个姿态，含 4 个肩部垂直附近姿态；双向采数按稳定程度提前完成"
                  << std::endl;
        std::cout << "[calib] approach=" << approach_mode
                  << (two_sided ? "（每个姿态从 +δ、−δ 各软释放一次，抵消静摩擦）" : "")
                  << std::endl;
    }

    GravityTune left_tune;
    GravityTune right_tune;
    GravityModel model;
    if (!load_gravity_yaml(yaml, &left_tune, &right_tune, &model)) {
        std::cerr << "[calib] 读不到 " << yaml << "，用解析默认" << std::endl;
        seed_default_friction(&left_tune);
        seed_default_friction(&right_tune);
    } else {
        // Loaded friction values, including explicit zeros, are authoritative.
        std::cout << "[calib] 已加载 " << yaml << " urdf_scale=" << model.urdf_scale << std::endl;
    }

    std::shared_ptr<CsvLogger> logger;
    std::shared_ptr<FrictionLogger> friction_logger;
    if (friction_mode || gripper_mode) {
        friction_logger = std::make_shared<FrictionLogger>(log_dir);
        if (friction_logger->path().empty()) {
            std::cerr << "[friction] 打不开日志目录 " << log_dir << std::endl;
            return 1;
        }
        std::cout << "FRICTION_LOG_PATH=" << friction_logger->path() << std::endl;
        if (gripper_mode) {
            std::cout << "[friction] 夹爪先探实际行程，再以 0.10/0.25/0.50 rad/s 正反向扫描。"
                      << "夹爪里不要放物体，臂保持不动。" << std::endl;
        } else {
            std::cout << "[friction] 逐关节恒速正反向扫描，速度 0.05/0.15/0.30 rad/s；"
                      << "硬跟踪、只加重力前馈，误差超过 " << kSweepErrAbort << " rad 即停"
                      << std::endl;
        }
    } else {
        logger = std::make_shared<CsvLogger>(log_dir);
        if (logger->path().empty()) {
            std::cerr << "[calib] 打不开日志目录 " << log_dir << std::endl;
            return 1;
        }
        std::cout << "LOG_PATH=" << logger->path() << std::endl;
        std::cout << "[calib] 左右严格镜像同步；移动硬跟踪，停留低 kp 软持采下垂残差。" << std::endl;
    }

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

    auto hard_hold = [&](const Pose& ql, const Pose& qr, double seconds, int traj,
                         bool adaptive = false) {
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        left->set_logging(false);
        right->set_logging(false);
        left->set_command(ql);
        right->set_command(qr);
        const auto start = std::chrono::steady_clock::now();
        const auto until = start + std::chrono::duration<double>(seconds);
        double quiet_for = 0.0;
        auto mark = start;
        while (g_running && std::chrono::steady_clock::now() < until) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj);
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::min(0.05, std::chrono::duration<double>(now - mark).count());
            mark = now;
            quiet_for = left->quiet(QUIET_SPEED) && right->quiet(QUIET_SPEED)
                ? quiet_for + dt : 0.0;
            const Pose measured_l = left->measured(), measured_r = right->measured();
            double error = 0.0;
            for (size_t joint = 0; joint < 7; ++joint) {
                error = std::max({error, std::abs(ql[joint] - measured_l[joint]),
                                  std::abs(qr[joint] - measured_r[joint])});
            }
            if (adaptive && quiet_for >= 0.2 && error < 0.10 &&
                std::chrono::duration<double>(now - start).count() >= 0.3) {
                break;
            }
        }
    };

    auto soft_dwell = [&](int traj_id, double seconds, const char* label, bool adaptive = false) {
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
            std::cout << std::fixed << std::setprecision(3)
                      << "[calib] 未停稳，跳过 " << label
                      << "（左 " << left->motion_speed() << " rad/s，右 "
                      << right->motion_speed() << " rad/s）" << std::endl;
            return;
        }
        left->set_logging(true);
        right->set_logging(true);
        std::cout << "[calib] 软持采数 " << label << " " << seconds << " s（左右同时）" << std::endl;
        const auto sample_start = std::chrono::steady_clock::now();
        const auto until = sample_start + std::chrono::duration<double>(seconds);
        StableHoldWindow plateau;
        while (g_running && std::chrono::steady_clock::now() < until) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj_id);
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - sample_start).count();
            if (adaptive && plateau.update(elapsed,
                    left->quiet(QUIET_SPEED) && right->quiet(QUIET_SPEED),
                    left->measured(), right->measured(), left->hold_torque(), right->hold_torque())) {
                std::cout << "[calib] 双臂位置/保持力矩稳定，提前完成采数 "
                          << label << " " << elapsed << " s" << std::endl;
                break;
            }
        }
        left->set_logging(false);
        right->set_logging(false);
    };

    // Hard-tracked smoothstep ramp; the arms must already be in hard hold.
    auto ramp_hard = [&](const Pose& from_l, const Pose& from_r, const Pose& to_l,
                         const Pose& to_r, int traj, double speed = TRANSIT_JOINT_SPEED) {
        const double duration = std::max(ramp_duration(from_l, to_l, speed),
                                         ramp_duration(from_r, to_r, speed));
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
            left->set_command(lerp(from_l, to_l, s), smooth_dq(from_l, to_l, alpha, duration));
            right->set_command(lerp(from_r, to_r, s), smooth_dq(from_r, to_r, alpha, duration));
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj);
        }
    };

    // Glide the soft-spring setpoint with zero velocity feedforward, so the joint
    // follows from one side and comes to rest without being driven past it.
    auto glide_soft = [&](const Pose& from_l, const Pose& from_r, const Pose& to_l,
                          const Pose& to_r, int traj) {
        const auto glide_start = std::chrono::steady_clock::now();
        while (g_running) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - glide_start)
                    .count();
            if (elapsed >= APPROACH_RAMP_S) {
                break;
            }
            const double alpha = elapsed / APPROACH_RAMP_S;
            const double s = alpha * alpha * (3.0 - 2.0 * alpha);
            left->set_command(lerp(from_l, to_l, s));
            right->set_command(lerp(from_r, to_r, s));
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            sleep_step(t, traj);
        }
        left->set_command(to_l);
        right->set_command(to_r);
    };

    // Stiffen around the sagged pose. Raising kp with the setpoint still at goal
    // would yank the arm back by the full sag.
    auto stiffen_in_place = [&](Pose& at_l, Pose& at_r, int traj) {
        at_l = left->measured();
        at_r = right->measured();
        left->set_command(at_l);
        right->set_command(at_r);
        blend_compliance(0.0, GAIN_BLEND_S, traj);
    };

    // Entered and left in hard hold at (at_l, at_r), which is updated to where the
    // arm ends. Two-sided holds move straight on to the next ±δ start, never via goal.
    auto hold_pose = [&](const Pose& goal_l, int traj, double single_seconds, const char* label,
                         Pose& at_l, Pose& at_r) {
        const Pose goal_r = mirror_right(goal_l);
        if (!two_sided) {
            ramp_hard(at_l, at_r, goal_l, goal_r, traj);
            hard_hold(goal_l, goal_r, HARD_SNAP_S, traj);
            soft_dwell(traj, single_seconds, label);
            stiffen_in_place(at_l, at_r, traj);
            ramp_hard(at_l, at_r, goal_l, goal_r, traj);
            hard_hold(goal_l, goal_r, HARD_SNAP_S, traj);
            at_l = goal_l;
            at_r = goal_r;
            return;
        }
        for (const int sign : approach_order(at_l, at_r, goal_l)) {
            if (!g_running) {
                break;
            }
            const Pose off_l = approach_offset(goal_l, sign);
            const Pose off_r = mirror_right(off_l);
            std::cout << "[calib] " << label << " 从 " << (sign > 0 ? "+δ" : "−δ") << " 侧接近"
                      << std::endl;
            ramp_hard(at_l, at_r, off_l, off_r, traj);
            hard_hold(off_l, off_r, HARD_SNAP_S, traj, /*adaptive=*/true);
            blend_compliance(1.0, GAIN_BLEND_S, traj);
            glide_soft(off_l, off_r, goal_l, goal_r, traj);
            left->set_approach(sign);
            right->set_approach(sign);
            soft_dwell(traj, APPROACH_DWELL_S, label, /*adaptive=*/true);
            left->set_approach(0);
            right->set_approach(0);
            stiffen_in_place(at_l, at_r, traj);
        }
    };

    if (gripper_mode) {
        if (!left->gripper_installed() && !right->gripper_installed()) {
            std::cerr << "[friction] 两侧夹爪都未安装，无法辨识夹爪摩擦" << std::endl;
            return 1;
        }
        const Pose hold_l = left->measured();
        const Pose hold_r = right->measured();
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        left->set_command(hold_l);
        right->set_command(hold_r);
        // Each gripper has its own stop. A limit is "the command moved and the
        // measured position did not", judged over 0.25 s. The velocity register
        // spikes on a hard stop, so it cannot be used: it both misses the stop
        // and keeps the position loop pushing, which is what makes the jaw buzz.
        struct GripperEnd {
            ArmCalib* arm;
            double from = 0.0;
            double target = 0.0;
            double dir = 1.0;
            double anchor_q = 0.0;
            double anchor_cmd = 0.0;
            double result = 0.0;
            bool done = true;
            std::chrono::steady_clock::time_point anchor{};
        };
        auto begin_end = [&](GripperEnd* end, double target) {
            end->done = end->arm == nullptr || !end->arm->gripper_installed();
            if (end->done) {
                return;
            }
            end->from = end->arm->gripper_position();
            end->target = target;
            end->dir = target >= end->from ? 1.0 : -1.0;
            end->anchor_q = end->from;
            end->anchor_cmd = end->from;
            end->anchor = std::chrono::steady_clock::now();
            end->result = end->from;
        };
        auto release = [&](ArmCalib* arm, double q) {
            const auto release_start = std::chrono::steady_clock::now();
            while (g_running &&
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - release_start)
                           .count() < 0.20) {
                arm->set_gripper_sweep(friction_logger, q, 0.0, 0.0, false);
                const double now =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                sleep_step(now, 8);
            }
        };
        auto drive = [&](double left_target, double right_target) {
            GripperEnd ends[2] = {{left.get()}, {right.get()}};
            begin_end(&ends[0], left_target);
            begin_end(&ends[1], right_target);
            const auto drive_start = std::chrono::steady_clock::now();
            while (g_running && !(ends[0].done && ends[1].done)) {
                const auto now_tp = std::chrono::steady_clock::now();
                const double elapsed =
                    std::chrono::duration<double>(now_tp - drive_start).count();
                if (elapsed > 14.0) {
                    for (auto& end : ends) {
                        if (!end.done) {
                            end.result = end.arm->gripper_position() - end.dir * 0.15;
                            end.done = true;
                        }
                    }
                    break;
                }
                for (auto& end : ends) {
                    if (end.done) {
                        if (end.arm->gripper_installed()) {
                            end.arm->set_gripper_sweep(friction_logger, end.result, 0.0, 0.0, false);
                        }
                        continue;
                    }
                    double q_cmd = end.from + end.dir * 0.12 * elapsed;
                    const bool at_target = (end.target - q_cmd) * end.dir <= 0.0;
                    if (at_target) {
                        q_cmd = end.target;
                    }
                    end.arm->set_gripper_sweep(friction_logger, q_cmd, at_target ? 0.0 : end.dir * 0.12,
                                               0.0, false);
                    const double q = end.arm->gripper_position();
                    const double window =
                        std::chrono::duration<double>(now_tp - end.anchor).count();
                    if (window >= 0.30) {
                        const double moved = (q - end.anchor_q) * end.dir;
                        // Stiction holds the jaw still until the position loop has
                        // pulled about 0.10 rad (1 Nm). That is not a stop. A stop
                        // is that pull already present and the jaw still not moving.
                        const double lead = (q_cmd - q) * end.dir;
                        if (lead >= 0.10 && moved < 0.008) {
                            end.arm->set_gripper_sweep(friction_logger, q, 0.0, 0.0, false);
                            end.result = q - end.dir * 0.15;
                            end.done = true;
                            std::cout << "[friction] " << (end.arm == left.get() ? "left" : "right")
                                      << " 夹爪限位 q=" << q << "，退到 " << end.result << std::endl;
                        } else {
                            end.anchor_q = q;
                            end.anchor_cmd = q_cmd;
                            end.anchor = now_tp;
                        }
                    }
                    if (!end.done && at_target &&
                        std::abs(end.arm->gripper_tracking_error()) < 0.04) {
                        end.result = end.target;
                        end.done = true;
                    }
                }
                const double now = std::chrono::duration<double>(now_tp - t0).count();
                sleep_step(now, 8);
            }
            for (auto& end : ends) {
                if (end.arm != nullptr && end.arm->gripper_installed()) {
                    release(end.arm, end.result);
                }
            }
            return std::pair<double, double>{ends[0].result, ends[1].result};
        };
        std::cout << "[friction] 左右夹爪分别探行程：位置不再前进才算限位，碰到后立刻松开并退回"
                  << std::endl;
        const auto travel_open = drive(kGripperLo, kGripperLo);
        const auto travel_closed = drive(kGripperHi, kGripperHi);
        const double left_lo = std::min(travel_open.first, travel_closed.first);
        const double left_hi = std::max(travel_open.first, travel_closed.first);
        const double right_lo = std::min(travel_open.second, travel_closed.second);
        const double right_hi = std::max(travel_open.second, travel_closed.second);
        const bool scan_left = left->gripper_installed() && left_hi - left_lo >= 0.30;
        const bool scan_right = right->gripper_installed() && right_hi - right_lo >= 0.30;
        if (!g_running) {
            std::cout << "[friction] 已中止。FRICTION_LOG_PATH=" << friction_logger->path()
                      << std::endl;
            return 130;
        }
        if (!scan_left && !scan_right) {
            std::cerr << "[friction] 夹爪可动行程不足 0.30 rad，不扫描 left="
                      << (left_hi - left_lo) << " right=" << (right_hi - right_lo) << std::endl;
            return 1;
        }
        std::cout << "[friction] 夹爪扫描 left=[" << left_lo << ", " << left_hi << "] right=["
                  << right_lo << ", " << right_hi << "]" << std::endl;
        auto grip_pass = [&](double speed, int dir) {
            const double distance = std::max(scan_left ? left_hi - left_lo : 0.0,
                                             scan_right ? right_hi - right_lo : 0.0);
            const double ta = speed / kGripperAccel;
            const double total = distance / speed + ta;
            const auto pass_start = std::chrono::steady_clock::now();
            while (g_running) {
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - pass_start).count();
                if (elapsed >= total) {
                    break;
                }
                double s = 0.0, v = 0.0;
                if (elapsed < ta) {
                    s = 0.5 * kGripperAccel * elapsed * elapsed;
                    v = kGripperAccel * elapsed;
                } else if (elapsed < total - ta) {
                    s = 0.5 * kGripperAccel * ta * ta + speed * (elapsed - ta);
                    v = speed;
                } else {
                    const double left_t = total - elapsed;
                    s = distance - 0.5 * kGripperAccel * left_t * left_t;
                    v = kGripperAccel * left_t;
                }
                const double progress = distance > 1e-6 ? std::clamp(s / distance, 0.0, 1.0) : 0.0;
                const bool cruise = elapsed >= ta + kSweepSettleS &&
                                    elapsed <= total - ta - kSweepSettleS;
                auto command_jaw = [&](ArmCalib* arm, bool scan, double lo, double hi) {
                    if (!arm->gripper_installed()) {
                        return;
                    }
                    if (!scan) {
                        arm->set_gripper_sweep(friction_logger, arm->gripper_position(), 0.0, 0.0,
                                               false);
                        return;
                    }
                    const double span = hi - lo;
                    const double q = dir > 0 ? lo + progress * span : hi - progress * span;
                    const double jaw_v = distance > 1e-6 ? dir * v * (span / distance) : 0.0;
                    // Past the probed stop the position loop buzzes. Stop pushing.
                    if ((q - arm->gripper_position()) * dir > 0.10) {
                        arm->set_gripper_sweep(friction_logger, arm->gripper_position(), 0.0, 0.0,
                                               false);
                        return;
                    }
                    arm->set_gripper_sweep(friction_logger, q, jaw_v, std::abs(jaw_v), cruise);
                };
                command_jaw(left.get(), scan_left, left_lo, left_hi);
                command_jaw(right.get(), scan_right, right_lo, right_hi);
                const double now =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                sleep_step(now, 8);
            }
        };
        drive(left_lo, right_lo);
        for (const double speed : kGripperSpeeds) {
            for (const int dir : {1, -1}) {
                if (!g_running) {
                    break;
                }
                std::cout << "[friction] 夹爪 " << (dir > 0 ? "闭合方向" : "张开方向") << " "
                          << speed << " rad/s" << std::endl;
                grip_pass(speed, dir);
            }
        }
        left->stop_gripper_sweep();
        right->stop_gripper_sweep();
        friction_logger->flush();
        if (!g_running) {
            std::cout << "[friction] 已中止。FRICTION_LOG_PATH=" << friction_logger->path()
                      << std::endl;
            return 130;
        }
        const double minutes =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / 60.0;
        std::cout << "[friction] 夹爪扫描完成，用时 " << std::fixed << std::setprecision(1)
                  << minutes << " 分钟。FRICTION_LOG_PATH=" << friction_logger->path() << std::endl;
        return 0;
    }

    if (friction_mode) {
        Pose at_l = left->measured();
        Pose at_r = right->measured();
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        // Trapezoid from `lo` to `hi` (dir +1) or back (dir −1) at cruise `speed`.
        auto pass = [&](const FrictionSweep& sweep, double speed, int dir) {
            const double distance = sweep.hi - sweep.lo;
            const double ta = speed / kSweepAccel;
            const double total = distance / speed + ta;
            const auto pass_start = std::chrono::steady_clock::now();
            while (g_running) {
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - pass_start).count();
                if (t >= total) {
                    break;
                }
                double s = 0.0, v = 0.0;
                if (t < ta) {
                    s = 0.5 * kSweepAccel * t * t;
                    v = kSweepAccel * t;
                } else if (t < total - ta) {
                    s = 0.5 * kSweepAccel * ta * ta + speed * (t - ta);
                    v = speed;
                } else {
                    const double left_t = total - t;
                    s = distance - 0.5 * kSweepAccel * left_t * left_t;
                    v = kSweepAccel * left_t;
                }
                Pose q_l = sweep.base;
                Pose dq_l{};
                q_l[sweep.joint] = dir > 0 ? sweep.lo + s : sweep.hi - s;
                dq_l[sweep.joint] = dir * v;
                const bool cruise = t >= ta + kSweepSettleS && t <= total - ta - kSweepSettleS;
                left->set_sweep(friction_logger, sweep.joint, speed, cruise);
                right->set_sweep(friction_logger, sweep.joint, speed, cruise);
                left->set_command(q_l, dq_l);
                right->set_command(mirror_right(q_l), mirror_right(dq_l));
                const double now =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                sleep_step(now, 400 + sweep.joint);
            }
            left->set_sweep(friction_logger, sweep.joint, speed, false);
            right->set_sweep(friction_logger, sweep.joint, speed, false);
        };
        for (const auto& sweep : kFrictionSweeps) {
            if (!g_running) {
                break;
            }
            Pose start_l = sweep.base;
            start_l[sweep.joint] = sweep.lo;
            const Pose start_r = mirror_right(start_l);
            std::cout << "[friction] J" << (sweep.joint + 1) << " 移到起点，扫描 [" << sweep.lo
                      << ", " << sweep.hi << "] rad" << std::endl;
            left->set_sweep(friction_logger, sweep.joint, 0.0, false);
            right->set_sweep(friction_logger, sweep.joint, 0.0, false);
            const bool first = &sweep == &kFrictionSweeps.front();
            ramp_hard(at_l, at_r, start_l, start_r, 400 + sweep.joint,
                      first ? MAX_JOINT_SPEED : TRANSIT_JOINT_SPEED);
            hard_hold(start_l, start_r, HARD_SNAP_S, 400 + sweep.joint, /*adaptive=*/true);
            for (const double speed : kSweepSpeeds) {
                for (const int dir : {1, -1}) {
                    if (!g_running) {
                        break;
                    }
                    std::cout << "[friction] J" << (sweep.joint + 1) << " "
                              << (dir > 0 ? "正向" : "反向") << " " << speed << " rad/s"
                              << std::endl;
                    pass(sweep, speed, dir);
                    Pose end_l = sweep.base;
                    end_l[sweep.joint] = dir > 0 ? sweep.hi : sweep.lo;
                    hard_hold(end_l, mirror_right(end_l), 0.4, 400 + sweep.joint);
                }
            }
            at_l = sweep.base;
            at_l[sweep.joint] = sweep.lo;
            at_r = mirror_right(at_l);
            friction_logger->flush();
        }
        left->set_sweep(nullptr, -1, 0.0, false);
        right->set_sweep(nullptr, -1, 0.0, false);
        friction_logger->flush();
        if (!g_running) {
            std::cout << "[friction] 已中止。FRICTION_LOG_PATH=" << friction_logger->path()
                      << std::endl;
            return 130;
        }
        const double minutes =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / 60.0;
        std::cout << "[friction] 完成，用时 " << std::fixed << std::setprecision(1) << minutes
                  << " 分钟。FRICTION_LOG_PATH=" << friction_logger->path() << std::endl;
        return 0;
    }

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
    }

    Pose from_l = home_l;
    Pose from_r = home_r;
    hold_pose(home_l, home_traj, SETTLE_S, waypoints.front().name, from_l, from_r);
    for (size_t index = 1; index < waypoints.size() && g_running; ++index) {
        const auto& wp = waypoints[index];
        const Pose& left_goal = wp.q;
        const Pose right_goal = mirror_right(left_goal);
        const int traj_id = wp.traj_id;
        const double duration =
            std::max(ramp_duration(from_l, left_goal, TRANSIT_JOINT_SPEED),
                     ramp_duration(from_r, right_goal, TRANSIT_JOINT_SPEED));
        std::cout << "[calib] " << index << "/" << (waypoints.size() - 1) << " → " << wp.name
                  << " traj=" << traj_id
                  << "  L=[" << left_goal[0] << "," << left_goal[1] << ",…," << left_goal[4]
                  << "," << left_goal[5] << "," << left_goal[6] << "]"
                  << "  R=[" << right_goal[0] << "," << right_goal[1] << ",…," << right_goal[4]
                  << "," << right_goal[5] << "," << right_goal[6] << "]"
                  << "  ramp=" << std::fixed << std::setprecision(1) << duration << "s"
                  << " dwell="
                  << (two_sided ? 2.0 * APPROACH_DWELL_S : DWELL_S) << "s" << std::endl;
        left->set_soft_hold(false);
        right->set_soft_hold(false);
        left->set_logging(false);
        right->set_logging(false);
        hold_pose(left_goal, traj_id, DWELL_S, wp.name, from_l, from_r);
    }
    if (g_running) {
        const Pose last_l = waypoints.back().q;
        const Pose last_r = mirror_right(last_l);
        ramp_hard(from_l, from_r, last_l, last_r, waypoints.back().traj_id);
        hard_hold(last_l, last_r, HARD_SNAP_S, waypoints.back().traj_id);
    }

    logger->flush();
    if (!g_running) {
        std::cout << "[calib] 已中止。日志：" << logger->path() << std::endl;
        return 130;
    }
    const double elapsed_minutes = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count() / 60.0;
    std::cout << "[calib] 完成，采数用时 " << std::fixed << std::setprecision(1)
              << elapsed_minutes << " 分钟（含初始同步）。LOG_PATH=" << logger->path() << std::endl;
    return 0;
}
