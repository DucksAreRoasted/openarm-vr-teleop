// Zero-gravity: official OpenArm teleop gravity_comp.
// MIT {kp=0, kd=0, τ=G(q)} — no position spring, no gain discount.
// https://github.com/enactic/openarm_teleop/blob/main/control/gravity_compasation.cpp

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <controller/dynamics.hpp>
#include <controller/follower_config.hpp>
#include <controller/gravity_comp.hpp>
#include <controller/startup_trajectory.hpp>
#include <io/teleop_io.hpp>
#include <io/joint_angle_stream.hpp>
#include <openarm_port/can_feedback.hpp>
#include <openarm_port/gravity_hardware.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdio>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <csignal>
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

std::atomic<bool> g_running{true};
std::atomic<bool> g_reload{false};

void on_signal(int) { g_running = false; }
void on_hup(int) { g_reload = true; }

constexpr double kFingerComX = 0.0;
constexpr double kFingerComY = 0.0;
constexpr double kFingerComZ = 0.1025 + 0.045;

void apply_gravity_model(Dynamics* dynamics, const GravityModel& model, const std::string& side,
                         bool reinit_links = false) {
    if (dynamics == nullptr) {
        return;
    }
    const auto& scales = link_mass_scales_for(model, side);
    dynamics->SetLinkMassScales(scales);
    dynamics->SetLinkComDeltas(link_com_deltas_for(model, side));
    if (reinit_links) {
        // Distal gains / camera / gripper mass must still apply even if reinit fails.
        if (!dynamics->ReinitWithMassScales(scales)) {
            std::cerr << "[zero-g] ReinitWithMassScales failed for " << side
                      << " — still applying distal/camera knobs" << std::endl;
        }
    }
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
        file_ << "t,side,traj";
        for (int joint = 1; joint <= 7; ++joint) {
            file_ << ",q_cmd" << joint << ",q_meas" << joint << ",e" << joint << ",dq" << joint
                  << ",dq_meas" << joint << ",g_model" << joint << ",tau_ff" << joint << ",mtau"
                  << joint;
        }
        file_ << "\n";
    }

    const std::string& path() const { return path_; }

    void write(double t, const std::string& side, const std::array<double, 7>& q_cmd,
               const std::array<double, 7>& q_meas, const std::array<double, 7>& dq_cmd,
               const std::array<double, 7>& dq_meas, const std::array<double, 7>& g_model,
               const std::array<double, 7>& tau_ff, const std::array<double, 7>& mtau) {
        if (!file_) {
            return;
        }
        file_ << t << "," << side << ",0";
        for (int joint = 0; joint < 7; ++joint) {
            file_ << "," << q_cmd[joint] << "," << q_meas[joint] << ","
                  << (q_cmd[joint] - q_meas[joint]) << "," << dq_cmd[joint] << "," << dq_meas[joint]
                  << "," << g_model[joint] << "," << tau_ff[joint] << "," << mtau[joint];
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

class ArmZeroG {
public:
    ArmZeroG(const std::string& can, const std::string& side, const std::string& urdf,
             GravityTune tune, GravityModel model, std::shared_ptr<CsvLogger> logger)
        : side_(side), tune_(tune), model_(model), logger_(std::move(logger)) {
        hardware_ = std::make_unique<openarm::can::socket::OpenArm>(can, true);
        initialize_gravity_motors(*hardware_, model_, side_);
        hardware_->enable_all();
        enabled_ = true;
        for (int cycle = 0; cycle < 20; ++cycle) {
            hardware_->refresh_all();
            hardware_->recv_all(2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const CanFeedbackVerdict verdict = refresh_until_fresh(hardware_.get());
        if (!verdict.ok) {
            const std::string detail = format_can_feedback_verdict(verdict, "[zero-g] " + side_);
            std::cerr << detail << std::endl;
            if (verdict.send_can_id == 0x08) {
                std::cerr << "[zero-g] 如夹爪已拆卸，请在重力补偿中保存该侧夹爪为未安装，再重启。"
                          << std::endl;
            }
            std::cerr << "[zero-g] " << side_
                      << " 没有新鲜有效反馈，拒绝进入出力循环（不按 q=0 计算重力）" << std::endl;
            try {
                hardware_->disable_all();
            } catch (...) {
            }
            enabled_ = false;
            throw std::runtime_error(detail);
        }
        dynamics_ = std::make_unique<Dynamics>(urdf, "openarm_body_link0",
                                               "openarm_" + side_ + "_link7");
        dynamics_->SetLinkMassScales(link_mass_scales_for(model_, side_));
        dynamics_->SetLinkComDeltas(link_com_deltas_for(model_, side_));
        if (!dynamics_->Init()) {
            dynamics_.reset();
            throw std::runtime_error(side_ + " 重力模型初始化失败");
        }
        apply_gravity_model(dynamics_.get(), model_, side_);
        joint_angle_stream_ = JointAngleStream::create(side_);
        read();
        gripper_hold_ = gripper_;
        std::cout << "[zero-g] " << side_ << " 重力补偿 kp=0 kd=0 "
                  << "τ=ff*(s*G+b) can=" << can << " urdf_scale=" << model_.urdf_scale
                  << " ff_gain=" << model_.ff_gain
                  << " gripper=" << (gripper_present_for(model_, side_) ? "installed" : "removed")
                  << " ee_kg=" << end_effector_mass_for(model_, side_)
                  << " camera_kg=" << wrist_camera_mass_for(model_, side_)
                  << " shoulder_scale=[" << tune_.scale[0] << "," << tune_.scale[1] << "]"
                  << " shoulder_bias=[" << tune_.bias[0] << "," << tune_.bias[1] << "]"
                  << " shoulder_fc=[" << tune_.fc[0] << "," << tune_.fc[1] << "]"
                  << " scale0=" << tune_.scale[0] << " bias0=" << tune_.bias[0]
                  << " dm4340_fc=[" << tune_.fc[2] << "," << tune_.fc[3] << "]" << std::endl;
    }

    ~ArmZeroG() {
        if (hardware_ && enabled_) {
            try {
                for (int cycle = 0; cycle < 30; ++cycle) {
                    std::vector<openarm::damiao_motor::MITParam> arm;
                    arm.reserve(7);
                    for (size_t index = 0; index < measured_.size(); ++index) {
                        arm.push_back({0.0, 0.0, 0.0, 0.0, 0.0});
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

    void set_tune(const GravityTune& tune) {
        std::lock_guard<std::mutex> lock(mutex_);
        tune_ = tune;
        // Preserve the supplied friction values, including zero.
    }

    void set_model(const GravityModel& model) {
        std::lock_guard<std::mutex> lock(mutex_);
        model_ = model;
        apply_gravity_model(dynamics_.get(), model_, side_, /*reinit_links=*/true);
    }

    struct Sample {
        std::array<double, 7> q{};
        std::array<double, 7> dq{};
        std::array<double, 7> tau{};
        double grip = 0.0;
        double grip_dq = 0.0;
        bool ok = false;
    };

    Sample sample() {
        std::lock_guard<std::mutex> lock(mutex_);
        Sample out;
        out.q = measured_;
        out.dq = measured_vel_;
        out.tau = measured_tau_;
        out.grip = gripper_;
        out.grip_dq = gripper_vel_;
        out.ok = enabled_ && !fault_;
        return out;
    }

    void set_track_gains(std::vector<double> kp, std::vector<double> kd) {
        std::lock_guard<std::mutex> lock(mutex_);
        track_kp_ = std::move(kp);
        track_kd_ = std::move(kd);
    }

    // Position tracking for the shared startup CSV. Returns false if feedback
    // is lost or a joint is more than 0.45 rad behind the reference.
    bool recover_feedback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hardware_ || !enabled_ || fault_) {
            return false;
        }
        const CanFeedbackVerdict verdict = refresh_until_fresh(hardware_.get());
        read_locked();
        if (!verdict.ok) {
            std::cerr << format_can_feedback_verdict(verdict, "[drag-teach] " + side_) << std::endl;
        }
        return verdict.ok;
    }

    bool track(const std::array<double, 8>& q, const std::array<double, 8>& dq) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_ || fault_ || track_kp_.size() < 8 || track_kd_.size() < 8) {
            return false;
        }
        std::string detail;
        if (!can_feedback_allows_torque(hardware_.get(), "[zero-g] " + side_, &detail)) {
            const CanFeedbackVerdict verdict = refresh_until_fresh(hardware_.get());
            read_locked();
            if (!verdict.ok) {
                std::cerr << detail << std::endl;
                std::cerr << "[drag-teach] " << side_ << " 反馈仍过期，取消前往起手姿势" << std::endl;
                return false;
            }
        }
        std::array<double, 7> gravity{};
        dynamics_->GetGravity(measured_.data(), gravity.data());
        std::vector<openarm::damiao_motor::MITParam> arm;
        arm.reserve(7);
        for (size_t index = 0; index < measured_.size(); ++index) {
            const double tau_ff = compose_tau_ff(gravity[index], tune_, index, measured_vel_[index],
                                                 model_.ff_gain);
            arm.push_back({track_kp_[index], track_kd_[index], q[index], dq[index], tau_ff});
            if (std::abs(q[index] - measured_[index]) > 0.45) {
                std::cerr << "[drag-teach] " << side_ << " 关节 " << (index + 1)
                          << " 偏离起手轨迹超过 0.45 rad，停止前往" << std::endl;
                return false;
            }
        }
        hardware_->get_arm().mit_control_all(arm);
        if (gripper_present_for(model_, side_)) {
            hardware_->get_gripper().mit_control_all(
                {{track_kp_[7], track_kd_[7], q[7], dq[7], 0.0}});
        }
        hardware_->recv_all(1000);
        read_locked();
        return true;
    }

    void step(double t_sec) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_ || fault_) {
            return;
        }
        std::string detail;
        if (!can_feedback_allows_torque(hardware_.get(), "[zero-g] " + side_, &detail)) {
            std::cerr << detail << std::endl;
            std::cerr << "[zero-g] " << side_ << " 反馈过期，停止下发并 disable_all" << std::endl;
            try {
                hardware_->disable_all();
            } catch (...) {
            }
            enabled_ = false;
            fault_ = true;
            g_running = false;
            return;
        }
        std::array<double, 7> gravity{};
        std::array<double, 7> zero{};

        // τ = ff_gain*(scale*G(q)+bias), kp=kd=0. Friction is not added.
        // Apply the scale/bias retained or updated by automatic/manual refinement.
        dynamics_->GetGravity(measured_.data(), gravity.data());
        std::array<double, 7> tau_ff{};
        std::vector<openarm::damiao_motor::MITParam> arm;
        arm.reserve(7);
        // Light Kd on wrist (j5–j7) only: damps drift from off-axis camera COM residuals
        // without adding a position spring (still kp=0).
        const double wrist_kd = std::clamp(model_.wrist_zero_g_kd, 0.0, 1.5);
        for (size_t index = 0; index < measured_.size(); ++index) {
            tau_ff[index] = compose_tau_ff(gravity[index], tune_, index, measured_vel_[index],
                                           model_.ff_gain);
        }
        for (size_t index = 0; index < measured_.size(); ++index) {
            const double kd = (index >= 4) ? wrist_kd : 0.0;
            arm.push_back({0.0, kd, 0.0, 0.0, tau_ff[index]});
        }
        hardware_->get_arm().mit_control_all(arm);
        // Joint 8 (DM4310 gripper): free drag, no friction feedforward.
        if (gripper_present_for(model_, side_)) {
            hardware_->get_gripper().mit_control_all({{0.0, 0.0, 0.0, 0.0, 0.0}});
        }
        hardware_->recv_all(1000);
        read_locked();
        if (logger_ && (++log_div_ % 2) == 0) {
            logger_->write(t_sec, side_, measured_, measured_, zero, measured_vel_, gravity, tau_ff,
                           measured_tau_);
        }
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
        if (motors.size() >= 7 && joint_angle_stream_) joint_angle_stream_->update(measured_);
        const auto gripper = hardware_->get_gripper().get_motors();
        if (!gripper.empty()) {
            gripper_ = gripper.front().get_position();
            gripper_vel_ = gripper.front().get_velocity();
        }
    }

    std::unique_ptr<JointAngleStream> joint_angle_stream_;
    std::string side_;
    GravityTune tune_;
    GravityModel model_;
    std::shared_ptr<CsvLogger> logger_;
    std::unique_ptr<openarm::can::socket::OpenArm> hardware_;
    std::unique_ptr<Dynamics> dynamics_;
    std::mutex mutex_;
    std::array<double, 7> measured_{};
    std::array<double, 7> measured_vel_{};
    std::array<double, 7> measured_tau_{};
    double gripper_ = 0.0;
    double gripper_vel_ = 0.0;
    double gripper_hold_ = 0.0;
    bool enabled_ = false;
    bool fault_ = false;
    int log_div_ = 0;
    std::vector<double> track_kp_;
    std::vector<double> track_kd_;
};

class DragTeachRecorder {
public:
    explicit DragTeachRecorder(std::string sessions_root)
        : sessions_root_(std::move(sessions_root)) {}

    bool open_udp() {
        const char* port_env = std::getenv("OPENARM_QUEST_UI_RECORD_PORT");
        uint16_t port = 47930;
        if (port_env && *port_env) {
            port = static_cast<uint16_t>(std::atoi(port_env));
        }
        if (!rx_.open(port, "drag-teach")) {
            std::cerr << "[drag-teach] 录制口 " << port << " 绑定失败，拖动示教不能记录" << std::endl;
            return false;
        }
        std::cout << "[drag-teach] 录制 UDP 127.0.0.1:" << port
                  << "  sessions=" << sessions_root_ << std::endl;
        return true;
    }

    void poll() {
        for (const auto& message : rx_.receive_all()) {
            handle(message);
        }
    }

    void set_motion_busy(bool busy) { motion_busy_ = busy; }

    bool consume_home() {
        const bool requested = home_request_;
        home_request_ = false;
        return requested;
    }

    bool recording() const { return file_.is_open(); }

    void write(const ArmZeroG::Sample& left, const ArmZeroG::Sample& right) {
        if (!file_.is_open() || !left.ok || !right.ok) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!timing_) {
            t0_ = now;
            timing_ = true;
            last_t_ = -1.0;
        }
        double t = std::chrono::duration<double>(now - t0_).count();
        if (t <= last_t_) {
            t = last_t_ + 1e-4;
        }
        last_t_ = t;
        file_ << std::setprecision(9) << t;
        write_arm(left);
        write_arm(right);
        file_ << "\n";
        ++rows_;
        if ((rows_ % 20) == 0) {
            file_.flush();
        }
    }

    void close_if_open() {
        if (file_.is_open()) {
            finish(/*announce=*/true);
        }
    }

private:
    static std::string trim(std::string text) {
        while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
            text.erase(text.begin());
        }
        while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
            text.pop_back();
        }
        return text;
    }

    static std::string sanitize(std::string name) {
        for (char& c : name) {
            const unsigned char uc = static_cast<unsigned char>(c);
            if (!(std::isalnum(uc) || c == '_' || c == '-' || c == '.')) {
                c = '_';
            }
        }
        while (!name.empty() && (name.front() == '_' || name.front() == '.' || name.front() == '-')) {
            name.erase(name.begin());
        }
        if (name.empty()) {
            name = "drag_teach";
        }
        return name;
    }

    int next_segment(const std::filesystem::path& batch) const {
        int next = 1;
        std::error_code ec;
        if (!std::filesystem::is_directory(batch, ec)) {
            return next;
        }
        for (const auto& entry : std::filesystem::directory_iterator(batch, ec)) {
            if (!entry.is_directory()) {
                continue;
            }
            const std::string name = entry.path().filename().string();
            if (name.empty() || !std::all_of(name.begin(), name.end(), ::isdigit)) {
                continue;
            }
            try {
                next = std::max(next, std::stoi(name) + 1);
            } catch (const std::exception&) {
            }
        }
        return next;
    }

    void write_arm(const ArmZeroG::Sample& sample) {
        for (size_t i = 0; i < 7; ++i) {
            file_ << "," << sample.q[i] << "," << sample.dq[i] << "," << sample.tau[i];
        }
        file_ << "," << sample.grip << "," << sample.grip_dq << ",0";
    }

    void finish(bool announce) {
        if (file_.is_open()) {
            file_.flush();
            file_.close();
        }
        if (rows_ < 2) {
            std::cerr << "[drag-teach] 样本不足 2 帧，这段不能回放：" << session_dir_ << std::endl;
        } else if (announce) {
            std::cout << "[quest-ui] recording stopped" << std::endl;
            std::cout << "[quest-ui] replay-ready session: " << session_dir_ << std::endl;
            std::cout << "  merged : " << csv_path_ << std::endl;
        }
        rows_ = 0;
        timing_ = false;
        last_t_ = -1.0;
    }

    void handle(const std::string& message) {
        std::string text = trim(message);
        std::string cmd = text;
        std::string arg;
        const auto comma = text.find(',');
        if (comma != std::string::npos) {
            cmd = trim(text.substr(0, comma));
            arg = trim(text.substr(comma + 1));
        }
        for (char& c : cmd) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        if (cmd == "home") {
            if (file_.is_open() || motion_busy_) {
                std::cerr << "[drag-teach] 正在记录或前往起手姿势，暂不能再次前往" << std::endl;
                return;
            }
            home_request_ = true;
            return;
        }
        if (cmd == "start") {
            if (motion_busy_) {
                std::cerr << "[drag-teach] 正在前往起手姿势，到达后再录制" << std::endl;
                return;
            }
            if (file_.is_open()) {
                std::cerr << "[drag-teach] 已在记录，先结束这一段" << std::endl;
                return;
            }
            const auto batch = std::filesystem::path(sessions_root_) / sanitize(arg);
            std::error_code ec;
            std::filesystem::create_directories(batch, ec);
            if (ec) {
                std::cerr << "[drag-teach] 建目录失败 " << batch << ": " << ec.message() << std::endl;
                return;
            }
            const int segment = next_segment(batch);
            char folder[16];
            std::snprintf(folder, sizeof(folder), "%02d", segment);
            session_dir_ = (batch / folder).string();
            std::filesystem::create_directories(session_dir_, ec);
            csv_path_ = session_dir_ + "/real_teleop_dual_merged.csv";
            file_.open(csv_path_);
            if (!file_) {
                std::cerr << "[drag-teach] 打不开 " << csv_path_ << std::endl;
                return;
            }
            rows_ = 0;
            file_ << "time";
            for (int i = 0; i < 16; ++i) {
                file_ << ",q" << i << ",dq" << i << ",tau" << i;
            }
            file_ << "\n";
            std::cout << "[quest-ui] recording segment " << folder << " -> " << session_dir_
                      << std::endl;
            std::cout << "  source=measured q/dq with wall-clock timestamps" << std::endl;
        } else if (cmd == "stop") {
            if (!file_.is_open()) {
                std::cerr << "[drag-teach] 当前没有在记录" << std::endl;
                return;
            }
            finish(true);
        }
    }

    std::string sessions_root_;
    UdpTextReceiver rx_;
    std::ofstream file_;
    std::string session_dir_;
    std::string csv_path_;
    std::size_t rows_ = 0;
    bool timing_ = false;
    bool home_request_ = false;
    bool motion_busy_ = false;
    double last_t_ = -1.0;
    std::chrono::steady_clock::time_point t0_{};
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
    std::signal(SIGHUP, on_hup);

    const std::string root = env_or("OPENARM_ROOT", "");
    if (root.empty()) {
        std::cerr << "[zero-g] 需要 OPENARM_ROOT" << std::endl;
        return 1;
    }
    const std::string urdf = env_or("OPENARM_URDF", root + "/assets/v1.urdf");
    const std::string yaml =
        env_or("OPENARM_GRAVITY_YAML", root + "/config/gravity_comp.yaml");
    const std::string left_can = env_or("OPENARM_LEFT_FOLLOWER_CAN", "can1");
    const std::string right_can = env_or("OPENARM_RIGHT_FOLLOWER_CAN", "can0");
    const std::string log_dir =
        env_or("OPENARM_ZERO_G_LOG_DIR", root + "/recordings/gravity");

    GravityTune left_tune;
    GravityTune right_tune;
    GravityModel model;
    if (!load_gravity_yaml(yaml, &left_tune, &right_tune, &model)) {
        std::cerr << "[zero-g] 读不到 " << yaml << "，用解析默认（scale=1 + 指尖质量）" << std::endl;
        seed_default_friction(&left_tune);
        seed_default_friction(&right_tune);
    } else {
        // Loaded friction values, including explicit zeros, are authoritative.
        std::cout << "[zero-g] 已加载 " << yaml << " urdf_scale=" << model.urdf_scale
                  << " ee_payload=" << model.ee_payload_kg << std::endl;
    }

    auto logger = std::make_shared<CsvLogger>(log_dir);
    if (logger->path().empty()) {
        std::cerr << "[zero-g] 打不开日志目录 " << log_dir << std::endl;
        return 1;
    }
    std::cout << "[zero-g] 日志：" << logger->path() << std::endl;
    std::cout << "[zero-g] 模式：kp=0 kd=0，τ=ff_gain*(scale*G+bias)，不加摩擦。"
              << std::endl;
    std::cout << "[zero-g] 拖动示教：托住后在控制台点开始录制，记下的是编码器实测关节。"
              << std::endl;

    DragTeachRecorder teach(root + "/recordings/sessions");
    teach.open_udp();

    std::unique_ptr<ArmZeroG> left;
    std::unique_ptr<ArmZeroG> right;
    try {
        left = std::make_unique<ArmZeroG>(left_can, "left", urdf, left_tune, model, logger);
        right = std::make_unique<ArmZeroG>(right_can, "right", urdf, right_tune, model, logger);
    } catch (const std::exception& error) {
        std::cerr << "[zero-g] 启动失败：" << error.what() << std::endl;
        return 1;
    }

    try {
        auto params = load_follower_params();
        apply_quest_vr_follower_mit_gains(&params.kp, &params.kd, false, "drag-teach");
        if (params.kp.size() < 8 || params.kd.size() < 8) {
            throw std::runtime_error("follower gains shorter than 8");
        }
        left->set_track_gains(params.kp, params.kd);
        right->set_track_gains(params.kp, params.kd);
    } catch (const std::exception& error) {
        std::cerr << "[drag-teach] 读不到跟随增益，无法前往起手姿势：" << error.what() << std::endl;
    }
    const std::string startup_path =
        env_or("OPENARM_QUEST_STARTUP_FILE", root + "/assets/steady_action_on_table.csv");
    std::unique_ptr<StartupTrajectory> startup;
    bool homing = false;
    bool holding_home = false;
    TeachingTrajectory::Sample home_hold{};
    double home_t = 0.0;
    auto home_clock = std::chrono::steady_clock::now();

    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    while (g_running) {
        if (g_reload.exchange(false)) {
            GravityTune l;
            GravityTune r;
            GravityModel m;
            if (load_gravity_yaml(yaml, &l, &r, &m)) {
                if (m.left_gripper_present != model.left_gripper_present ||
                    m.right_gripper_present != model.right_gripper_present) {
                    std::cerr << "[zero-g] 夹爪安装状态改变，需要停止并重启；仍使用当前参数。" << std::endl;
                    continue;
                }
                // Keep explicit zero friction during hot reload as well.
                left->set_tune(l);
                right->set_tune(r);
                left->set_model(m);
                right->set_model(m);
                model = m;
                std::cout << "[zero-g] 已热加载重力参数" << std::endl;
            } else {
                std::cerr << "[zero-g] 热加载失败，仍用旧参数" << std::endl;
            }
        }
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        teach.set_motion_busy(homing);
        teach.poll();
        if (!homing && teach.consume_home()) {
            try {
                const auto left_now = left->sample();
                const auto right_now = right->sample();
                if (!left_now.ok || !right_now.ok) {
                    throw std::runtime_error("arm not ready");
                }
                TeachingTrajectory::Pose pose{};
                for (int i = 0; i < 7; ++i) {
                    pose[i] = left_now.q[static_cast<size_t>(i)];
                    pose[8 + i] = right_now.q[static_cast<size_t>(i)];
                }
                pose[7] = left_now.grip;
                pose[15] = right_now.grip;
                startup = std::make_unique<StartupTrajectory>();
                startup->reset(startup_path, pose, false);
                if (!left->recover_feedback() || !right->recover_feedback()) {
                    throw std::runtime_error("CAN feedback was not fresh after loading the startup trajectory");
                }
                homing = true;
                home_t = 0.0;
                home_clock = std::chrono::steady_clock::now();
                std::cout << "[drag-teach] 前往起手姿势 " << startup_path
                          << " 时长 " << startup->duration() << " s" << std::endl;
            } catch (const std::exception& error) {
                homing = false;
                startup.reset();
                std::cerr << "[drag-teach] 无法前往起手姿势：" << error.what() << std::endl;
            }
        }
        if (homing && startup) {
            const auto now = std::chrono::steady_clock::now();
            home_t += std::max(0.0, std::chrono::duration<double>(now - home_clock).count());
            home_clock = now;
            bool arrived = false;
            try {
                const auto goal = startup->sample(std::min(home_t, startup->duration()));
                std::array<double, 8> left_q{};
                std::array<double, 8> left_dq{};
                std::array<double, 8> right_q{};
                std::array<double, 8> right_dq{};
                for (int i = 0; i < 8; ++i) {
                    left_q[static_cast<size_t>(i)] = goal.position[static_cast<size_t>(i)];
                    left_dq[static_cast<size_t>(i)] = goal.velocity[static_cast<size_t>(i)];
                    right_q[static_cast<size_t>(i)] = goal.position[static_cast<size_t>(8 + i)];
                    right_dq[static_cast<size_t>(i)] = goal.velocity[static_cast<size_t>(8 + i)];
                }
                if (!left->track(left_q, left_dq) || !right->track(right_q, right_dq)) {
                    throw std::runtime_error("tracking stopped");
                }
                arrived = home_t >= startup->duration();
            } catch (const std::exception& error) {
                homing = false;
                startup.reset();
                std::cerr << "[drag-teach] 已停止前往起手姿势，回到可拖动状态："
                          << error.what() << std::endl;
            }
            if (homing && arrived) {
                home_hold = startup->sample(startup->duration());
                home_hold.velocity.fill(0);
                homing = false;
                holding_home = true;
                startup.reset();
                std::cout << "[drag-teach] 已到达起手姿势，保持左右镜像。点开始录制后松开，即可拖动。"
                          << std::endl;
            }
        } else if (holding_home && !teach.recording()) {
            try {
                std::array<double, 8> left_q{};
                std::array<double, 8> left_dq{};
                std::array<double, 8> right_q{};
                std::array<double, 8> right_dq{};
                for (int i = 0; i < 8; ++i) {
                    left_q[static_cast<size_t>(i)] = home_hold.position[static_cast<size_t>(i)];
                    right_q[static_cast<size_t>(i)] = home_hold.position[static_cast<size_t>(8 + i)];
                }
                if (!left->track(left_q, left_dq) || !right->track(right_q, right_dq)) {
                    throw std::runtime_error("tracking stopped");
                }
            } catch (const std::exception& error) {
                holding_home = false;
                std::cerr << "[drag-teach] 已松开起手姿势：" << error.what() << std::endl;
            }
        } else {
            if (holding_home) {
                holding_home = false;
                std::cout << "[drag-teach] 已松开起手姿势，可以拖动" << std::endl;
            }
            left->step(t);
            right->step(t);
            teach.write(left->sample(), right->sample());
        }
        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }

    teach.close_if_open();
    logger->flush();
    std::cout << "[zero-g] 已退出。日志：" << logger->path() << std::endl;
    return 0;
}
