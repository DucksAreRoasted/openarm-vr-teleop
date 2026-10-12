// Zero-gravity: official OpenArm teleop gravity_comp.
// MIT {kp=0, kd=0, τ=G(q)} — no position spring, no gain discount.
// https://github.com/enactic/openarm_teleop/blob/main/control/gravity_compasation.cpp

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
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

constexpr double kFingerMassKg = 2.0 * 0.03602545343277134;
// Gripper motor (joint 8) default if yaml omits gripper_motor_kg.
constexpr double kGripperMotorKgDefault = 0.12;
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
        dynamics_ = std::make_unique<Dynamics>(urdf, "openarm_body_link0",
                                               "openarm_" + side_ + "_link7");
        dynamics_->SetLinkMassScales(link_mass_scales_for(model_, side_));
        dynamics_->SetLinkComDeltas(link_com_deltas_for(model_, side_));
        if (!dynamics_->Init()) {
            dynamics_.reset();
            throw std::runtime_error(side_ + " 重力模型初始化失败");
        }
        apply_gravity_model(dynamics_.get(), model_, side_);
        read();
        gripper_hold_ = gripper_;
        std::cout << "[zero-g] " << side_ << " 重力补偿 kp=0 kd=0 "
                  << "τ=ff*(s*G+b)+摩擦 can=" << can << " urdf_scale=" << model_.urdf_scale
                  << " ff_gain=" << model_.ff_gain
                  << " ee_kg=" << (kFingerMassKg + model_.gripper_motor_kg + model_.ee_payload_kg)
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
        ensure_friction(&tune_);
    }

    void set_model(const GravityModel& model) {
        std::lock_guard<std::mutex> lock(mutex_);
        model_ = model;
        apply_gravity_model(dynamics_.get(), model_, side_, /*reinit_links=*/true);
    }

    void step(double t_sec) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::array<double, 7> gravity{};
        std::array<double, 7> zero{};

        // τ = ff_gain*(scale*G(q)+bias) + friction, kp=kd=0.
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
            const double kd = (index >= 4) ? wrist_kd : 0.0;
            arm.push_back({0.0, kd, 0.0, 0.0, tau_ff[index]});
        }
        hardware_->get_arm().mit_control_all(arm);
        // Joint 8 (DM4310 gripper): free drag with Coulomb/viscous FF (no position hold).
        const double gripper_tau = std::clamp(
            friction_torque(gripper_vel_, model_.gripper_friction_fc, model_.gripper_friction_fv,
                            model_.gripper_friction_k),
            -2.0, 2.0);
        hardware_->get_gripper().mit_control_all({{0.0, 0.0, 0.0, 0.0, gripper_tau}});
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
        const auto gripper = hardware_->get_gripper().get_motors();
        if (!gripper.empty()) {
            gripper_ = gripper.front().get_position();
            gripper_vel_ = gripper.front().get_velocity();
        }
    }

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
    int log_div_ = 0;
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
        ensure_friction(&left_tune);
        ensure_friction(&right_tune);
        std::cout << "[zero-g] 已加载 " << yaml << " urdf_scale=" << model.urdf_scale
                  << " ee_payload=" << model.ee_payload_kg << std::endl;
    }

    auto logger = std::make_shared<CsvLogger>(log_dir);
    if (logger->path().empty()) {
        std::cerr << "[zero-g] 打不开日志目录 " << log_dir << std::endl;
        return 1;
    }
    std::cout << "[zero-g] 日志：" << logger->path() << std::endl;
    std::cout << "[zero-g] 模式：kp=0 kd=0，τ=ff_gain*(scale*G+bias)+摩擦（惯量辨识后可残差精修）。"
              << std::endl;

    std::unique_ptr<ArmZeroG> left;
    std::unique_ptr<ArmZeroG> right;
    try {
        left = std::make_unique<ArmZeroG>(left_can, "left", urdf, left_tune, model, logger);
        right = std::make_unique<ArmZeroG>(right_can, "right", urdf, right_tune, model, logger);
    } catch (const std::exception& error) {
        std::cerr << "[zero-g] 启动失败：" << error.what() << std::endl;
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    while (g_running) {
        if (g_reload.exchange(false)) {
            GravityTune l;
            GravityTune r;
            GravityModel m;
            if (load_gravity_yaml(yaml, &l, &r, &m)) {
                ensure_friction(&l);
                ensure_friction(&r);
                left->set_tune(l);
                right->set_tune(r);
                left->set_model(m);
                right->set_model(m);
                std::cout << "[zero-g] 已热加载重力+摩擦参数" << std::endl;
            } else {
                std::cerr << "[zero-g] 热加载失败，仍用旧参数" << std::endl;
            }
        }
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        left->step(t);
        right->step(t);
        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }

    logger->flush();
    std::cout << "[zero-g] 已退出。日志：" << logger->path() << std::endl;
    return 0;
}
