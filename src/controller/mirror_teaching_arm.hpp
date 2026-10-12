#pragma once

#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <controller/mirror_reference.hpp>
#include <controller/mirror_feedback.hpp>
#include <hardware/teleop_hardware.hpp>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <openarm_port/openarm_init.hpp>

#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

// One-thread hardware ownership: both feedback validation and sends run in the servo.
class MirrorTeachingArm {
public:
    using Pose = MirrorReference::Pose;
    MirrorTeachingArm(const std::string& side, const std::string& can, const std::string& urdf,
                      const GravityTune& tune, const GravityModel& model,
                      std::shared_ptr<MujocoSimWorld> world = nullptr, bool replay_follower = false)
        : side_(side), can_name_(can), dynamics_(urdf, "openarm_body_link0", "openarm_" + side + "_link7"),
          tune_(tune), model_(model), follower_feedback_(side == "left" || replay_follower),
          gripper_on_(gripper_configured_installed(model)) {
        prepare_friction();
        apply_model(false);
        if (!dynamics_.Init()) throw std::runtime_error("mirror: gravity model init failed");
        apply_model(false);
        std::vector<double> lo, hi;
        if (!dynamics_.GetJointPositionLimits(&lo, &hi) || lo.size() != 7) {
            throw std::runtime_error("mirror: missing URDF joint limits");
        }
        std::copy(lo.begin(), lo.end(), lower.begin());
        std::copy(hi.begin(), hi.end(), upper.begin());
        lower[7] = -0.98; upper[7] = 0.62;
        if (world) {
            sim_ = std::make_shared<MujocoTeleopHardware>(world, side);
            sim_->set_home({0, 0, 0, 1.570796, 0, 0, 0}, 0);
            hardware_ = sim_;
            lower[7] = 0; upper[7] = 0.044;
        } else {
            // Initialize into neutral measured-position commands; no homing motion.
            MotorConfig config = DEFAULT_MOTOR_CONFIG;
            config.init_gripper = gripper_on_;
            can_.reset(openarm_init::OpenArmInitializer::initialize_openarm(can, config, false));
            std::cout << "[mirror] " << side_ << " can=" << can_name_
                      << " transport=CAN-FD (debug logging disabled)" << std::endl;
            hardware_ = std::make_shared<CanTeleopHardware>(can_.get());
        }
        try {
            if (can_) wait_for_feedback();
            read();
        } catch (...) {
            hardware_->disable_all();
            throw;
        }
    }
    ~MirrorTeachingArm() { stop(); }
    void stop() noexcept {
        if (hardware_) {
            try { hardware_->disable_all(); } catch (...) {}
        }
    }
    void read() {
        // These motors may report only in response to an explicit status request.
        if (can_) can_->refresh_all();
        hardware_->recv_all(1000);
        if (can_) {
            validate_feedback(can_->get_arm().get_motors());
            if (gripper_on_) validate_feedback(can_->get_gripper().get_motors());
        }
        const auto arm = hardware_->read_arm();
        if (arm.size() != 7) throw std::runtime_error("mirror: expected 7 arm motors");
        measured.clear();
        for (const auto& m : arm) measured.push_back({m.position, m.velocity, m.effort});
        if (gripper_on_) {
            const auto grip = hardware_->read_gripper();
            if (grip.size() != 1) throw std::runtime_error("mirror: expected 7+1 motors");
            measured.push_back({grip[0].position, grip[0].velocity, grip[0].effort});
        } else {
            measured.push_back({0, 0, 0});
        }
        const size_t checked = gripper_on_ ? 8 : 7;
        for (size_t i = 0; i < checked; ++i) {
            const auto& m = measured[i];
            validate_mirror_feedback(side_, !follower_feedback_, i,
                                     m.position, m.velocity, m.effort, lower[i], upper[i]);
        }
    }
    Pose pose() const {
        Pose q{};
        for (size_t i = 0; i < 8; ++i) q[i] = measured[i].position;
        return q;
    }
    Pose velocity() const {
        Pose dq{};
        for (size_t i = 0; i < 8; ++i) dq[i] = measured[i].velocity;
        return dq;
    }
    void send(const Pose& target, const Pose& velocity, bool follower) {
        speed_guard_mask = 0;
        Pose q = pose();
        std::array<double, 7> gravity{};
        dynamics_.GetGravity(q.data(), gravity.data());
        constexpr std::array<double, 7> kp{22, 22, 16, 16, 8, 8, 8};
        constexpr std::array<double, 7> kd{1.0, 1.0, 0.7, 0.7, 0.3, 0.25, 0.3};
        constexpr std::array<double, 7> effort{20, 20, 7, 7, 2, 2, 2};
        std::vector<openarm::damiao_motor::MITParam> commands;
        frame.resize(8);
        for (size_t i = 0; i < 7; ++i) {
            if (!std::isfinite(target[i]) || !std::isfinite(velocity[i]) ||
                (follower && std::abs(target[i] - q[i]) > 0.45)) {
                std::ostringstream error;
                error << "mirror: tracking error on " << side_ << " joint " << i+1
                      << " target=" << target[i] << " measured=" << q[i]
                      << " error=" << target[i]-q[i] << " limit=0.45";
                throw std::runtime_error(error.str());
            }
            double tau = compose_tau_ff(gravity[i], tune_, i, measured[i].velocity, model_.ff_gain);
            if (!std::isfinite(tau)) throw std::runtime_error("mirror: invalid gravity torque");
            // Bound the complete estimated MIT torque, including PD (not just FF).
            const double p = follower ? kp[i] : 0;
            const double d = follower ? kd[i] : (i >= 4 ? std::clamp(model_.wrist_zero_g_kd, 0.0, 1.5) : 0);
            const double pos = follower ? target[i] : q[i];
            const double vel = follower ? velocity[i] : 0;
            const double pd = p * (pos - q[i]) + d * (vel - measured[i].velocity);
            double total = tau + pd;
            if (follower) {
                const double gravity_hold = compose_tau_ff(gravity[i], tune_, i, 0, model_.ff_gain);
                total = mirror_follower_torque(total, gravity_hold, measured[i].velocity, effort[i]);
                if (std::abs(measured[i].velocity) > kMirrorSpeedGuardStart) {
                    speed_guard_mask |= 1u << i;
                }
            } else {
                total = std::clamp(total, -effort[i], effort[i]);
            }
            // Motor-side MIT PD is still active: subtract it once so the applied
            // estimated total equals the guarded torque, not guarded torque + PD.
            tau = total - pd;
            commands.push_back({p, d, pos, vel, tau});
            // Right master has no position spring: measured pose is the demonstrated action.
            frame[i] = {pos, follower ? vel : measured[i].velocity, tau};
        }
        openarm::damiao_motor::MITParam grip_cmd{};
        if (gripper_on_) {
            const double grip_pos = follower ? target[7] : q[7];
            const double grip_vel = follower ? velocity[7] : 0;
            if (!std::isfinite(grip_pos) || !std::isfinite(grip_vel) ||
                (follower && (grip_pos < lower[7] || grip_pos > upper[7] ||
                              std::abs(grip_pos-q[7]) > .201))) {
                std::ostringstream error;
                error << "mirror: invalid gripper command on " << side_ << " target=" << grip_pos
                      << " measured=" << q[7] << " velocity=" << grip_vel << " max_lead=0.2";
                throw std::runtime_error(error.str());
            }
            const double gp = follower ? 6.0 : 0;
            const double gd = follower ? 0.45 : 0;
            const double gpd = gp * (grip_pos-q[7]) + gd * (grip_vel-measured[7].velocity);
            double gtau = friction_torque(measured[7].velocity, model_.gripper_friction_fc,
                                         model_.gripper_friction_fv, model_.gripper_friction_k);
            if (!std::isfinite(gtau)) throw std::runtime_error("mirror: invalid gripper friction torque");
            gtau = std::clamp(gtau + gpd, -1.0, 1.0) - gpd;
            grip_cmd = {gp, gd, grip_pos, grip_vel, gtau};
            frame[7] = {grip_pos, follower ? grip_vel : measured[7].velocity, gtau};
        } else {
            frame[7] = {0, 0, 0};
        }
        // All checks finish before sending anything on this arm.
        hardware_->send_arm(commands);
        if (gripper_on_) hardware_->send_gripper({grip_cmd});
    }
    void reload(const GravityTune& tune, const GravityModel& model) {
        tune_ = tune; model_ = model; prepare_friction();
        if (!gripper_configured_installed(model_)) gripper_on_ = false;
        apply_model(true);
    }
    void set_sim_master(const Pose& q, const Pose& dq = {}) {
        if (!sim_) throw std::runtime_error("mirror: simulation demo is simulation-only");
        sim_->set_home(std::vector<double>(q.begin(), q.begin()+7), q[7]);
        std::vector<openarm::damiao_motor::MITParam> commands;
        for (size_t i = 0; i < 7; ++i) commands.push_back({1, 0, q[i], dq[i], 0});
        sim_->send_arm(commands);
    }
    Dynamics* dynamics() { return &dynamics_; }
    Pose lower{}, upper{};
    unsigned speed_guard_mask = 0;
    std::vector<JointState> measured, frame;
private:
    void wait_for_feedback() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        std::string last_error;
        do {
            can_->refresh_all();
            // recv_all stops draining when the queue is momentarily empty. Allow
            // separately arriving replies from all eight motors before checking.
            for (int i = 0; i < 5; ++i) {
                can_->recv_all(2000);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            try {
                validate_feedback(can_->get_arm().get_motors());
                if (gripper_on_) validate_feedback(can_->get_gripper().get_motors());
                std::cout << "[mirror] " << side_ << " can=" << can_name_ << " "
                          << (gripper_on_ ? "all 8 motors have fresh feedback"
                                          : "7 arm motors have fresh feedback, gripper off")
                          << std::endl;
                return;
            } catch (const std::exception& error) {
                last_error = error.what();
            }
        } while (std::chrono::steady_clock::now() < deadline);
        try {
            validate_feedback(can_->get_arm().get_motors());
        } catch (const std::exception& error) {
            throw std::runtime_error("mirror: startup feedback wait failed (1s): " + error.what());
        }
        if (!gripper_on_) {
            return;
        }
        const auto grippers = can_->get_gripper().get_motors();
        const bool missing = grippers.empty() ||
                             grippers.front().last_state_received().time_since_epoch().count() == 0;
        if (missing) {
            gripper_on_ = false;
            apply_model(false);
            std::cout << "[mirror] " << side_ << " can=" << can_name_
                      << " 夹爪电机无反馈（" << last_error
                      << "），按无夹爪继续：不下发 0x08，重力不含指尖和夹爪电机质量" << std::endl;
            return;
        }
        throw std::runtime_error("mirror: startup feedback wait failed (1s): " + last_error);
    }
    void prepare_friction() {
        // Match openarm_zero_g's fallback for absent friction and zero wrist entries.
        double sum = 0;
        for (double v : tune_.fc) sum += std::abs(v);
        if (sum < 1e-6) {
            seed_default_friction(&tune_);
            tune_.fc[2] = 0.22; tune_.fv[2] = 0.016;
            tune_.fc[3] = 0.32; tune_.fv[3] = 0.022;
        }
        for (size_t i : {4u, 5u, 6u}) {
            if (std::abs(tune_.fc[i]) < 1e-9) tune_.fc[i] = 0.06;
            if (std::abs(tune_.fv[i]) < 1e-9) tune_.fv[i] = 0.004;
            if (tune_.fk[i] < 1) tune_.fk[i] = 25;
        }
    }
    template <class Motors> void validate_feedback(const Motors& motors) {
        for (const auto& m : motors) {
            const bool missing = m.last_state_received().time_since_epoch().count() == 0;
            const double age_ms = missing ? -1.0 : std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - m.last_state_received()).count();
            if (missing || age_ms > 150 || m.get_error_code() > 1) {
                std::ostringstream message;
                message << "mirror: " << (missing ? "no CAN feedback" :
                    m.get_error_code() > 1 ? "motor fault" : "stale CAN feedback")
                        << " side=" << side_ << " can=" << can_name_
                        << " joint=" << m.get_send_can_id()
                        << " rx_id=0x" << std::hex << m.get_recv_can_id() << std::dec
                        << " age_ms=" << age_ms << " status_code=" << m.get_error_code();
                throw std::runtime_error(message.str());
            }
        }
    }
    void apply_model(bool reinit) {
        const auto& scales = link_mass_scales_for(model_, side_);
        dynamics_.SetLinkMassScales(scales);
        dynamics_.SetLinkComDeltas(link_com_deltas_for(model_, side_));
        if (reinit && !dynamics_.ReinitWithMassScales(scales)) throw std::runtime_error("mirror: model reload failed");
        dynamics_.SetUrdfGravityScale(model_.urdf_scale);
        dynamics_.SetEndEffectorPointMass(end_effector_point_mass_kg(model_, gripper_on_), 0, 0,
                                          kOpenArmFingerComZ);
        const auto& off = side_ == "left" ? model_.wrist_camera_offset_left : model_.wrist_camera_offset_right;
        dynamics_.SetWristCameraMass(model_.wrist_camera_kg, side_ == "left", off[0], off[1], off[2]);
        dynamics_.SetDistalMassGains(model_.distal_ee_gain, model_.distal_cam_gain);
    }
    std::string side_;
    std::string can_name_;
    Dynamics dynamics_;
    GravityTune tune_;
    GravityModel model_;
    bool follower_feedback_;
    bool gripper_on_;
    std::unique_ptr<openarm::can::socket::OpenArm> can_;
    std::shared_ptr<MujocoTeleopHardware> sim_;
    std::shared_ptr<TeleopHardware> hardware_;
};
