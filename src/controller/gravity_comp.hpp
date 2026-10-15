// Shared gravity + friction feedforward.
// Aligned with OpenArmX gravity_comp: τ_ff = g_scale * G(q), then clamp.
// Ref: https://github.com/hzhxxxxx/Xc-openarm/.../openarmx_gravity_comp/README_CN.md
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

struct GravityTune {
    std::array<double, 7> scale{1, 1, 1, 1, 1, 1, 1};
    std::array<double, 7> bias{};
    // Coulomb-like + viscous (Robotics Center / CRISP tanh model).
    std::array<double, 7> fc{};
    std::array<double, 7> fv{};
    std::array<double, 7> fk{20, 20, 20, 20, 25, 25, 25};
};

struct GravityModel {
    // Physical installation, fixed for the lifetime of a hardware session.
    bool left_gripper_present = true;
    bool right_gripper_present = true;
    double urdf_scale = 1.0;
    double ee_payload_kg = 0.0;
    // Shared gripper-motor mass (joint 8); added with the fingers as a point mass.
    double gripper_motor_kg = 0.12;
    // Depth camera + bracket on the wrist (link7 frame). Off-axis COM torques j5/j6/j7.
    double wrist_camera_kg = 0.12;
    bool left_wrist_camera_present = true;
    bool right_wrist_camera_present = true;
    // Optional per-arm camera COM in link7 [m]. NaN → Dynamics default (±Y, +Z).
    std::array<double, 3> wrist_camera_offset_left{std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 3> wrist_camera_offset_right{std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN()};
    // Zero-g only: light Kd on j5–j7 damps residual drift from off-axis camera COM.
    double wrist_zero_g_kd = 0.22;
    // Gripper (joint 8, DM4310) Coulomb/viscous FF for zero-g free open/close.
    double gripper_friction_fc = 0.08;
    double gripper_friction_fv = 0.005;
    double gripper_friction_k = 25.0;
    // Per-joint distal point-mass gains (past link7 EE + wrist camera).
    // Defaults match hand-tuned zero-g feel.
    std::array<double, 7> distal_ee_gain{0.24, 0.255, 0.45, 0.15, 0.60, 1.0, 1.1};
    std::array<double, 7> distal_cam_gain{0.30, 0.32, 0.50, 0.30, 1.20, 0.75, 0.45};
    // OpenArmX calls this g_scale.
    // Zero-g and MoveIt real_arm share this calibrated gravity gain.
    double ff_gain = 0.90;
    // Quest position tracking. Ignored by zero-g and MoveIt real_arm.
    double tracking_ff_gain = 1.00;
    // Shared link1..link7 mass scales (both arms). left/right keys override if set alone.
    std::array<double, 7> link_mass_scale{1, 1, 1, 1, 1, 1, 1};
    std::array<double, 7> left_link_mass_scale{1, 1, 1, 1, 1, 1, 1};
    std::array<double, 7> right_link_mass_scale{1, 1, 1, 1, 1, 1, 1};
    bool has_shared_link_scale = false;
    // link1..link7 CoM deltas in link frame [m], packed xyz×7 (from inertial ID).
    std::array<double, 21> left_link_com_delta{};
    std::array<double, 21> right_link_com_delta{};
};

inline bool gripper_present_for(const GravityModel& model, const std::string& side) {
    return side == "left" ? model.left_gripper_present : model.right_gripper_present;
}

inline double wrist_camera_mass_for(const GravityModel& model, const std::string& side) {
    const bool present = side == "left" ? model.left_wrist_camera_present : model.right_wrist_camera_present;
    return present ? std::max(0.0, model.wrist_camera_kg) : 0.0;
}

inline double end_effector_mass_for(const GravityModel& model, const std::string& side) {
    // The removable assembly includes its motor, fingers, and grasped payload.
    // Camera/bracket compensation is independent and remains on the wrist.
    if (!gripper_present_for(model, side)) return 0.0;
    return 2.0 * 0.03602545343277134 + std::max(0.0, model.gripper_motor_kg) +
           std::max(0.0, model.ee_payload_kg);
}

inline const std::array<double, 7>& link_mass_scales_for(const GravityModel& model,
                                                         const std::string& side) {
    if (model.has_shared_link_scale) {
        return model.link_mass_scale;
    }
    return (side == "left") ? model.left_link_mass_scale : model.right_link_mass_scale;
}

inline const std::array<double, 21>& link_com_deltas_for(const GravityModel& model,
                                                         const std::string& side) {
    return (side == "left") ? model.left_link_com_delta : model.right_link_com_delta;
}

// OpenArmX TAU_LIMITS (Nm): proximal RS04-class, mid RS03, wrist RS00.
inline constexpr std::array<double, 7> kTauFfLimits{20.0, 20.0, 7.0, 7.0, 2.0, 2.0, 2.0};

inline double friction_torque(double dq, double fc, double fv, double fk) {
    const double k = std::max(1.0, fk);
    return fc * std::tanh(k * dq) + fv * dq;
}

inline double compose_gravity_torque(double g, const GravityTune& tune, size_t joint,
                                     double ff_gain) {
    return ff_gain * (tune.scale[joint] * g + tune.bias[joint]);
}

inline double compose_tau_ff(double g, const GravityTune& tune, size_t joint, double dq,
                             double ff_gain = 1.0) {
    const double grav = compose_gravity_torque(g, tune, joint, ff_gain);
    double tau = grav + friction_torque(dq, tune.fc[joint], tune.fv[joint], tune.fk[joint]);
    const double lim = (joint < kTauFfLimits.size()) ? kTauFfLimits[joint] : 20.0;
    return std::clamp(tau, -lim, lim);
}

inline bool parse_array7_text(const std::string& raw, std::array<double, 7>* out) {
    std::string body = raw;
    const auto lb = body.find('[');
    const auto rb = body.find(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
        return false;
    }
    body = body.substr(lb + 1, rb - lb - 1);
    std::stringstream ss(body);
    std::string token;
    size_t index = 0;
    while (std::getline(ss, token, ',') && index < out->size()) {
        (*out)[index++] = std::stod(token);
    }
    return index == out->size();
}

inline bool parse_array3_text(const std::string& raw, std::array<double, 3>* out) {
    std::string body = raw;
    const auto lb = body.find('[');
    const auto rb = body.find(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
        return false;
    }
    body = body.substr(lb + 1, rb - lb - 1);
    std::stringstream ss(body);
    std::string token;
    size_t index = 0;
    while (std::getline(ss, token, ',') && index < out->size()) {
        (*out)[index++] = std::stod(token);
    }
    return index == out->size();
}

inline bool parse_array21_text(const std::string& raw, std::array<double, 21>* out) {
    std::string body = raw;
    const auto lb = body.find('[');
    const auto rb = body.find(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
        return false;
    }
    body = body.substr(lb + 1, rb - lb - 1);
    std::stringstream ss(body);
    std::string token;
    size_t index = 0;
    while (std::getline(ss, token, ',') && index < out->size()) {
        (*out)[index++] = std::stod(token);
    }
    return index == out->size();
}

inline std::string trim_ws(std::string s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.erase(s.begin());
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.pop_back();
    }
    return s;
}

inline bool load_gravity_yaml(const std::string& path, GravityTune* left, GravityTune* right,
                              GravityModel* model) {
    std::ifstream file(path);
    if (!file) {
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) {
            line = line.substr(0, hash);
        }
        line = trim_ws(line);
        auto take = [&](const char* key, std::array<double, 7>* dest) {
            const std::string prefix = std::string(key) + ":";
            if (line.rfind(prefix, 0) == 0) {
                parse_array7_text(line.substr(prefix.size()), dest);
            }
        };
        auto take_scalar = [&](const char* key, double* dest) {
            const std::string prefix = std::string(key) + ":";
            if (line.rfind(prefix, 0) == 0) {
                *dest = std::stod(trim_ws(line.substr(prefix.size())));
            }
        };
        take("left_gravity_scale", &left->scale);
        take("left_gravity_bias", &left->bias);
        take("left_friction_fc", &left->fc);
        take("left_friction_fv", &left->fv);
        take("left_friction_k", &left->fk);
        take("right_gravity_scale", &right->scale);
        take("right_gravity_bias", &right->bias);
        take("right_friction_fc", &right->fc);
        take("right_friction_fv", &right->fv);
        take("right_friction_k", &right->fk);
        if (model != nullptr) {
            auto take_bool = [&](const char* key, bool* dest) {
                const std::string prefix = std::string(key) + ":";
                if (line.rfind(prefix, 0) != 0) return;
                const std::string value = trim_ws(line.substr(prefix.size()));
                if (value == "true" || value == "True" || value == "1") *dest = true;
                else if (value == "false" || value == "False" || value == "0") *dest = false;
                else throw std::runtime_error("Invalid boolean for " + std::string(key));
            };
            take_bool("left_gripper_present", &model->left_gripper_present);
            take_bool("right_gripper_present", &model->right_gripper_present);
            take_scalar("urdf_gravity_scale", &model->urdf_scale);
            take_scalar("ee_payload_kg", &model->ee_payload_kg);
            take_scalar("gripper_motor_kg", &model->gripper_motor_kg);
            take_scalar("wrist_camera_kg", &model->wrist_camera_kg);
            take_bool("left_wrist_camera_present", &model->left_wrist_camera_present);
            take_bool("right_wrist_camera_present", &model->right_wrist_camera_present);
            take_scalar("wrist_zero_g_kd", &model->wrist_zero_g_kd);
            take_scalar("gripper_friction_fc", &model->gripper_friction_fc);
            take_scalar("gripper_friction_fv", &model->gripper_friction_fv);
            take_scalar("gripper_friction_k", &model->gripper_friction_k);
            take("distal_ee_gain", &model->distal_ee_gain);
            take("distal_cam_gain", &model->distal_cam_gain);
            if (line.rfind("wrist_camera_offset_left:", 0) == 0) {
                parse_array3_text(line.substr(std::string("wrist_camera_offset_left:").size()),
                                  &model->wrist_camera_offset_left);
            }
            if (line.rfind("wrist_camera_offset_right:", 0) == 0) {
                parse_array3_text(line.substr(std::string("wrist_camera_offset_right:").size()),
                                  &model->wrist_camera_offset_right);
            }
            // OpenArmX name is g_scale; we also accept ff_gain.
            take_scalar("ff_gain", &model->ff_gain);
            take_scalar("g_scale", &model->ff_gain);
            take_scalar("tracking_g_scale", &model->tracking_ff_gain);
            take_scalar("moveit_g_scale", &model->tracking_ff_gain);
            take("link_mass_scale", &model->link_mass_scale);
            if (line.rfind("link_mass_scale:", 0) == 0 &&
                line.rfind("left_link_mass_scale:", 0) != 0 &&
                line.rfind("right_link_mass_scale:", 0) != 0) {
                model->has_shared_link_scale = true;
            }
            take("left_link_mass_scale", &model->left_link_mass_scale);
            take("right_link_mass_scale", &model->right_link_mass_scale);
            if (line.rfind("left_link_com_delta:", 0) == 0) {
                parse_array21_text(line.substr(std::string("left_link_com_delta:").size()),
                                   &model->left_link_com_delta);
            }
            if (line.rfind("right_link_com_delta:", 0) == 0) {
                parse_array21_text(line.substr(std::string("right_link_com_delta:").size()),
                                   &model->right_link_com_delta);
            }
        }
    }
    return true;
}

// Conservative OpenArm defaults (DM8009 / DM4340 / DM4310). Keep modest —
// large Fc/Fv at near-zero velocity still couples into slow drift.
inline void seed_default_friction(GravityTune* tune) {
    if (tune == nullptr) {
        return;
    }
    tune->fc = {0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06};
    tune->fv = {0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005};
    tune->fk = {20, 20, 20, 20, 25, 25, 25};
}
