#pragma once

#include <common.hpp>
#include <controller/control.hpp>
#include <yamlloader.hpp>

// Shared real/simulation follower policy for Quest teleop and teaching replay.
struct FollowerParams {
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> Fc;
    std::vector<double> k;
    std::vector<double> Fv;
    std::vector<double> Fo;
};

inline FollowerParams load_follower_params() {
    YamlLoader follower_loader("config/follower.yaml");
    FollowerParams params;
    params.kp = follower_loader.get_vector("FollowerArmParam", "Kp");
    params.kd = follower_loader.get_vector("FollowerArmParam", "Kd");
    params.Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
    params.k = follower_loader.get_vector("FollowerArmParam", "k");
    params.Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
    params.Fo = follower_loader.get_vector("FollowerArmParam", "Fo");
    return params;
}

inline void configure_follower_control(Control* control, const FollowerParams& params, size_t total_joints,
                                bool sim, const std::string& arm_side = "") {
    std::vector<double> kp = params.kp;
    std::vector<double> kd = params.kd;
    apply_quest_vr_follower_mit_gains(&kp, &kd, sim, "quest-cartesian");
    // Optional per-arm gripper MIT override (hold torque ≈ Kp · grasp_lag).
    if (!sim && !kp.empty()) {
        const size_t g = kp.size() - 1;
        const char* side_kp_env =
            (arm_side == "right_arm")
                ? "OPENARM_GRIPPER_KP_RIGHT"
                : ((arm_side == "left_arm") ? "OPENARM_GRIPPER_KP_LEFT" : nullptr);
        if (side_kp_env != nullptr) {
            kp[g] = env_double_or_common(side_kp_env, kp[g]);
        }
        const char* side_kd_env =
            (arm_side == "right_arm")
                ? "OPENARM_GRIPPER_KD_RIGHT"
                : ((arm_side == "left_arm") ? "OPENARM_GRIPPER_KD_LEFT" : nullptr);
        if (side_kd_env != nullptr && g < kd.size()) {
            kd[g] = env_double_or_common(side_kd_env, kd[g]);
        }
        if (arm_side == "right_arm" || arm_side == "left_arm") {
            std::cout << "[quest-cartesian] " << arm_side << " gripper MIT Kp=" << kp[g]
                      << " Kd=" << (g < kd.size() ? kd[g] : 0.0) << std::endl;
        }
    }
    control->SetParameter(kp, kd, params.Fc, params.k, params.Fv, params.Fo);
    control->SetSafetyFilter(std::make_unique<SafetyFilter>(
        sim ? make_sim_safety_config(total_joints) : make_default_safety_config(total_joints)));
}

inline void configure_follower_gravity_model(Dynamics* dynamics, const GravityModel& model,
                                             const std::string& side) {
    dynamics->SetLinkMassScales(link_mass_scales_for(model, side));
    dynamics->SetLinkComDeltas(link_com_deltas_for(model, side));
    dynamics->SetUrdfGravityScale(model.urdf_scale);
    dynamics->SetEndEffectorPointMass(end_effector_mass_for(model, side), 0, 0, 0.1475);
    const auto& offset = side == "left" ? model.wrist_camera_offset_left : model.wrist_camera_offset_right;
    dynamics->SetWristCameraMass(wrist_camera_mass_for(model, side), side == "left", offset[0], offset[1], offset[2]);
    dynamics->SetDistalMassGains(model.distal_ee_gain, model.distal_cam_gain);
}
