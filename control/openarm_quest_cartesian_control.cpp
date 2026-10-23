#include <controller/startup_trajectory.hpp>
// Copyright 2026 Enactic, Inc.
//
// Quest 3 Cartesian teleoperation for OpenArm follower.
// VR clutch pose mapping -> smoothed TCP target -> position IK -> joint reference
// -> Control::unilateral_step() -> SafetyFilter -> MIT -> backend.
// SafetyFilter clamps arm position to the URDF, holds the previous output on
// NaN/inf, and limits the first position step. velocity_limit / effort_limit
// stay 0, which means those clamps are not enabled. Real follower MIT adds
// calibrated, bounded gravity feedforward; the motor Kp spring is not limited here.

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <common.hpp>
#include <controller/control.hpp>
#include <controller/follower_config.hpp>
#include <controller/dynamics.hpp>
#include <controller/human_mimic_controller.hpp>
#include <controller/unitree_style_profile.hpp>
#include <controller/s_curve.hpp>
#include <controller/teaching_replay.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <hardware/teleop_hardware.hpp>
#include <iostream>
#include <io/quest_pose_receiver.hpp>
#include <io/teleop_io.hpp>
#include <io/teleop_trajectory_logger.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <openarm_port/can_feedback.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <csignal>
#include <set>
#include <spawn.h>
#include <sys/time.h>
#include <sys/wait.h>

extern char** environ;
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <yamlloader.hpp>
#include <motor_telemetry.hpp>

namespace {

constexpr double kQuestReceiveHz = 500.0;
constexpr double kQuestReferenceHz = FREQUENCY;  // 1000 Hz joint references; Quest packets drained at 500 Hz

std::string select_leaf_link(const std::string& urdf_path, const std::string& arm_side) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    // Prefer hand_tcp (tool frame). Do not prefer a bare "_hand" name that may not exist.
    const std::vector<std::string> candidates = {
        "openarm_" + side + "_hand_tcp",
        "openarm_" + side + "_link7",
        "openarm_" + side + "_link8",
    };
    for (const auto& candidate : candidates) {
        std::ifstream file(urdf_path);
        if (!file) continue;
        std::stringstream buffer;
        buffer << file.rdbuf();
        if (buffer.str().find("link name=\"" + candidate + "\"") != std::string::npos) {
            return candidate;
        }
    }
    return candidates.front();
}

struct Options {
    std::string arm_side = "right_arm";
    std::string follower_can = "can0";
    std::string follower_can_left = "can1";
    std::string follower_can_right = "can0";
    std::string urdf_path = "assets/v1.urdf";
    std::string mujoco_xml =
        "/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml";
    uint16_t quest_port = 47911;
    uint16_t quest_port_left = 47912;
    uint16_t quest_port_right = 47911;
    bool auto_home = false;
    bool sim = false;
    bool bimanual = false;
    bool sim_table_ready = true;  // teleport sim arms to TABLE_READY_POSITION at start
    // Quest modes: unitree_style (default), pika_style.
    std::string teleop_mode = "unitree_style";
    // Quest orientation input mapping: "absolute" (default; fix hand<->TCP
    // calibration once) or "incremental" (re-anchor orientation on every grip).
    std::string orientation_mapping = "absolute";
    // Optional smooth startup motion before teleop (both backends). Default target
    // is TABLE_READY; override with a pose/trajectory file.
    // Dual-arm merged CSV (time,q0,dq0,tau0,...,q15,...) is also supported: current
    // pose -> first frame via S-curve, play trajectory, then hold for VR clutch.
    bool startup_move = false;
    std::string startup_file;     // optional pose/trajectory file
    double startup_move_sec = 4.0;  // duration when file has no explicit times
    // Quintic current->first duration for dual CSV; <=0 means adaptive (CsvReplay).
    double startup_transition_sec = 5.0;
    // Desktop Tk panels (录制 / TCP 微调) talking to these UDP ports.
    bool quest_ui = true;
    uint16_t quest_ui_record_port = 47930;
    uint16_t quest_ui_jog_port = 47931;
    std::string quest_record_dir;  // empty => recordings/quest_sessions/<stamp>
    double quest_jog_step_m = 0.01;      // legacy one-shot nudge (m)
    double quest_jog_speed_mps = 0.02;   // hold-to-jog tip speed (m/s), default 20 mm/s
    double quest_jog_omega_rps = 0.25;   // hold-to-jog TCP rotation (rad/s), ~14 deg/s
};

std::vector<double> table_ready_arm_pose() {
    return {TABLE_READY_POSITION[0], TABLE_READY_POSITION[1], TABLE_READY_POSITION[2],
            TABLE_READY_POSITION[3], TABLE_READY_POSITION[4], TABLE_READY_POSITION[5],
            TABLE_READY_POSITION[6]};
}

bool env_bool_or(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    return !(v[0] == '0' || v[0] == 'f' || v[0] == 'F' || v[0] == 'n' || v[0] == 'N');
}

double env_double_or(const char* name, double fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    try {
        return std::stod(v);
    } catch (...) {
        return fallback;
    }
}

// Shared per-run HM log directory (left/right + status share one folder).
std::string hm_log_session_dir() {
    static std::string dir;
    static bool ready = false;
    if (ready) {
        return dir;
    }
    const char* env = std::getenv("OPENARM_HM_LOG_DIR");
    if (env != nullptr && env[0] != '\0') {
        dir = env;
    } else {
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&t, &tm);
        std::ostringstream oss;
        oss << "recordings/hm_logs/quest_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
        dir = oss.str();
        ::setenv("OPENARM_HM_LOG_DIR", dir.c_str(), 0);
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    ready = true;
    std::cout << "[quest-cartesian] HM logs -> " << std::filesystem::absolute(dir, ec).string()
              << std::endl;
    return dir;
}

// 1 Hz teleop status lines: file by default; console only if OPENARM_HM_STATUS_CONSOLE=1.
void emit_teleop_status(const std::string& line) {
    static std::ofstream status_file;
    static bool status_opened = false;
    if (!status_opened) {
        status_opened = true;
        const std::string path = hm_log_session_dir() + "/status.log";
        status_file.open(path, std::ios::out | std::ios::app);
        if (!status_file.is_open()) {
            std::cerr << "[quest-cartesian] failed to open status log: " << path << std::endl;
        }
    }
    if (status_file.is_open()) {
        status_file << line << std::endl;
    }
    if (env_bool_or("OPENARM_HM_STATUS_CONSOLE", false)) {
        std::cout << line << std::endl;
    }
}

MotorFaultMonitor& motor_fault_monitor() {
    static MotorFaultMonitor monitor;
    static bool inited = false;
    if (!inited) {
        inited = true;
        const std::string dir = hm_log_session_dir();
        monitor.set_log_dir(dir);
        // Optional: OPENARM_MOTOR_STATUS_HZ (default 10)
        const char* hz = std::getenv("OPENARM_MOTOR_STATUS_HZ");
        if (hz != nullptr && *hz != '\0') {
            try {
                monitor.set_status_hz(std::stod(hz));
            } catch (...) {
            }
        }
    }
    return monitor;
}

// Shared configuration for the Unitree and PIKA controllers.
HumanMimicConfig make_quest_base_config(const std::string& urdf_path,
                                         const std::string& arm_side,
                                         const std::string& orientation_mapping, bool sim) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    HumanMimicConfig cfg;
    cfg.orientation_mapping_mode = (orientation_mapping == "absolute")
                                       ? OrientationMappingMode::ABSOLUTE_CALIBRATED
                                       : OrientationMappingMode::INCREMENTAL_CLUTCH;
    cfg.urdf_path = urdf_path;
    cfg.root_link = "openarm_body_link0";
    cfg.shoulder_link = "openarm_" + side + "_link1";
    cfg.elbow_link = "openarm_" + side + "_link4";
    cfg.wrist_link = "openarm_" + side + "_link7";
    cfg.wrist_to_tcp = Eigen::Vector3d(0.0, 0.0, 0.186);
    cfg.ik_track_tcp = env_bool_or("OPENARM_HM_IK_TCP", true);
    // Baseline outward bias (pre-20260720). The flipped dirs reduced inward curl
    // for some postures but made XYZ elbow residual fight the arm and chatter q0.
    cfg.outward_dir =
        (side == "left") ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(0.0, -1.0, 0.0);
    cfg.q_nominal = table_ready_arm_pose();

    // Gripper open/closed/speed differ by BACKEND UNITS:
    //   - Sim  : MuJoCo finger_joint in METERS   (~0.00 open .. 0.04 closed).
    //   - Real : DM4310 motor position in RADIANS (1:1 passthrough, no transmission).
    // A sim-meter value like 0.04 is only ~2 deg on the real motor, so the real
    // gripper barely moves. Set real open/closed to the MEASURED motor angles.
    // Override any of these without recompiling via the env vars below.
    if (sim) {
        cfg.gripper_open = 0.0;
        cfg.gripper_closed = 0.04;
        cfg.gripper_speed = 0.05;
        cfg.gripper_close_speed = 0.02;  // slower close
    } else {
        // Real (DM4310, radians). Confirmed direction: POSITIVE = closing.
        // Power-on rest q0 ~= -0.18; user wants ~0.8 rad of travel PER DIRECTION, so
        // open = q0-0.8 and closed = q0+0.8 (total stroke ~1.6 rad). Override per-arm
        // via the env vars if either end pushes against a mechanical stop.
        cfg.gripper_open = -0.98;
        cfg.gripper_closed = 0.62;
        cfg.gripper_speed = 2.0;         // open rad/s
        cfg.gripper_close_speed = 0.45;   // close slower (~3.5 s full stroke)
    }
    cfg.gripper_open = env_double_or("OPENARM_GRIPPER_OPEN", cfg.gripper_open);
    cfg.gripper_closed = env_double_or("OPENARM_GRIPPER_CLOSED", cfg.gripper_closed);
    cfg.gripper_speed = env_double_or("OPENARM_GRIPPER_SPEED", cfg.gripper_speed);
    cfg.gripper_close_speed =
        env_double_or("OPENARM_GRIPPER_CLOSE_SPEED", cfg.gripper_close_speed);
    cfg.gripper_buttons_enabled = env_bool_or("OPENARM_GRIPPER_BUTTONS", true);
    cfg.gripper_soft_stall = env_bool_or("OPENARM_GRIPPER_SOFT_STALL", false);
    // Sim stroke is meters (~0–0.04); real is DM4310 radians (~1.6 rad). Defaults
    // scale with that so contact freeze triggers before MIT winds up hard.
    cfg.gripper_stall_error =
        env_double_or("OPENARM_GRIPPER_STALL_ERROR", sim ? 0.003 : 0.04);
    cfg.gripper_stall_velocity =
        env_double_or("OPENARM_GRIPPER_STALL_VEL", sim ? 0.005 : 0.05);
    cfg.gripper_stall_time = env_double_or("OPENARM_GRIPPER_STALL_TIME", 0.10);
    cfg.gripper_stall_progress =
        env_double_or("OPENARM_GRIPPER_STALL_PROGRESS", sim ? 0.0005 : 0.005);
    cfg.gripper_hold_preload =
        env_double_or("OPENARM_GRIPPER_HOLD_PRELOAD", cfg.gripper_hold_preload);
    cfg.gripper_grasp_hold = env_bool_or("OPENARM_GRIPPER_GRASP_HOLD", false);
    // Per-arm grasp-hold enable (e.g. left off / right on).
    if (!sim && side == "right") {
        cfg.gripper_grasp_hold =
            env_bool_or("OPENARM_GRIPPER_GRASP_HOLD_RIGHT", cfg.gripper_grasp_hold);
        cfg.gripper_soft_stall =
            env_bool_or("OPENARM_GRIPPER_SOFT_STALL_RIGHT", cfg.gripper_soft_stall);
    } else if (!sim && side == "left") {
        cfg.gripper_grasp_hold =
            env_bool_or("OPENARM_GRIPPER_GRASP_HOLD_LEFT", cfg.gripper_grasp_hold);
        cfg.gripper_soft_stall =
            env_bool_or("OPENARM_GRIPPER_SOFT_STALL_LEFT", cfg.gripper_soft_stall);
    }
    // Real: τ≈Kp·lag with Kp≈10 → lag 0.22 ≈ 2.2 Nm. Sim uses smaller meter lag.
    // Per-arm overrides (e.g. OPENARM_GRIPPER_GRASP_LAG_RIGHT) apply after the shared default.
    cfg.gripper_grasp_hold_lag =
        env_double_or("OPENARM_GRIPPER_GRASP_LAG", sim ? 0.006 : 0.22);
    cfg.gripper_grasp_hold_lag_max =
        env_double_or("OPENARM_GRIPPER_GRASP_LAG_MAX", sim ? 0.010 : 0.30);
    if (!sim && side == "right") {
        cfg.gripper_grasp_hold_lag =
            env_double_or("OPENARM_GRIPPER_GRASP_LAG_RIGHT", cfg.gripper_grasp_hold_lag);
        cfg.gripper_grasp_hold_lag_max =
            env_double_or("OPENARM_GRIPPER_GRASP_LAG_MAX_RIGHT", cfg.gripper_grasp_hold_lag_max);
    } else if (!sim && side == "left") {
        cfg.gripper_grasp_hold_lag =
            env_double_or("OPENARM_GRIPPER_GRASP_LAG_LEFT", cfg.gripper_grasp_hold_lag);
        cfg.gripper_grasp_hold_lag_max =
            env_double_or("OPENARM_GRIPPER_GRASP_LAG_MAX_LEFT", cfg.gripper_grasp_hold_lag_max);
    }

    // Reach envelope. The workspace cap is ASYMMETRIC: only BACKWARD stays tight;
    // forward and lateral are opened up.
    //   - backward only  : OPENARM_HM_MAX_OFFSET      (original 0.45 feel, unchanged)
    //   - forward+lateral: OPENARM_HM_MAX_OFFSET_FWD  (opened up so the arm can extend/reach out)
    // Keep reach_margin small so the arm can straighten forward; all env-tunable.
    // Soft-reach: progress-based ceiling (bent mid-travel → near-straight at workspace
    // bottom). Disable: OPENARM_HM_REACH_SOFT=0.
    cfg.max_teleop_offset_m = env_double_or("OPENARM_HM_MAX_OFFSET", 0.60);
    cfg.max_teleop_offset_forward_m = env_double_or("OPENARM_HM_MAX_OFFSET_FWD", 0.85);
    cfg.max_teleop_offset_up_m = env_double_or("OPENARM_HM_MAX_OFFSET_UP", 0.75);
    cfg.max_teleop_offset_down_m = env_double_or("OPENARM_HM_MAX_OFFSET_DOWN", 0.80);
    cfg.reach_margin_m = env_double_or("OPENARM_HM_REACH_MARGIN", 0.002);
    cfg.reach_soft_enabled = env_bool_or("OPENARM_HM_REACH_SOFT", true);
    // Down-progress gate (scheme A). Disable alone: OPENARM_HM_REACH_SOFT_PROGRESS=0
    // (keeps soft ceiling with fixed REACH_SOFT_MARGIN). Disable all soft-reach:
    // OPENARM_HM_REACH_SOFT=0.
    cfg.reach_soft_progress_enabled =
        env_bool_or("OPENARM_HM_REACH_SOFT_PROGRESS", true);
    cfg.reach_soft_forward_progress_enabled =
        env_bool_or("OPENARM_HM_REACH_SOFT_FWD", true);
    cfg.reach_soft_forward_span_m =
        env_double_or("OPENARM_HM_REACH_SOFT_FWD_SPAN", cfg.reach_soft_forward_span_m);
    // At workspace bottom (progress=1): ~2mm short of L1+L2 ≈ near-straight.
    cfg.reach_soft_margin_m = env_double_or("OPENARM_HM_REACH_SOFT_MARGIN", 0.002);
    // At clutch height (progress=0): keep a clear bend for fine motion.
    cfg.reach_soft_bent_margin_m =
        env_double_or("OPENARM_HM_REACH_SOFT_BENT_MARGIN", 0.045);
    cfg.reach_soft_zone_m = env_double_or("OPENARM_HM_REACH_SOFT_ZONE", 0.12);
    cfg.reach_intent_elbow_yield_enabled =
        env_bool_or("OPENARM_HM_REACH_ELBOW_YIELD", true);
    cfg.reach_intent_elbow_scale_min =
        env_double_or("OPENARM_HM_REACH_ELBOW_MIN", cfg.reach_intent_elbow_scale_min);
    // Forward = robot base +X by default; flip with OPENARM_HM_FORWARD_SIGN=-1 if the
    // "front" opened the wrong way for your base mounting.
    cfg.forward_axis =
        Eigen::Vector3d(env_double_or("OPENARM_HM_FORWARD_SIGN", 1.0), 0.0, 0.0);

    // Whole-arm IK task weights (env-tunable for on-device balancing of
    // reach vs posture vs orientation).
    cfg.w_wrist_pos = env_double_or("OPENARM_HM_W_WRIST", cfg.w_wrist_pos);
    cfg.w_hand_ori = env_double_or("OPENARM_HM_W_ORI", cfg.w_hand_ori);
    // OPENARM_HM_W_ELBOW: PHI = swivel residual weight; XYZ = elbow position weight.
    cfg.w_elbow = env_double_or("OPENARM_HM_W_ELBOW", cfg.w_elbow);
    cfg.w_continuity = env_double_or("OPENARM_HM_W_CONT", cfg.w_continuity);
    {
        const char* et = std::getenv("OPENARM_HM_ELBOW_TASK");
        if (et != nullptr) {
            const std::string s(et);
            if (s == "phi" || s == "PHI" || s == "sew" || s == "SEW") {
                cfg.elbow_task_mode = ElbowTaskMode::PHI;
            } else if (s == "xyz" || s == "XYZ" || s == "pos" || s == "POS") {
                cfg.elbow_task_mode = ElbowTaskMode::XYZ;
            }
        }
    }
    cfg.elbow_nullspace_enabled =
        env_bool_or("OPENARM_HM_ELBOW_NULLSPACE", cfg.elbow_nullspace_enabled);
    cfg.elbow_nullspace_gain =
        env_double_or("OPENARM_HM_ELBOW_NS_GAIN", cfg.elbow_nullspace_gain);
    cfg.elbow_nullspace_sigma_min =
        env_double_or("OPENARM_HM_ELBOW_NS_SIGMA", cfg.elbow_nullspace_sigma_min);
    cfg.elbow_nullspace_dq_max =
        env_double_or("OPENARM_HM_ELBOW_NS_DQMAX", cfg.elbow_nullspace_dq_max);
    cfg.elbow_nullspace_cont_scale =
        env_double_or("OPENARM_HM_ELBOW_NS_CONT", cfg.elbow_nullspace_cont_scale);
    cfg.wrist_lag_assist_enabled =
        env_bool_or("OPENARM_HM_WRIST_LAG_ASSIST", cfg.wrist_lag_assist_enabled);
    cfg.wrist_lag_assist_err_m =
        env_double_or("OPENARM_HM_WRIST_LAG_ERR", cfg.wrist_lag_assist_err_m);
    cfg.wrist_lag_assist_cont_min =
        env_double_or("OPENARM_HM_WRIST_LAG_CONT", cfg.wrist_lag_assist_cont_min);
    cfg.wrist_limit_ori_soften_enabled =
        env_bool_or("OPENARM_HM_WRIST_ORI_SOFTEN", cfg.wrist_limit_ori_soften_enabled);
    cfg.wrist_limit_ori_margin_rad =
        env_double_or("OPENARM_HM_WRIST_ORI_MARGIN", cfg.wrist_limit_ori_margin_rad);
    cfg.wrist_limit_ori_scale_min =
        env_double_or("OPENARM_HM_WRIST_ORI_MIN", cfg.wrist_limit_ori_scale_min);
    cfg.wrist_escape_phi_enabled =
        env_bool_or("OPENARM_HM_WRIST_ESCAPE_PHI", cfg.wrist_escape_phi_enabled);
    cfg.wrist_escape_ori_err_rad =
        env_double_or("OPENARM_HM_WRIST_ESCAPE_ORI", cfg.wrist_escape_ori_err_rad);
    cfg.wrist_escape_cont_min =
        env_double_or("OPENARM_HM_WRIST_ESCAPE_CONT", cfg.wrist_escape_cont_min);
    cfg.wrist_sat_anti_hop_enabled =
        env_bool_or("OPENARM_HM_WRIST_ANTI_HOP", cfg.wrist_sat_anti_hop_enabled);
    cfg.wrist_sat_anti_hop_margin_rad =
        env_double_or("OPENARM_HM_WRIST_ANTI_HOP_MARGIN", cfg.wrist_sat_anti_hop_margin_rad);
    cfg.wrist_sat_ori_scale =
        env_double_or("OPENARM_HM_WRIST_SAT_ORI", cfg.wrist_sat_ori_scale);
    cfg.wrist_sat_cont_boost =
        env_double_or("OPENARM_HM_WRIST_SAT_CONT", cfg.wrist_sat_cont_boost);
    cfg.wrist_sat_distal_slew_radps =
        env_double_or("OPENARM_HM_WRIST_SAT_SLEW", cfg.wrist_sat_distal_slew_radps);
    cfg.wrist_sat_distal_max_step =
        env_double_or("OPENARM_HM_WRIST_SAT_STEP", cfg.wrist_sat_distal_max_step);
    cfg.j5_twist_priority_enabled =
        env_bool_or("OPENARM_HM_J5_TWIST_PRIORITY", cfg.j5_twist_priority_enabled);
    cfg.j5_twist_weight =
        env_double_or("OPENARM_HM_J5_TWIST_WEIGHT", cfg.j5_twist_weight);
    cfg.j5_other_twist_suppress_weight = env_double_or(
        "OPENARM_HM_J5_OTHER_TWIST_WEIGHT", cfg.j5_other_twist_suppress_weight);
    cfg.j5_soft_limit_margin_rad =
        env_double_or("OPENARM_HM_J5_SOFT_MARGIN", cfg.j5_soft_limit_margin_rad);
    cfg.j5_soft_limit_hysteresis_rad =
        env_double_or("OPENARM_HM_J5_SOFT_HYST", cfg.j5_soft_limit_hysteresis_rad);
    cfg.j5_continuity_scale =
        env_double_or("OPENARM_HM_J5_CONT_SCALE", cfg.j5_continuity_scale);
    cfg.j6_continuity_scale =
        env_double_or("OPENARM_HM_J6_CONT_SCALE", cfg.j6_continuity_scale);
    cfg.j7_continuity_scale =
        env_double_or("OPENARM_HM_J7_CONT_SCALE", cfg.j7_continuity_scale);
    cfg.proximal_continuity_scale =
        env_double_or("OPENARM_HM_PROX_CONT_SCALE", cfg.proximal_continuity_scale);
    cfg.trigger_twist_mode =
        env_bool_or("OPENARM_HM_TRIGGER_TWIST", cfg.trigger_twist_mode);
    cfg.trigger_twist_ori_boost =
        env_double_or("OPENARM_HM_TRIGGER_ORI_BOOST", cfg.trigger_twist_ori_boost);
    cfg.trigger_twist_elbow_scale =
        env_double_or("OPENARM_HM_TRIGGER_ELBOW_SCALE", cfg.trigger_twist_elbow_scale);
    cfg.trigger_twist_j5_slew_radps =
        env_double_or("OPENARM_HM_TRIGGER_J5_SLEW", cfg.trigger_twist_j5_slew_radps);
    cfg.trigger_twist_j6_slew_radps =
        env_double_or("OPENARM_HM_TRIGGER_J6_SLEW", cfg.trigger_twist_j6_slew_radps);
    cfg.trigger_twist_j7_slew_radps =
        env_double_or("OPENARM_HM_TRIGGER_J7_SLEW", cfg.trigger_twist_j7_slew_radps);
    cfg.wrist_branch_guard_enabled =
        env_bool_or("OPENARM_HM_WRIST_BRANCH_GUARD", cfg.wrist_branch_guard_enabled);
    cfg.wrist_branch_guard_target_ori_rad = env_double_or(
        "OPENARM_HM_WRIST_BRANCH_ORI", cfg.wrist_branch_guard_target_ori_rad);
    cfg.wrist_branch_guard_target_pos_m = env_double_or(
        "OPENARM_HM_WRIST_BRANCH_POS", cfg.wrist_branch_guard_target_pos_m);
    cfg.wrist_branch_guard_candidate_rad = env_double_or(
        "OPENARM_HM_WRIST_BRANCH_DQ", cfg.wrist_branch_guard_candidate_rad);
    cfg.wrist_branch_guard_improve_ratio = env_double_or(
        "OPENARM_HM_WRIST_BRANCH_IMPROVE", cfg.wrist_branch_guard_improve_ratio);
    cfg.ik_branch_guard_proximal_rad = env_double_or(
        "OPENARM_HM_BRANCH_PROX_DQ", cfg.ik_branch_guard_proximal_rad);
    cfg.wrist_slew_j5_radps =
        env_double_or("OPENARM_HM_WRIST_SLEW_J5", cfg.wrist_slew_j5_radps);
    cfg.wrist_slew_j6_radps =
        env_double_or("OPENARM_HM_WRIST_SLEW_J6", cfg.wrist_slew_j6_radps);
    cfg.wrist_slew_j7_radps =
        env_double_or("OPENARM_HM_WRIST_SLEW_J7", cfg.wrist_slew_j7_radps);
    cfg.wrist_escape_hold_sec =
        env_double_or("OPENARM_HM_WRIST_ESCAPE_HOLD", cfg.wrist_escape_hold_sec);
    cfg.wrist_escape_release_sec =
        env_double_or("OPENARM_HM_WRIST_ESCAPE_RELEASE", cfg.wrist_escape_release_sec);
    cfg.elbow_target_blend_enabled =
        env_bool_or("OPENARM_HM_ELBOW_BLEND", cfg.elbow_target_blend_enabled);
    cfg.elbow_target_blend_start_m =
        env_double_or("OPENARM_HM_ELBOW_BLEND_START", cfg.elbow_target_blend_start_m);
    cfg.elbow_target_blend_full_m =
        env_double_or("OPENARM_HM_ELBOW_BLEND_FULL", cfg.elbow_target_blend_full_m);
    cfg.elbow_target_blend_phi_start_rad =
        env_double_or("OPENARM_HM_ELBOW_BLEND_PHI0", cfg.elbow_target_blend_phi_start_rad);
    cfg.elbow_target_blend_phi_full_rad =
        env_double_or("OPENARM_HM_ELBOW_BLEND_PHI1", cfg.elbow_target_blend_phi_full_rad);
    cfg.ik_proximal_max_step =
        env_double_or("OPENARM_HM_IK_PROX_STEP", cfg.ik_proximal_max_step);
    cfg.ik_distal_max_step = env_double_or("OPENARM_HM_IK_DIST_STEP", cfg.ik_distal_max_step);
    {
        const char* bk = std::getenv("OPENARM_HM_IK_BACKEND");
        if (bk != nullptr) {
            const std::string s(bk);
            if (s == "lm" || s == "LM") {
                cfg.ik_backend = HumanMimicIkBackend::LM;
            } else if (s == "diff_qp" || s == "DIFF_QP" || s == "qp" || s == "QP") {
                cfg.ik_backend = HumanMimicIkBackend::DIFF_QP;
            }
        }
    }
    cfg.ik_kp_pos = env_double_or("OPENARM_HM_IK_KP_POS", cfg.ik_kp_pos);
    cfg.ik_kp_ori = env_double_or("OPENARM_HM_IK_KP_ORI", cfg.ik_kp_ori);
    cfg.ik_kp_phi = env_double_or("OPENARM_HM_IK_KP_PHI", cfg.ik_kp_phi);
    cfg.ik_joint_limit_margin_frac =
        env_double_or("OPENARM_HM_IK_LIMIT_MARGIN", cfg.ik_joint_limit_margin_frac);

    // Reach-forward assist (elbow/orientation yield near full extension).
    cfg.extension_yield_start =
        env_double_or("OPENARM_HM_EXT_START", cfg.extension_yield_start);
    cfg.extension_elbow_scale_min =
        env_double_or("OPENARM_HM_EXT_ELBOW_MIN", cfg.extension_elbow_scale_min);
    cfg.extension_ori_scale_min =
        env_double_or("OPENARM_HM_EXT_ORI_MIN", cfg.extension_ori_scale_min);
    cfg.extension_cont_scale_min =
        env_double_or("OPENARM_HM_EXT_CONT_MIN", cfg.extension_cont_scale_min);
    cfg.extension_nom_scale_min =
        env_double_or("OPENARM_HM_EXT_NOM_MIN", cfg.extension_nom_scale_min);
    cfg.extension_phi_freeze_stretch =
        env_double_or("OPENARM_HM_EXT_PHI_FREEZE", cfg.extension_phi_freeze_stretch);
    cfg.extension_roll_slew_scale =
        env_double_or("OPENARM_HM_EXT_ROLL_SLEW", cfg.extension_roll_slew_scale);
    cfg.q_target_slew_enabled = env_bool_or("OPENARM_HM_Q_SLEW", cfg.q_target_slew_enabled);
    cfg.q_target_slew_max_radps =
        env_double_or("OPENARM_HM_Q_SLEW_MAX", cfg.q_target_slew_max_radps);
    cfg.k_joint = env_double_or("OPENARM_HM_K_JOINT", cfg.k_joint);
    cfg.qdot_max = env_double_or("OPENARM_HM_QDOT_MAX", cfg.qdot_max);
    cfg.joint_output_filter_enabled =
        env_bool_or("OPENARM_HM_Q_FILT", cfg.joint_output_filter_enabled);
    cfg.joint_output_lowpass_hz =
        env_double_or("OPENARM_HM_Q_FILT_HZ", cfg.joint_output_lowpass_hz);
    cfg.qddot_max = env_double_or("OPENARM_HM_QDDOT_MAX", cfg.qddot_max);
    cfg.elbow_cmd_slew_enabled =
        env_bool_or("OPENARM_HM_ELBOW_PHI_SLEW", cfg.elbow_cmd_slew_enabled);
    cfg.elbow_cmd_slew_radps =
        env_double_or("OPENARM_HM_ELBOW_PHI_SLEW_MAX", cfg.elbow_cmd_slew_radps);
    cfg.elbow_cmd_max_step_rad =
        env_double_or("OPENARM_HM_ELBOW_PHI_STEP", cfg.elbow_cmd_max_step_rad);
    cfg.elbow_joint_slew_radps =
        env_double_or("OPENARM_HM_ELBOW_Q_SLEW", cfg.elbow_joint_slew_radps);
    cfg.elbow_joint_slew_down_scale = std::clamp(
        env_double_or("OPENARM_HM_ELBOW_Q_SLEW_DOWN", cfg.elbow_joint_slew_down_scale), 0.05,
        1.0);
    cfg.elbow_raise_hold_enabled =
        env_bool_or("OPENARM_HM_ELBOW_RAISE_HOLD", cfg.elbow_raise_hold_enabled);
    cfg.elbow_raise_hold_d_m =
        env_double_or("OPENARM_HM_ELBOW_RAISE_D", cfg.elbow_raise_hold_d_m);
    cfg.elbow_raise_hold_q3_rad =
        env_double_or("OPENARM_HM_ELBOW_RAISE_Q3", cfg.elbow_raise_hold_q3_rad);
    cfg.elbow_raise_hold_release_d_m =
        env_double_or("OPENARM_HM_ELBOW_RAISE_RELEASE_D", cfg.elbow_raise_hold_release_d_m);
    cfg.elbow_raise_hold_medial_fk_scale = std::clamp(
        env_double_or("OPENARM_HM_ELBOW_RAISE_FK_SCALE", cfg.elbow_raise_hold_medial_fk_scale),
        0.0, 1.0);
    cfg.elbow_cmd_slew_droop_scale = std::clamp(
        env_double_or("OPENARM_HM_ELBOW_PHI_DROOP_SCALE", cfg.elbow_cmd_slew_droop_scale), 0.05,
        1.0);
    cfg.elbow_freeze_phi_on_yield =
        env_bool_or("OPENARM_HM_ELBOW_FREEZE_PHI_YIELD", cfg.elbow_freeze_phi_on_yield);
    cfg.elbow_freeze_phi_medial =
        env_double_or("OPENARM_HM_ELBOW_FREEZE_PHI_MEDIAL", cfg.elbow_freeze_phi_medial);

    // Elbow bias defaults to natural gravity droop, with a mild outward preference.
    cfg.w_outward = env_double_or("OPENARM_HM_W_OUTWARD", 0.15);
    cfg.w_upward = env_double_or("OPENARM_HM_W_UPWARD", cfg.w_upward);
    cfg.w_downward = env_double_or("OPENARM_HM_W_DOWNWARD", 0.25);
    cfg.anti_inward_boost =
        env_double_or("OPENARM_HM_ANTI_INWARD", cfg.anti_inward_boost);
    cfg.lateral_droop_boost =
        env_double_or("OPENARM_HM_LATERAL_DROOP", cfg.lateral_droop_boost);
    cfg.lateral_continuity_fade =
        env_double_or("OPENARM_HM_LATERAL_CONT_FADE", cfg.lateral_continuity_fade);
    cfg.elevation_droop_fade =
        env_double_or("OPENARM_HM_ELEV_DROOP_FADE", cfg.elevation_droop_fade);

    // Optional PER-SIDE, per-joint IK limit overrides. joint2 (index 1) is mirrored:
    // right tight side = min (~−10°), left tight side = max (~+10°). That 10° URDF
    // travel is what blocks swinging the arm behind the torso / fully opening the
    // shoulder. In SIM we widen it by default so kinematics can be validated; on
    // hardware it stays at the URDF limit unless explicitly overridden.
    //   OPENARM_HM_QMIN_1_R / OPENARM_HM_QMAX_1_L
    constexpr int kArmDof = 7;
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    cfg.q_min_override.assign(kArmDof, nan_v);
    cfg.q_max_override.assign(kArmDof, nan_v);
    const std::string suf = (side == "left") ? "_L" : "_R";
    bool any_override = false;
    if (sim) {
        // Widen shoulder joint2 (index 1) tight side to ±60° in sim.
        if (side == "right") {
            cfg.q_min_override[1] = env_double_or("OPENARM_HM_QMIN_1_R", -1.047);  // −60°
            any_override = true;
        } else {
            cfg.q_max_override[1] = env_double_or("OPENARM_HM_QMAX_1_L", 1.047);   // +60°
            any_override = true;
        }
        // Widen shoulder yaw joint1 (index 0) past the URDF ~±80° so the arm can
        // swing to pure left/right (±90°+) via upper-arm yaw instead of cheating
        // with an flared elbow. Hardware keeps the stock URDF until calibrated.
        if (side == "right") {
            cfg.q_min_override[0] = env_double_or("OPENARM_HM_QMIN_0_R", -2.5);
            any_override = true;
        } else {
            cfg.q_max_override[0] = env_double_or("OPENARM_HM_QMAX_0_L", 2.5);
            any_override = true;
        }
    }
    for (int i = 0; i < kArmDof; ++i) {
        const std::string kmin = "OPENARM_HM_QMIN_" + std::to_string(i) + suf;
        const std::string kmax = "OPENARM_HM_QMAX_" + std::to_string(i) + suf;
        const double vmin = env_double_or(kmin.c_str(), nan_v);
        const double vmax = env_double_or(kmax.c_str(), nan_v);
        if (std::isfinite(vmin)) {
            cfg.q_min_override[i] = vmin;
            any_override = true;
        }
        if (std::isfinite(vmax)) {
            cfg.q_max_override[i] = vmax;
            any_override = true;
        }
    }
    if (!any_override) {
        cfg.q_min_override.clear();
        cfg.q_max_override.clear();
    }

    cfg.elbow_phi_disambiguate =
        env_bool_or("OPENARM_HM_ELBOW_PHI_DISAMBIG", cfg.elbow_phi_disambiguate);
    cfg.elbow_phi_reach_lock_stretch =
        env_double_or("OPENARM_HM_ELBOW_REACH_LOCK", cfg.elbow_phi_reach_lock_stretch);
    cfg.elbow_phi_obtuse_flex_rad =
        env_double_or("OPENARM_HM_ELBOW_OBTUSE_FLEX", cfg.elbow_phi_obtuse_flex_rad);
    cfg.elbow_singularity_h_m =
        env_double_or("OPENARM_HM_ELBOW_SING_H", cfg.elbow_singularity_h_m);
    cfg.max_reanchor_correction_rad =
        env_double_or("OPENARM_HM_REANCHOR_CAP", cfg.max_reanchor_correction_rad);
    cfg.reanchor_to_commanded =
        env_bool_or("OPENARM_HM_REANCHOR_CMD", cfg.reanchor_to_commanded);
    cfg.tau_orientation = env_double_or("OPENARM_HM_ORI_TAU", cfg.tau_orientation);
    cfg.orientation_deadband_deg =
        env_double_or("OPENARM_HM_ORI_DEADBAND", cfg.orientation_deadband_deg);
    cfg.clutch_settle_ticks =
        static_cast<int>(env_double_or("OPENARM_HM_CLUTCH_SETTLE", cfg.clutch_settle_ticks));
    cfg.clutch_start_motion_m =
        env_double_or("OPENARM_HM_CLUTCH_START_M", cfg.clutch_start_motion_m);
    cfg.clutch_start_rotation_deg =
        env_double_or("OPENARM_HM_CLUTCH_START_DEG", cfg.clutch_start_rotation_deg);
    cfg.clutch_track_settle_ticks = static_cast<int>(
        env_double_or("OPENARM_HM_TRACK_SETTLE", cfg.clutch_track_settle_ticks));
    cfg.clutch_phi_hold_enabled =
        env_bool_or("OPENARM_HM_CLUTCH_PHI_HOLD", cfg.clutch_phi_hold_enabled);
    cfg.clutch_phi_unlock_m =
        env_double_or("OPENARM_HM_CLUTCH_PHI_UNLOCK", cfg.clutch_phi_unlock_m);
    cfg.clutch_phi_blend_span_m =
        env_double_or("OPENARM_HM_CLUTCH_PHI_BLEND", cfg.clutch_phi_blend_span_m);
    cfg.clutch_seed_reach_progress =
        env_bool_or("OPENARM_HM_CLUTCH_SEED_REACH", cfg.clutch_seed_reach_progress);
    cfg.lock_segment_lengths =
        env_bool_or("OPENARM_HM_LOCK_L12", cfg.lock_segment_lengths);
    cfg.auto_calibrate_wrist_tcp =
        env_bool_or("OPENARM_HM_AUTO_WRIST_TCP", cfg.auto_calibrate_wrist_tcp);
    cfg.tip_priority_enabled =
        env_bool_or("OPENARM_HM_TIP_PRIORITY", cfg.tip_priority_enabled);
    cfg.tip_priority_wrist_err_m =
        env_double_or("OPENARM_HM_TIP_WRIST_ERR", cfg.tip_priority_wrist_err_m);
    cfg.tip_priority_elbow_scale_min =
        env_double_or("OPENARM_HM_TIP_ELBOW_MIN", cfg.tip_priority_elbow_scale_min);
    cfg.tip_priority_cont_scale_min =
        env_double_or("OPENARM_HM_TIP_CONT_MIN", cfg.tip_priority_cont_scale_min);
    cfg.tip_priority_shoulder_slew_boost =
        env_double_or("OPENARM_HM_TIP_SH_SLEW", cfg.tip_priority_shoulder_slew_boost);
    cfg.tip_priority_soft_reach_scale =
        env_double_or("OPENARM_HM_TIP_SOFT_SCALE", cfg.tip_priority_soft_reach_scale);
    cfg.tip_yield_slew_per_s =
        env_double_or("OPENARM_HM_TIP_YIELD_SLEW", cfg.tip_yield_slew_per_s);
    cfg.reach_progress_slew_per_s =
        env_double_or("OPENARM_HM_REACH_PROG_SLEW", cfg.reach_progress_slew_per_s);
    cfg.position_map_world_frame =
        env_bool_or("OPENARM_HM_POS_WORLD", cfg.position_map_world_frame);
    cfg.fine_motion_hold_enabled =
        env_bool_or("OPENARM_HM_FINE_HOLD", cfg.fine_motion_hold_enabled);
    cfg.position_deadband_m =
        env_double_or("OPENARM_HM_POS_DEADBAND", cfg.position_deadband_m);
    cfg.q_update_deadband_rad =
        env_double_or("OPENARM_HM_Q_DEADBAND", cfg.q_update_deadband_rad);
    cfg.wrist_hold_err_m =
        env_double_or("OPENARM_HM_WRIST_HOLD_ERR", cfg.wrist_hold_err_m);
    cfg.jog_bypass_hold_ticks = static_cast<int>(
        env_double_or("OPENARM_HM_JOG_BYPASS_TICKS", cfg.jog_bypass_hold_ticks));
    cfg.jog_chase_iters = static_cast<int>(
        env_double_or("OPENARM_HM_JOG_CHASE_ITERS", cfg.jog_chase_iters));
    cfg.jog_chase_dt = env_double_or("OPENARM_HM_JOG_CHASE_DT", cfg.jog_chase_dt);
    cfg.jog_wrist_tol_m = env_double_or("OPENARM_HM_JOG_TOL", cfg.jog_wrist_tol_m);
    cfg.jog_elbow_scale = env_double_or("OPENARM_HM_JOG_ELBOW", cfg.jog_elbow_scale);
    cfg.jog_q_slew_radps = env_double_or("OPENARM_HM_JOG_Q_SLEW", cfg.jog_q_slew_radps);
    cfg.panel_bypass_soft_reach =
        env_bool_or("OPENARM_HM_PANEL_BYPASS_SOFT", true);
    cfg.panel_tcp_ori_scale =
        env_double_or("OPENARM_HM_PANEL_ORI_SCALE", cfg.panel_tcp_ori_scale);

    // Periodic act/settle diagnostics: file by default (not the terminal).
    cfg.debug_log = env_bool_or("OPENARM_HM_DEBUG_LOG", cfg.debug_log);
    cfg.debug_log_console = env_bool_or("OPENARM_HM_LOG_CONSOLE", cfg.debug_log_console);
    cfg.debug_log_hz = env_double_or("OPENARM_HM_LOG_HZ", cfg.debug_log_hz);
    const char* log_path = std::getenv("OPENARM_HM_LOG_PATH");
    if (log_path != nullptr && log_path[0] != '\0') {
        cfg.debug_log_path = log_path;
    } else {
        cfg.debug_log_path = hm_log_session_dir() + "/" + side + "_hm.log";
    }
    return cfg;
}

// Default Quest preset: fixed world mapping and bounded 6D IK.
HumanMimicConfig make_unitree_style_config(const std::string& urdf_path,
                                          const std::string& arm_side,
                                          const std::string& orientation_mapping, bool sim) {
    auto cfg = make_quest_base_config(urdf_path, arm_side, orientation_mapping, sim);
    apply_unitree_style_profile(&cfg);
    cfg.unitree_arm_side = arm_side == "left_arm" ? "left" : "right";
    cfg.reference_yaw_rad = env_double_or("OPENARM_UNITREE_REFERENCE_YAW_DEG", 0.0) * M_PI / 180.0;
    if (!std::isfinite(cfg.reference_yaw_rad)) {
        throw std::runtime_error("reference yaw must be finite");
    }
    const std::string side = arm_side == "left_arm" ? "left" : "right";
    const char* log_path = std::getenv("OPENARM_HM_LOG_PATH");
    if (log_path == nullptr || log_path[0] == '\0') {
        cfg.debug_log_path = hm_log_session_dir() + "/" + side + "_unitree.log";
    }
    return cfg;
}

// PIKA preset: Quest map/clutch with pure 6D DIFF_QP
// (no elbow skeleton / soft-reach / tip-priority). Env: OPENARM_PIKA_*.
HumanMimicConfig make_pika_style_config(const std::string& urdf_path, const std::string& arm_side,
                                        const std::string& orientation_mapping, bool sim) {
    HumanMimicConfig cfg =
        make_quest_base_config(urdf_path, arm_side, orientation_mapping, sim);
    cfg.use_elbow_skeleton = false;
    cfg.reach_soft_enabled = false;
    cfg.tip_priority_enabled = false;
    cfg.extension_yield_enabled = false;
    cfg.fine_motion_hold_enabled = false;
    cfg.reanchor_to_commanded = false;
    cfg.w_elbow = 0.0;
    cfg.w_wrist_pos = env_double_or("OPENARM_PIKA_W_WRIST", 12.0);
    cfg.w_hand_ori = env_double_or("OPENARM_PIKA_W_ORI", 8.0);
    cfg.w_continuity = env_double_or("OPENARM_PIKA_W_CONT", 2.0);
    cfg.w_nominal = env_double_or("OPENARM_PIKA_W_NOM", 0.01);
    cfg.ik_backend = HumanMimicIkBackend::DIFF_QP;
    {
        const char* bk = std::getenv("OPENARM_PIKA_IK_BACKEND");
        if (bk != nullptr) {
            const std::string s(bk);
            if (s == "lm" || s == "LM") {
                cfg.ik_backend = HumanMimicIkBackend::LM;
            } else if (s == "diff_qp" || s == "DIFF_QP" || s == "qp" || s == "QP") {
                cfg.ik_backend = HumanMimicIkBackend::DIFF_QP;
            }
        }
    }
    // Log alongside HM sessions but tag path so A/B diffs are obvious.
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    const char* log_path = std::getenv("OPENARM_HM_LOG_PATH");
    if (log_path == nullptr || log_path[0] == '\0') {
        cfg.debug_log_path = hm_log_session_dir() + "/" + side + "_pika.log";
    }
    return cfg;
}



void print_real_hardware_banner() {
    std::cout << "[quest-cartesian] REAL HARDWARE mode\n"
              << "  - Same Unitree / PIKA controller as sim, with hardware safety filters.\n"
              << "  - Joint references @ " << kQuestReferenceHz
              << " Hz; Quest target update @ ~" << kQuestReceiveHz << " Hz.\n"
              << "  - Internal q_ref integration; MIT-PD gets q_des + qdot_des.\n"
              << "  - Hold Grip to teleop, release to hold pose. Re-grip after Quest timeout.\n"
              << "  - Start with moderate motions; Ctrl+C / record-panel 退出 / 急停锁定:\n"
              << "    freeze joints with MIT Kp/Kd, then disable motors."
              << std::endl;
}

void print_teleop_help() {
    std::cout << "[quest] Unitree / PIKA teleoperation\n"
              << "  Grip: hold to follow hand position and orientation; release to hold.\n"
              << "  Regrip anchors the current hand pose to the current TCP without a jump.\n"
              << "  Front trigger: pull to close the gripper, release to open.\n"
              << "  A+B / X+Y while holding Grip: recenter orientation.\n"
              << "  Quest timeout: hold references and recover through the clutch.\n"
              << std::endl;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "Options:\n"
              << "  --sim                             MuJoCo simulation (no CAN hardware)\n"
              << "  --bimanual                        control both arms (Quest left+right)\n"
              << "  --arm-side right_arm|left_arm     single-arm mode only, default: right_arm\n"
              << "  --follower-can IFACE              single-arm CAN, default: can0\n"
              << "  --follower-can-left IFACE         bimanual left CAN, default: can1\n"
              << "  --follower-can-right IFACE        bimanual right CAN, default: can0\n"
              << "  --urdf PATH                       follower URDF path\n"
              << "  --xml PATH                        MuJoCo scene XML (--sim), default: scene_table.xml\n"
              << "  --quest-port PORT                 single-arm UDP port, default: 47911\n"
              << "  --quest-port-left PORT            bimanual left UDP, default: 47912\n"
              << "  --quest-port-right PORT           bimanual right UDP, default: 47911\n"
              << "  --auto-home                       run startup home motion (real hardware)\n"
              << "  --no-auto-home                    skip startup home (default)\n"
              << "  --sim-table-ready                 teleport sim to table-ready pose (default)\n"
              << "  --no-sim-table-ready              leave sim at MuJoCo zero pose\n"
              << "  --teleop-mode MODE                unitree_style (default) | pika_style\n"
              << "                                    unitree_style: fixed world mapping, bounded 6D LM\n"
              << "                                    pika_style: 6D DIFF_QP (no elbow/soft-reach)\n"
              << "  --orientation-mapping MODE        Quest: absolute (default) | incremental\n"
              << "  --absolute-orientation            alias for --orientation-mapping absolute\n"
              << "  --startup-move                    smoothly move arm(s) to TABLE_READY before teleop\n"
              << "  --no-startup-move                 skip startup move (default)\n"
              << "  --startup-file PATH               startup pose/trajectory/dual-CSV (implies --startup-move)\n"
              << "                                    line: [t] side q0..q6 [grip]; side=left|right|both\n"
              << "                                    or dual merged CSV: time,q0,dq0,tau0,...,q15,...\n"
              << "  --startup-move-sec SEC            startup duration when file has no times (default 4)\n"
              << "  --startup-transition-sec SEC      dual-CSV: S-curve current->first (0=adaptive)\n"
              << "  --quest-ui / --no-quest-ui         enable/disable record+jog UDP panel ports (default on)\n"
              << "  --quest-ui-record-port PORT       action-record panel UDP (default 47930)\n"
              << "  --quest-ui-jog-port PORT          TCP jog panel UDP (default 47931)\n"
              << "  --quest-record-dir DIR            CSV record directory for panel Start\n"
              << "  --quest-jog-step M                one-shot nudge meters (default 0.01)\n"
              << "  --quest-jog-speed M_S             hold-to-jog tip speed m/s (default 0.02)\n"
              << "  --quest-jog-omega RAD_S           hold-to-jog TCP rotation rad/s (default 0.25)\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sim") {
            options->sim = true;
        } else if (arg == "--bimanual") {
            options->bimanual = true;
        } else if (arg == "--arm-side" && i + 1 < argc) {
            options->arm_side = argv[++i];
        } else if (arg == "--follower-can" && i + 1 < argc) {
            options->follower_can = argv[++i];
        } else if (arg == "--follower-can-left" && i + 1 < argc) {
            options->follower_can_left = argv[++i];
        } else if (arg == "--follower-can-right" && i + 1 < argc) {
            options->follower_can_right = argv[++i];
        } else if (arg == "--urdf" && i + 1 < argc) {
            options->urdf_path = argv[++i];
        } else if (arg == "--xml" && i + 1 < argc) {
            options->mujoco_xml = argv[++i];
        } else if (arg == "--quest-port" && i + 1 < argc) {
            options->quest_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-port-left" && i + 1 < argc) {
            options->quest_port_left = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-port-right" && i + 1 < argc) {
            options->quest_port_right = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--no-auto-home") {
            options->auto_home = false;
        } else if (arg == "--auto-home") {
            options->auto_home = true;
        } else if (arg == "--sim-table-ready") {
            options->sim_table_ready = true;
        } else if (arg == "--no-sim-table-ready") {
            options->sim_table_ready = false;
        } else if (arg == "--teleop-mode" && i + 1 < argc) {
            options->teleop_mode = argv[++i];
            if (options->teleop_mode != "pika_style" &&
                options->teleop_mode != "unitree_style") {
                std::cerr << "[quest-cartesian][ERROR] invalid --teleop-mode '"
                          << options->teleop_mode
                          << "' (expected unitree_style | pika_style)"
                          << std::endl;
                return false;
            }
        } else if (arg == "--orientation-mapping" && i + 1 < argc) {
            options->orientation_mapping = argv[++i];
            if (options->orientation_mapping != "incremental" &&
                options->orientation_mapping != "absolute") {
                std::cerr << "[quest-cartesian][ERROR] invalid --orientation-mapping '"
                          << options->orientation_mapping << "' (expected incremental | absolute)"
                          << std::endl;
                return false;
            }
        } else if (arg == "--absolute-orientation") {
            options->orientation_mapping = "absolute";
        } else if (arg == "--startup-move") {
            options->startup_move = true;
        } else if (arg == "--no-startup-move") {
            options->startup_move = false;
        } else if (arg == "--startup-file" && i + 1 < argc) {
            options->startup_file = argv[++i];
            options->startup_move = true;
        } else if (arg == "--startup-move-sec" && i + 1 < argc) {
            options->startup_move_sec = std::stod(argv[++i]);
        } else if (arg == "--startup-transition-sec" && i + 1 < argc) {
            options->startup_transition_sec = std::stod(argv[++i]);
        } else if (arg == "--quest-ui") {
            options->quest_ui = true;
        } else if (arg == "--no-quest-ui") {
            options->quest_ui = false;
        } else if (arg == "--quest-ui-record-port" && i + 1 < argc) {
            options->quest_ui_record_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-ui-jog-port" && i + 1 < argc) {
            options->quest_ui_jog_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-record-dir" && i + 1 < argc) {
            options->quest_record_dir = argv[++i];
        } else if (arg == "--quest-jog-step" && i + 1 < argc) {
            options->quest_jog_step_m = std::stod(argv[++i]);
        } else if (arg == "--quest-jog-speed" && i + 1 < argc) {
            options->quest_jog_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--quest-jog-omega" && i + 1 < argc) {
            options->quest_jog_omega_rps = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[quest-cartesian][ERROR] unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }
    if (!options->bimanual && options->arm_side != "left_arm" && options->arm_side != "right_arm") {
        std::cerr << "[quest-cartesian][ERROR] invalid arm side: " << options->arm_side
                  << std::endl;
        return false;
    }
    return true;
}

std::vector<JointState> collect_joint_states(const std::shared_ptr<RobotSystemState>& state) {
    std::vector<JointState> joints;
    const auto arm = state->arm_state().get_all_responses();
    const auto hand = state->hand_state().get_all_responses();
    joints.insert(joints.end(), arm.begin(), arm.end());
    joints.insert(joints.end(), hand.begin(), hand.end());
    return joints;
}


// Soft exit: freeze the first measured pose and keep MIT Kp so the arm holds
// instead of sagging. Chasing live q with Kp=0 twitch-then-drops under gravity.
void send_kd_only_release(openarm::can::socket::OpenArm* left_openarm,
                          openarm::can::socket::OpenArm* right_openarm,
                          const std::vector<double>& kp, const std::vector<double>& kd,
                          double duration_sec = -1.0) {
    double duration = duration_sec;
    if (duration < 0.0) {
        const char* env = std::getenv("OPENARM_QUEST_DAMP_SEC");
        if (env == nullptr || *env == '\0') {
            env = std::getenv("OPENARM_REPLAY_DAMP_SEC");
        }
        if (env != nullptr && *env != '\0') {
            try {
                duration = std::stod(env);
            } catch (...) {
                duration = 6.0;
            }
        } else {
            duration = 6.0;
        }
    }
    duration = std::clamp(duration, 0.2, 60.0);

    double kd_scale = 1.0;
    if (const char* env = std::getenv("OPENARM_QUEST_DAMP_KD_SCALE"); env && *env) {
        try {
            kd_scale = std::stod(env);
        } catch (...) {
            kd_scale = 1.0;
        }
    } else if (const char* env = std::getenv("OPENARM_REPLAY_DAMP_KD_SCALE"); env && *env) {
        try {
            kd_scale = std::stod(env);
        } catch (...) {
            kd_scale = 1.0;
        }
    }
    kd_scale = std::clamp(kd_scale, 0.05, 5.0);

    double kp_scale = 1.0;
    if (const char* env = std::getenv("OPENARM_QUEST_STOP_KP_SCALE"); env && *env) {
        try {
            kp_scale = std::stod(env);
        } catch (...) {
            kp_scale = 1.0;
        }
    }
    kp_scale = std::clamp(kp_scale, 0.0, 2.0);

    const int period_ms = 5;
    const int cycles = std::max(1, static_cast<int>(std::lround(duration * 1000.0 / period_ms)));

    struct HeldPose {
        std::vector<double> arm_q;
        std::vector<double> gripper_q;
        bool valid = false;
    };
    const auto capture = [](openarm::can::socket::OpenArm* openarm) {
        HeldPose held;
        if (openarm == nullptr) {
            return held;
        }
        openarm->recv_all(200);
        for (const auto& motor : openarm->get_arm().get_motors()) {
            held.arm_q.push_back(motor.get_position());
        }
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            held.gripper_q.push_back(motor.get_position());
        }
        held.valid = !held.arm_q.empty() || !held.gripper_q.empty();
        return held;
    };
    const HeldPose left_held = capture(left_openarm);
    const HeldPose right_held = capture(right_openarm);

    const auto send_held = [&kp, &kd, kd_scale, kp_scale](
                               openarm::can::socket::OpenArm* openarm, const HeldPose& held) {
        if (openarm == nullptr || !held.valid) {
            return;
        }
        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(held.arm_q.size());
        size_t idx = 0;
        for (double q : held.arm_q) {
            const double stiffness = (idx < kp.size() ? kp[idx] : 24.0) * kp_scale;
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            arm_cmds.push_back({stiffness, damping, q, 0.0, 0.0});
            ++idx;
        }
        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(held.gripper_q.size());
        for (double q : held.gripper_q) {
            const double stiffness = (idx < kp.size() ? kp[idx] : 16.0) * kp_scale;
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            gripper_cmds.push_back({stiffness, damping, q, 0.0, 0.0});
            ++idx;
        }
        openarm->get_arm().mit_control_all(arm_cmds);
        openarm->get_gripper().mit_control_all(gripper_cmds);
    };

    if (left_openarm == nullptr && right_openarm == nullptr) {
        return;
    }
    std::cout << "[quest-cartesian] frozen-q position hold for " << duration
              << " s (kp_scale=" << kp_scale << ", kd_scale=" << kd_scale
              << "); then disable. Set OPENARM_QUEST_STOP_KP_SCALE=0 for old sag." << std::endl;
    for (int i = 0; i < cycles; ++i) {
        send_held(left_openarm, left_held);
        send_held(right_openarm, right_held);
        if (left_openarm != nullptr) {
            left_openarm->recv_all(200);
        }
        if (right_openarm != nullptr) {
            right_openarm->recv_all(200);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}

// Lightweight UDP: left/right gripper measured torque + MOS/rotor temps for Quest HUD.
class GripperTelemSender {
public:
    bool open(uint16_t port) {
        close();
        if (port == 0) {
            return false;
        }
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::cerr << "[quest-grip-telem] socket failed" << std::endl;
            return false;
        }
        addr_ = {};
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr_.sin_port = htons(port);
        std::cout << "[quest-grip-telem] -> 127.0.0.1:" << port << std::endl;
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~GripperTelemSender() { close(); }

    void maybe_send(openarm::can::socket::OpenArm* left, openarm::can::socket::OpenArm* right,
                    double min_period_s = 0.05) {
        if (fd_ < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (last_send_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_send_).count() < min_period_s) {
            return;
        }
        last_send_ = now;

        double l_tau = 0, l_q = 0, r_tau = 0, r_q = 0;
        int l_mos = 0, l_rot = 0, r_mos = 0, r_rot = 0;
        read_gripper(left, &l_tau, &l_q, &l_mos, &l_rot);
        read_gripper(right, &r_tau, &r_q, &r_mos, &r_rot);

        const double t =
            std::chrono::duration<double>(now.time_since_epoch()).count();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << t << "," << l_tau << "," << l_mos << ","
            << l_rot << "," << l_q << "," << r_tau << "," << r_mos << "," << r_rot << "," << r_q;
        const std::string msg = oss.str();
        ::sendto(fd_, msg.data(), msg.size(), 0, reinterpret_cast<sockaddr*>(&addr_),
                 sizeof(addr_));
    }

private:
    static void read_gripper(openarm::can::socket::OpenArm* openarm, double* tau, double* q,
                             int* tmos, int* trotor) {
        if (openarm == nullptr) {
            return;
        }
        const auto& motors = openarm->get_gripper().get_motors();
        if (motors.empty()) {
            return;
        }
        const auto& m = motors.front();
        *tau = m.get_torque();
        *q = m.get_position();
        *tmos = m.get_state_tmos();
        *trotor = m.get_state_trotor();
    }

    int fd_ = -1;
    sockaddr_in addr_{};
    std::chrono::steady_clock::time_point last_send_{};
};


struct ArmTeleopBundle {
    std::string label;
    std::string arm_side;
    std::string mujoco_side;
    Dynamics* dynamics = nullptr;
    Control* control = nullptr;
    std::shared_ptr<RobotSystemState> state;
    std::unique_ptr<HumanMimicController> human_mimic;
    std::unique_ptr<QuestPoseReceiver> quest_receiver;
    std::shared_ptr<MujocoTeleopHardware> sim_hardware;
    openarm::can::socket::OpenArm* openarm = nullptr;
    uint64_t log_tick = 0;
    bool is_left = false;
};

void append_gripper_motor_status(std::ostringstream& oss, ArmTeleopBundle* arm) {
    if (arm == nullptr || arm->openarm == nullptr) {
        oss << " grip_track=0 motor_err=- grip_tau=0 t_mos=0 t_rot=0 enabled=-";
        return;
    }
    const int err = motor_gripper_error_code(arm->openarm);
    const double tau = motor_gripper_tau(arm->openarm);
    const int t_mos = motor_gripper_tmos(arm->openarm);
    const int t_rot = motor_gripper_trotor(arm->openarm);
    double track = 0.0;
    if (arm->human_mimic != nullptr) {
        track = arm->human_mimic->gripper_measured() - arm->human_mimic->gripper_target();
    }
    int enabled = 0;
    const auto& gmotors = arm->openarm->get_gripper().get_motors();
    if (!gmotors.empty()) {
        enabled = gmotors.front().is_enabled() ? 1 : 0;
    }
    oss << " grip_track=" << track
        << " motor_err=" << err << "(" << motor_fault_text(err) << ")"
        << " grip_tau=" << tau
        << " t_mos=" << t_mos
        << " t_rot=" << t_rot
        << " enabled=" << enabled;
}

// Desktop-panel bridge: UDP record (start/stop/dump) + TCP jog (hold/release or nudge).
struct QuestUiBridge {
    UdpTextReceiver record_rx;
    UdpTextReceiver jog_rx;
    CsvRecorder recorder_left;
    CsvRecorder recorder_right;
    std::string sessions_root = "recordings/sessions";  // parent of named batches
    std::string record_dir;          // batch: sessions_root/<name>  (empty until first start/dump)
    std::string active_session_dir;  // current segment: .../01 , .../02
    std::string active_left_csv;
    std::string active_right_csv;
    std::string dump_path;
    double jog_step_m = 0.01;
    double jog_speed_mps = 0.02;
    double jog_omega_rps = 0.25;
    int segment = 0;
    ArmTeleopBundle* left = nullptr;
    ArmTeleopBundle* right = nullptr;
    std::set<std::string> jog_dirs_left;
    std::set<std::string> jog_dirs_right;

    // Stuck-joint detector state (position frozen while other joints move).
    std::vector<JointState> first_meas_left_;
    std::vector<JointState> first_meas_right_;
    std::vector<double> joint_moved_left_;
    std::vector<double> joint_moved_right_;
    bool stuck_warn_left_ = false;
    bool stuck_warn_right_ = false;
    std::mutex record_mutex_;
    // LeRobot episode log: 30 Hz measured qpos (observation.state) and MIT
    // position command (action), left 8 + right 8. Packed on Stop.
    std::ofstream lerobot_csv_;
    int lerobot_rows_ = 0;
    bool lerobot_pending_export_ = false;
    std::chrono::steady_clock::time_point lerobot_t0_{};
    std::chrono::steady_clock::time_point lerobot_next_{};
    double lerobot_obs_l_[8]{};
    double lerobot_act_l_[8]{};
    double lerobot_obs_r_[8]{};
    double lerobot_act_r_[8]{};
    bool lerobot_got_l_ = false;
    bool lerobot_got_r_ = false;
    bool lerobot_awaiting_label_ = false;
    // Last soft-deleted take. `redo` renames the episode into recordings/trash;
    // `undo` moves it back until this deadline. The Qt console uses the same
    // 10s window and replies before the rename finishes.
    struct TrashedTake {
        std::string original;
        std::string trashed;
        int segment_number = 0;
        bool awaiting_label = false;
        std::chrono::steady_clock::time_point until{};
    };
    std::optional<TrashedTake> trashed_take_;
    pid_t camera_pid_ = -1;
    bool camera_view_attached_ = false;

    // Master (right) / slave (left) relative pose lock.
    bool couple_active = false;
    Eigen::Vector3d couple_p_rel = Eigen::Vector3d::Zero();   // left in right TCP frame
    Eigen::Matrix3d couple_R_rel = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d couple_R_right = Eigen::Matrix3d::Identity();
    // Relative residual closed-loop (scheme 1): correct left target using
    // Δp = left_tcp − (p_r + R_freeze·p_rel). Integral removes FB/UD bias.
    Eigen::Vector3d couple_integral = Eigen::Vector3d::Zero();
    uint64_t couple_fb_log_tick = 0;
    bool couple_fb_enabled = true;
    double couple_fb_kp = 0.25;
    double couple_fb_ki = 0.4;
    double couple_fb_imax = 0.02;     // m, per-axis integral clamp
    double couple_fb_meas_w = 0.25;   // blend measured residual into command residual
    double couple_fb_leak = 0.25;     // 1/s leak on integral
    double couple_fb_corr_max = 0.06; // m, total correction clamp
    // Below this residual: no integrate, bleed I, prefer open-loop geometric pose.
    double couple_fb_deadband_m = 0.008;
    double couple_fb_deadband_leak = 3.0;  // 1/s — drain I quickly inside deadband
    bool couple_p_r_prev_valid = false;
    Eigen::Vector3d couple_p_r_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d couple_p_l_prev = Eigen::Vector3d::Zero();
    bool couple_p_l_prev_valid = false;

    // Couple-mode horizontal circle (right master tip around engage-time center).
    bool circle_active = false;
    bool circle_radial_phase = false;  // true: grow radius 0→r before orbiting
    Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d circle_R = Eigen::Matrix3d::Identity();
    double circle_radius_m = 0.05;
    double circle_theta = 0.0;
    double circle_period_s = 8.0;
    double circle_ramp_t = 0.0;
    double circle_ramp_s = 1.0;       // seconds to reach full radius
    double circle_tip_speed_mps = 0.06;  // tip rate limit during circle (m/s)

    void reanchor_arm_vr_map(ArmTeleopBundle* arm) {
        if (arm == nullptr || arm->human_mimic == nullptr || arm->state == nullptr) {
            return;
        }
        const auto joints = collect_joint_states(arm->state);
        std::vector<double> q_meas;
        q_meas.reserve(joints.size());
        for (const auto& j : joints) {
            q_meas.push_back(j.position);
        }
        QuestPoseSample sample;
        sample.valid = false;
        if (arm->quest_receiver != nullptr) {
            auto latest = arm->quest_receiver->latest();
            if (latest.has_value()) {
                sample = latest.value();
            }
        }
        arm->human_mimic->reanchor_vr_map(q_meas, sample);
    }

    void stop_circle(const char* reason = nullptr) {
        if (!circle_active) {
            return;
        }
        circle_active = false;
        circle_radial_phase = false;
        circle_theta = 0.0;
        circle_ramp_t = 0.0;
        if (right != nullptr && right->human_mimic != nullptr) {
            // Right was temporarily couple_follow so VR is skipped while circling.
            right->human_mimic->set_couple_follow(false);
            reanchor_arm_vr_map(right);
        }
        jog_dirs_right.clear();
        std::cout << "[quest-ui] circle STOP"
                  << (reason != nullptr ? std::string(" (") + reason + ")" : "") << std::endl;
    }

    bool start_circle(double radius_mm) {
        if (!couple_active) {
            std::cerr << "[quest-ui] circle start rejected: couple not active" << std::endl;
            return false;
        }
        if (right == nullptr || right->human_mimic == nullptr) {
            std::cerr << "[quest-ui] circle start rejected: no right Quest controller" << std::endl;
            return false;
        }
        const double r_m = std::clamp(radius_mm, 1.0, 500.0) * 0.001;
        Eigen::Vector3d p_r;
        Eigen::Matrix3d R_r;
        if (!right->human_mimic->commanded_tcp_pose(&p_r, &R_r) || !p_r.allFinite() ||
            !R_r.allFinite()) {
            std::cerr << "[quest-ui] circle start rejected: no commanded TCP" << std::endl;
            return false;
        }
        circle_center = p_r;
        circle_R = R_r;
        circle_radius_m = r_m;
        circle_theta = 0.0;
        circle_ramp_t = 0.0;
        circle_radial_phase = true;
        circle_period_s = std::max(1.0, env_double_or("OPENARM_QUEST_CIRCLE_PERIOD", 8.0));
        circle_ramp_s = std::max(0.2, env_double_or("OPENARM_QUEST_CIRCLE_RAMP", 1.0));
        circle_tip_speed_mps =
            std::max(0.01, env_double_or("OPENARM_QUEST_CIRCLE_TIP_SPEED", 0.06));
        right->human_mimic->set_couple_follow(true);
        jog_dirs_right.clear();
        circle_active = true;
        const double v_orbit_mms = (2.0 * M_PI * circle_radius_m / circle_period_s) * 1000.0;
        std::cout << "[quest-ui] circle START  r=" << (circle_radius_m * 1000.0) << " mm  T="
                  << circle_period_s << " s  orbit≈" << v_orbit_mms
                  << " mm/s  ramp=" << circle_ramp_s << " s  tip_cap="
                  << (circle_tip_speed_mps * 1000.0) << " mm/s  (XY, one lap)" << std::endl;
        return true;
    }

    // Soft tip rate limit so a large radius jump cannot slam MIT / SafetyFilter.
    void drive_circle_tcp(const Eigen::Vector3d& p_des, double dt) {
        Eigen::Vector3d p_cmd = p_des;
        Eigen::Vector3d p_now;
        Eigen::Matrix3d R_tmp;
        if (right->human_mimic->commanded_tcp_pose(&p_now, &R_tmp) && p_now.allFinite()) {
            Eigen::Vector3d d = p_cmd - p_now;
            const double max_step =
                std::max(1e-4, circle_tip_speed_mps) * std::max(1e-4, dt);
            const double dn = d.norm();
            if (dn > max_step && dn > 1e-9) {
                p_cmd = p_now + d * (max_step / dn);
            }
        }
        if (!p_cmd.allFinite()) {
            stop_circle("non-finite TCP target");
            return;
        }
        right->human_mimic->drive_tcp_robot(p_cmd, circle_R, dt);
    }

    // Advance right TCP along the circle. Returns true when the lap just finished
    // (final pose already driven); caller should step_arm then stop_circle.
    bool advance_circle(double dt) {
        if (!circle_active || !couple_active || right == nullptr || right->human_mimic == nullptr) {
            return false;
        }
        dt = std::max(0.0, dt);
        Eigen::Vector3d p = circle_center;

        if (circle_radial_phase) {
            // Grow radius smoothly from center → (r,0) before orbiting (avoids
            // an instantaneous r jump that spikes torque with raw yaml Kp).
            circle_ramp_t += dt;
            double a = std::clamp(circle_ramp_t / std::max(1e-3, circle_ramp_s), 0.0, 1.0);
            a = a * a * (3.0 - 2.0 * a);  // smoothstep
            p.x() += circle_radius_m * a;
            drive_circle_tcp(p, dt);
            if (circle_ramp_t >= circle_ramp_s) {
                circle_radial_phase = false;
                circle_theta = 0.0;
                std::cout << "[quest-ui] circle ramp done — start orbit" << std::endl;
            }
            return false;
        }

        const double T = std::max(1e-3, circle_period_s);
        circle_theta += (2.0 * M_PI / T) * dt;
        bool finished = false;
        if (circle_theta >= 2.0 * M_PI) {
            circle_theta = 2.0 * M_PI;
            finished = true;
        }
        p.x() += circle_radius_m * std::cos(circle_theta);
        p.y() += circle_radius_m * std::sin(circle_theta);
        // z stays at center.z
        drive_circle_tcp(p, dt);
        return finished;
    }

    void load_couple_fb_gains() {
        // Geometry-first but catch residual near limits (was KP=0.25/KI=0.4/CORR=3cm).
        // Still below the old aggressive 0.55/1.2/8cm that permanently biased offset.
        couple_fb_enabled = env_bool_or("OPENARM_COUPLE_FB", true);
        couple_fb_kp = env_double_or("OPENARM_COUPLE_KP", 0.4);
        couple_fb_ki = env_double_or("OPENARM_COUPLE_KI", 0.7);
        couple_fb_imax = env_double_or("OPENARM_COUPLE_IMAX", 0.03);
        couple_fb_meas_w = std::clamp(env_double_or("OPENARM_COUPLE_MEAS_W", 0.25), 0.0, 1.0);
        couple_fb_leak = std::max(0.0, env_double_or("OPENARM_COUPLE_ILEAK", 0.25));
        couple_fb_corr_max = std::max(0.005, env_double_or("OPENARM_COUPLE_CORR_MAX", 0.06));
        couple_fb_deadband_m =
            std::max(0.0, env_double_or("OPENARM_COUPLE_DEADBAND", 0.008));
        couple_fb_deadband_leak =
            std::max(0.0, env_double_or("OPENARM_COUPLE_DB_LEAK", 3.0));
    }

    static bool arm_tcp_pose(ArmTeleopBundle* arm, Eigen::Vector3d* p, Eigen::Matrix3d* R) {
        if (arm == nullptr) {
            return false;
        }
        if (arm->human_mimic != nullptr && arm->human_mimic->commanded_tcp_pose(p, R)) {
            return true;
        }

        if (arm->dynamics == nullptr || arm->state == nullptr) {
            return false;
        }
        const auto joints = collect_joint_states(arm->state);
        const size_t dof = arm->dynamics->GetDof();
        std::vector<double> q(dof, 0.0);
        for (size_t i = 0; i < dof && i < joints.size(); ++i) {
            q[i] = joints[i].position;
        }
        Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        arm->dynamics->GetEECordinate(q.data(), r, pos);
        if (!pos.allFinite() || !r.allFinite()) {
            return false;
        }
        if (p != nullptr) {
            *p = pos;
        }
        if (R != nullptr) {
            *R = r;
        }
        return true;
    }

    static void arm_set_orientation_lock(ArmTeleopBundle* arm, bool on,
                                         const Eigen::Matrix3d& R_lock) {
        if (arm == nullptr) {
            return;
        }
        if (arm->human_mimic != nullptr) {
            arm->human_mimic->set_orientation_lock(on, R_lock);
        }
    }

    static void arm_set_couple_follow(ArmTeleopBundle* arm, bool on) {
        if (arm == nullptr) {
            return;
        }
        if (arm->human_mimic != nullptr) {
            arm->human_mimic->set_couple_follow(on);
        }
    }

    static void arm_drive_tcp(ArmTeleopBundle* arm, const Eigen::Vector3d& p,
                              const Eigen::Matrix3d& R, double dt) {
        if (arm == nullptr) {
            return;
        }
        if (arm->human_mimic != nullptr) {
            arm->human_mimic->drive_tcp_robot(p, R, dt);
            return;
        }
    }

    bool engage_couple() {
        if (left == nullptr || right == nullptr) {
            return false;
        }
        Eigen::Vector3d p_l, p_r;
        Eigen::Matrix3d R_l, R_r;
        if (!arm_tcp_pose(left, &p_l, &R_l) || !arm_tcp_pose(right, &p_r, &R_r)) {
            std::cerr << "[quest-ui] couple engage failed: no commanded TCP" << std::endl;
            return false;
        }
        load_couple_fb_gains();
        couple_R_right = R_r;
        couple_p_rel = R_r.transpose() * (p_l - p_r);
        couple_R_rel = R_r.transpose() * R_l;
        couple_integral.setZero();
        couple_fb_log_tick = 0;
        couple_p_r_prev = p_r;
        couple_p_r_prev_valid = true;
        couple_p_l_prev = p_l;
        couple_p_l_prev_valid = true;
        arm_set_orientation_lock(right, true, R_r);
        arm_set_orientation_lock(left, true, R_l);
        arm_set_couple_follow(left, true);
        couple_active = true;
        jog_dirs_left.clear();
        std::cout << "[quest-ui] couple ON — right=position-only master, left=relative slave  "
                     "||p_rel||="
                  << couple_p_rel.norm() << " m"
                  << "  fb=" << (couple_fb_enabled ? 1 : 0) << " kp=" << couple_fb_kp
                  << " ki=" << couple_fb_ki << " corr_max=" << couple_fb_corr_max << std::endl;
        return true;
    }

    void disengage_couple() {
        if (!couple_active) {
            return;
        }
        stop_circle("couple off");
        couple_active = false;
        couple_integral.setZero();
        couple_p_r_prev_valid = false;
        couple_p_l_prev_valid = false;
        // Soft exit: drop absolute follow, then re-anchor VR relative map to the
        // current Quest sample + commanded TCP before unlocking orientation.
        // Without this, the first post-couple tick dumps (VR_now - stale_hand_ref)
        // onto a robot_ref that was updated during couple → visible snap/突变.
        if (left != nullptr) {
            arm_set_couple_follow(left, false);
            reanchor_arm_vr_map(left);
            arm_set_orientation_lock(left, false, Eigen::Matrix3d::Identity());
        }
        if (right != nullptr) {
            arm_set_couple_follow(right, false);
            reanchor_arm_vr_map(right);
            arm_set_orientation_lock(right, false, Eigen::Matrix3d::Identity());
        }
        std::cout << "[quest-ui] couple OFF (VR map re-anchored)" << std::endl;
    }

    void apply_couple_left_follow(double dt) {
        if (!couple_active || left == nullptr || right == nullptr) {
            return;
        }
        Eigen::Vector3d p_r;
        Eigen::Matrix3d R_r_cmd;
        if (!arm_tcp_pose(right, &p_r, &R_r_cmd)) {
            return;
        }
        // Open-loop geometric slave pose (right orientation frozen at engage).
        // Equivalent to fixed world Δp while ori locks hold.
        const Eigen::Vector3d p_l_ol = p_r + couple_R_right * couple_p_rel;
        const Eigen::Matrix3d R_l = couple_R_right * couple_R_rel;

        Eigen::Vector3d p_l_des = p_l_ol;
        if (couple_fb_enabled && dt > 0.0) {
            Eigen::Vector3d p_l_cmd;
            Eigen::Matrix3d R_tmp;
            if (!arm_tcp_pose(left, &p_l_cmd, &R_tmp)) {
                arm_drive_tcp(left, p_l_ol, R_l, dt);
                couple_p_r_prev = p_r;
                couple_p_r_prev_valid = true;
                return;
            }
            Eigen::Vector3d residual = p_l_cmd - p_l_ol;
            if (couple_fb_meas_w > 1e-6 && left->state != nullptr) {
                Eigen::Vector3d p_l_meas;
                bool have_meas = false;
                if (left->human_mimic != nullptr) {
                    const auto arm = left->state->arm_state().get_all_responses();
                    std::vector<double> q_meas;
                    q_meas.reserve(arm.size());
                    for (const auto& j : arm) {
                        q_meas.push_back(j.position);
                    }
                    have_meas = left->human_mimic->fk_tcp_pose(q_meas, &p_l_meas, &R_tmp);
                }
                if (have_meas && p_l_meas.allFinite()) {
                    const Eigen::Vector3d r_meas = p_l_meas - p_l_ol;
                    residual = (1.0 - couple_fb_meas_w) * residual + couple_fb_meas_w * r_meas;
                }
            }
            if (residual.allFinite()) {
                const double rn = residual.norm();
                const double db = couple_fb_deadband_m;
                if (rn <= db) {
                    // Inside deadband: open-loop geometry + bleed windup.
                    const double leak_db =
                        std::max(couple_fb_leak, couple_fb_deadband_leak);
                    couple_integral *= std::max(0.0, 1.0 - leak_db * dt);
                    if (couple_integral.norm() < 1e-4) {
                        couple_integral.setZero();
                    }
                    p_l_des = p_l_ol;
                } else {
                    couple_integral *= std::max(0.0, 1.0 - couple_fb_leak * dt);
                    Eigen::Vector3d r_int = residual;
                    if (db > 1e-9 && rn > 1e-9) {
                        r_int *= (rn - db) / rn;
                    }
                    const Eigen::Vector3d corr_p = couple_fb_kp * residual + couple_integral;
                    const bool saturated =
                        corr_p.norm() >= couple_fb_corr_max - 1e-6 &&
                        corr_p.dot(r_int) > 0.0;
                    if (!saturated) {
                        couple_integral += couple_fb_ki * r_int * dt;
                    }
                    for (int i = 0; i < 3; ++i) {
                        couple_integral[i] =
                            std::clamp(couple_integral[i], -couple_fb_imax, couple_fb_imax);
                    }
                    Eigen::Vector3d corr = couple_fb_kp * residual + couple_integral;
                    const double cn = corr.norm();
                    if (cn > couple_fb_corr_max && cn > 1e-9) {
                        corr *= couple_fb_corr_max / cn;
                    }
                    p_l_des = p_l_ol - corr;
                }
                if (++couple_fb_log_tick >= 50) {  // ~1 Hz at 50 Hz teleop
                    couple_fb_log_tick = 0;
                    double v_r = 0.0;
                    double v_l = 0.0;
                    if (couple_p_r_prev_valid && dt > 1e-6) {
                        v_r = (p_r - couple_p_r_prev).norm() / dt;
                    }
                    if (couple_p_l_prev_valid && dt > 1e-6) {
                        v_l = (p_l_cmd - couple_p_l_prev).norm() / dt;
                    }
                    std::cout << "[quest-ui] couple-fb  |res|=" << rn * 1000.0
                              << " mm  |I|=" << couple_integral.norm() * 1000.0
                              << " mm  |corr|="
                              << (p_l_des - p_l_ol).norm() * 1000.0 << " mm"
                              << "  |vr|=" << v_r * 1000.0 << "  |vl|=" << v_l * 1000.0
                              << " mm/s";
                    if (left->human_mimic != nullptr) {
                        std::cout << "  phi_flip=" << left->human_mimic->last_phi_flip()
                                  << "  e_w=" << left->human_mimic->last_wrist_err() * 1000.0
                                  << " mm";
                    }
                    std::cout << (rn <= db ? "  (deadband→OL)" : "") << std::endl;
                    couple_p_l_prev = p_l_cmd;
                    couple_p_l_prev_valid = true;
                }
            }
        }
        arm_drive_tcp(left, p_l_des, R_l, dt);
        couple_p_r_prev = p_r;
        couple_p_r_prev_valid = true;
    }

    bool open(const Options& opt) {
        if (!opt.quest_ui) {
            return true;
        }
        jog_step_m = std::max(1e-4, opt.quest_jog_step_m);
        jog_speed_mps = std::max(1e-4, opt.quest_jog_speed_mps);
        jog_omega_rps = std::max(1e-4, opt.quest_jog_omega_rps);
        if (!record_rx.open(opt.quest_ui_record_port, "quest-ui-record")) {
            return false;
        }
        if (!jog_rx.open(opt.quest_ui_jog_port, "quest-ui-jog")) {
            return false;
        }
        if (!opt.quest_record_dir.empty()) {
            sessions_root = opt.quest_record_dir;
        } else {
            sessions_root = "recordings/sessions";
        }
        record_dir.clear();  // set on first start,<name> / dump,<name>
        dump_path.clear();
        std::cout << "Quest UI       : record UDP " << opt.quest_ui_record_port
                  << "  jog UDP " << opt.quest_ui_jog_port
                  << "  sessions_root=" << sessions_root
                  << "  jog_speed=" << jog_speed_mps << " m/s"
                  << "  jog_omega=" << jog_omega_rps << " rad/s" << std::endl;
        circle_period_s = std::max(1.0, env_double_or("OPENARM_QUEST_CIRCLE_PERIOD", 8.0));
        std::cout << "  [quest-ui] session folder created only after Start/Dump; "
                     "name comes from panel (start,<name>)"
                  << std::endl;
        std::cout << "  [quest-ui] couple circle period T=" << circle_period_s
                  << " s (OPENARM_QUEST_CIRCLE_PERIOD)" << std::endl;
        return true;
    }

    static std::string sanitize_session_name(std::string name) {
        for (char& c : name) {
            const unsigned char uc = static_cast<unsigned char>(c);
            if (std::isalnum(uc) || c == '_' || c == '-' || c == '.') {
                continue;
            }
            c = '_';
        }
        while (!name.empty() && (name.front() == '_' || name.front() == '.' || name.front() == '-')) {
            name.erase(name.begin());
        }
        while (!name.empty() && (name.back() == '_' || name.back() == '.' || name.back() == '-')) {
            name.pop_back();
        }
        if (name.empty()) {
            const auto stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm tm{};
            localtime_r(&stamp, &tm);
            char buf[64];
            std::strftime(buf, sizeof(buf), "quest_%Y%m%d_%H%M%S", &tm);
            name = buf;
        }
        return name;
    }

    bool ensure_batch_dir(const std::string& raw_name, bool ensure_dirs) {
        const std::string name = sanitize_session_name(raw_name);
        const std::string next_dir = sessions_root + "/" + name;
        if (next_dir != record_dir) {
            segment = 0;
        }
        record_dir = next_dir;
        dump_path = record_dir + "/pose_snapshots.txt";
        if (ensure_dirs) {
            std::error_code ec;
            std::filesystem::create_directories(record_dir, ec);
            if (ec) {
                std::cerr << "[quest-ui] failed to create " << record_dir << ": " << ec.message()
                          << std::endl;
                return false;
            }
        }
        return true;
    }

    static void copy_qpos8(const std::vector<JointState>& src, double* dst) {
        for (int i = 0; i < 8; ++i) {
            dst[i] = 0.0;
        }
        const size_t n = std::min(src.size(), size_t{8});
        for (size_t i = 0; i < n; ++i) {
            dst[i] = src[i].position;
        }
    }

    void open_lerobot_episode() {
        std::string camera_out;
        {
            std::lock_guard<std::mutex> lock(record_mutex_);
            lerobot_rows_ = 0;
            lerobot_got_l_ = false;
            lerobot_got_r_ = false;
            lerobot_next_ = {};
            std::fill(std::begin(lerobot_obs_l_), std::end(lerobot_obs_l_), 0.0);
            std::fill(std::begin(lerobot_act_l_), std::end(lerobot_act_l_), 0.0);
            std::fill(std::begin(lerobot_obs_r_), std::end(lerobot_obs_r_), 0.0);
            std::fill(std::begin(lerobot_act_r_), std::end(lerobot_act_r_), 0.0);
            if (active_session_dir.empty()) {
                return;
            }
            const std::string path = active_session_dir + "/lerobot_episode.csv";
            lerobot_csv_.open(path, std::ios::out | std::ios::trunc);
            if (!lerobot_csv_) {
                std::cerr << "[lerobot] failed to open " << path << std::endl;
                return;
            }
            lerobot_t0_ = std::chrono::steady_clock::now();
            lerobot_csv_ << std::fixed << std::setprecision(6);
            lerobot_csv_ << "time";
            for (const char* side : {"l", "r"}) {
                for (const char* kind : {"obs", "act"}) {
                    for (int i = 0; i < 8; ++i) {
                        lerobot_csv_ << "," << side << "_" << kind << i;
                    }
                }
            }
            lerobot_csv_ << ",unix\n";
            camera_out = active_session_dir + "/cameras";
            std::cout << "[lerobot] logging " << path << std::endl;
        }
        start_camera_capture(camera_out);
    }

    static std::string lerobot_script(const char* filename) {
        std::string script = std::string("script/") + filename;
        std::error_code ec;
        if (std::filesystem::is_regular_file(script, ec)) {
            return script;
        }
        const std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
        if (ec) {
            return script;
        }
        return (exe.parent_path().parent_path() / "script" / filename).string();
    }

    static bool camera_view_command(const std::string& line) {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            return false;
        }
        timeval tv{};
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(47932);
        if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1 ||
            ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd);
            return false;
        }
        const std::string msg = line + "\n";
        if (::send(fd, msg.data(), msg.size(), 0) != static_cast<ssize_t>(msg.size())) {
            ::close(fd);
            return false;
        }
        char buf[16] = {};
        const ssize_t got = ::recv(fd, buf, sizeof(buf) - 1, 0);
        ::close(fd);
        return got > 0 && std::string(buf).find("ok") != std::string::npos;
    }

    void start_camera_capture(const std::string& out) {
        {
            std::lock_guard<std::mutex> lock(record_mutex_);
            if (camera_pid_ > 0 || camera_view_attached_ || out.empty()) {
                return;
            }
        }
        std::error_code ec;
        std::filesystem::create_directories(out, ec);
        if (camera_view_command("record " + out)) {
            std::lock_guard<std::mutex> lock(record_mutex_);
            camera_view_attached_ = true;
            std::cout << "[lerobot] cameras stay in the view windows -> " << out << std::endl;
            return;
        }
        const std::string script = lerobot_script("lerobot_cameras.py");
        if (!std::filesystem::is_regular_file(script)) {
            std::cerr << "[lerobot] camera script not found; joints only" << std::endl;
            return;
        }
        const char* py = std::getenv("OPENARM_LEROBOT_PYTHON");
        if (py == nullptr || py[0] == '\0') {
            py = "python3";
        }
        std::vector<std::string> storage = {py, script, "record", "--out", out};
        std::vector<char*> argv;
        argv.reserve(storage.size() + 1);
        for (auto& item : storage) {
            argv.push_back(item.data());
        }
        argv.push_back(nullptr);
        pid_t pid = 0;
        const int rc = posix_spawnp(&pid, py, nullptr, nullptr, argv.data(), environ);
        if (rc != 0) {
            std::cerr << "[lerobot] failed to start cameras: " << std::strerror(rc) << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(record_mutex_);
        camera_pid_ = pid;
        std::cout << "[lerobot] cameras -> " << out << std::endl;
    }

    static void stop_camera_process(pid_t pid) {
        if (pid <= 0) {
            return;
        }
        ::kill(pid, SIGINT);
        int status = 0;
        for (int i = 0; i < 30; ++i) {
            const pid_t got = ::waitpid(pid, &status, WNOHANG);
            if (got == pid || got < 0) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }

    void close_lerobot_episode_locked() {
        if (!lerobot_csv_.is_open()) {
            return;
        }
        lerobot_csv_.flush();
        lerobot_csv_.close();
        if (lerobot_rows_ > 0) {
            lerobot_awaiting_label_ = true;
        }
    }

    bool recording_active() const {
        return recorder_left.active() || recorder_right.active();
    }

    void write_episode_outcome(const std::string& label) {
        if (recording_active()) {
            std::cerr << "[quest-ui] stop the take before labeling it" << std::endl;
            return;
        }
        if (!lerobot_awaiting_label_ || active_session_dir.empty()) {
            std::cerr << "[quest-ui] no take is waiting for success, fail, or redo" << std::endl;
            return;
        }
        const std::string path = active_session_dir + "/outcome.txt";
        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out) {
            std::cerr << "[quest-ui] failed to write " << path << std::endl;
            return;
        }
        out << label << "\n";
        lerobot_awaiting_label_ = false;
        lerobot_pending_export_ = true;
        std::cout << "[quest-ui] " << active_session_dir << " labeled " << label << std::endl;
        export_lerobot_dataset();
    }

    void discard_last_episode() {
        if (recording_active()) {
            std::cerr << "[quest-ui] stop the take before re-recording it" << std::endl;
            return;
        }
        if (active_session_dir.empty()) {
            std::cerr << "[quest-ui] no take to discard" << std::endl;
            return;
        }
        namespace fs = std::filesystem;
        const fs::path original(active_session_dir);
        const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
        const fs::path dest = fs::path(sessions_root).parent_path() / "trash" /
                              original.parent_path().filename() /
                              (original.filename().string() + "_" + std::to_string(millis));
        std::error_code ec;
        fs::create_directories(dest.parent_path(), ec);
        if (ec) {
            std::cerr << "[quest-ui] failed to create " << dest.parent_path() << ": "
                      << ec.message() << std::endl;
            return;
        }
        ec.clear();
        fs::rename(original, dest, ec);
        if (ec) {
            std::cerr << "[quest-ui] failed to discard " << active_session_dir << ": " << ec.message()
                      << std::endl;
            return;
        }
        trashed_take_ = TrashedTake{
            original.string(),
            dest.string(),
            segment,
            lerobot_awaiting_label_,
            std::chrono::steady_clock::now() + std::chrono::milliseconds(10500),
        };
        std::cout << "[quest-ui] moved " << active_session_dir << " to " << dest.string()
                  << " — undo within 10s" << std::endl;
        active_session_dir.clear();
        active_left_csv.clear();
        active_right_csv.clear();
        lerobot_awaiting_label_ = false;
        if (segment > 0) {
            --segment;
        }
        lerobot_pending_export_ = !record_dir.empty();
        export_lerobot_dataset();
    }

    void undo_last_discard() {
        if (recording_active()) {
            std::cerr << "[quest-ui] stop the take before undoing a discard" << std::endl;
            return;
        }
        if (!trashed_take_) {
            std::cerr << "[quest-ui] no discarded take to restore" << std::endl;
            return;
        }
        if (std::chrono::steady_clock::now() > trashed_take_->until) {
            std::cerr << "[quest-ui] undo window expired for " << trashed_take_->original
                      << std::endl;
            trashed_take_.reset();
            return;
        }
        namespace fs = std::filesystem;
        std::error_code ec;
        if (fs::exists(trashed_take_->original, ec)) {
            std::cerr << "[quest-ui] cannot undo; " << trashed_take_->original
                      << " already exists" << std::endl;
            trashed_take_.reset();
            return;
        }
        ec.clear();
        fs::create_directories(fs::path(trashed_take_->original).parent_path(), ec);
        ec.clear();
        fs::rename(trashed_take_->trashed, trashed_take_->original, ec);
        if (ec) {
            std::cerr << "[quest-ui] failed to restore " << trashed_take_->trashed << ": "
                      << ec.message() << std::endl;
            return;
        }
        active_session_dir = trashed_take_->original;
        active_left_csv =
            active_session_dir + "/real_teleop_split/left/real_teleop_left.csv";
        active_right_csv =
            active_session_dir + "/real_teleop_split/right/real_teleop_right.csv";
        segment = trashed_take_->segment_number;
        lerobot_awaiting_label_ = trashed_take_->awaiting_label;
        std::cout << "[quest-ui] restored " << active_session_dir << std::endl;
        trashed_take_.reset();
        lerobot_pending_export_ = !record_dir.empty();
        export_lerobot_dataset();
    }

    // Reclaim episode number NN for the next start (segment becomes N-1).
    // arg: "<NN>" or "<batch>,<NN>"
    void rerecord_episode(const std::string& raw_arg) {
        if (recording_active()) {
            std::cerr << "[quest-ui] stop the take before re-recording" << std::endl;
            return;
        }
        if (lerobot_awaiting_label_) {
            std::cerr << "[quest-ui] label or redo the pending take before rerecord,<NN>"
                      << std::endl;
            return;
        }
        std::string batch_arg;
        std::string ep_arg = raw_arg;
        {
            const auto comma = raw_arg.find(',');
            if (comma != std::string::npos) {
                batch_arg = raw_arg.substr(0, comma);
                ep_arg = raw_arg.substr(comma + 1);
                auto trim = [](std::string& s) {
                    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
                        s.erase(s.begin());
                    }
                    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
                        s.pop_back();
                    }
                };
                trim(batch_arg);
                trim(ep_arg);
            }
        }
        if (!batch_arg.empty()) {
            if (!ensure_batch_dir(batch_arg, /*ensure_dirs=*/true)) {
                return;
            }
        } else if (record_dir.empty()) {
            std::cerr << "[quest-ui] rerecord needs an active batch "
                         "(start once, or send rerecord,<batch>,<NN>)"
                      << std::endl;
            return;
        }
        int n = 0;
        try {
            n = std::stoi(ep_arg);
        } catch (...) {
            std::cerr << "[quest-ui] rerecord: invalid episode id " << ep_arg << std::endl;
            return;
        }
        if (n <= 0) {
            std::cerr << "[quest-ui] rerecord: episode id must be >= 1" << std::endl;
            return;
        }
        char seg[16];
        std::snprintf(seg, sizeof(seg), "%02d", n);
        const std::string path_a = record_dir + "/" + seg;
        const std::string path_b = record_dir + "/" + std::to_string(n);
        std::error_code ec;
        bool removed = false;
        if (std::filesystem::exists(path_a, ec)) {
            std::filesystem::remove_all(path_a, ec);
            if (ec) {
                std::cerr << "[quest-ui] failed to remove " << path_a << ": " << ec.message()
                          << std::endl;
                return;
            }
            removed = true;
        }
        ec.clear();
        if (std::filesystem::exists(path_b, ec) && path_b != path_a) {
            std::filesystem::remove_all(path_b, ec);
            if (ec) {
                std::cerr << "[quest-ui] failed to remove " << path_b << ": " << ec.message()
                          << std::endl;
                return;
            }
            removed = true;
        }
        if (!removed) {
            std::cout << "[quest-ui] rerecord: no folder for " << seg
                      << " (will reuse number on next start)" << std::endl;
        } else {
            std::cout << "[quest-ui] rerecord: removed episode " << seg << " under " << record_dir
                      << std::endl;
        }
        if (!active_session_dir.empty() &&
            (active_session_dir == path_a || active_session_dir == path_b)) {
            active_session_dir.clear();
            active_left_csv.clear();
            active_right_csv.clear();
        }
        segment = n - 1;
        lerobot_awaiting_label_ = false;
        lerobot_pending_export_ = !record_dir.empty();
        export_lerobot_dataset();
        std::cout << "[quest-ui] next start will reuse segment " << seg << std::endl;
    }

    void export_lerobot_dataset() {
        if (!lerobot_pending_export_ || record_dir.empty()) {
            lerobot_pending_export_ = false;
            return;
        }
        lerobot_pending_export_ = false;
        const char* flag = std::getenv("OPENARM_LEROBOT");
        if (flag != nullptr) {
            const std::string s(flag);
            if (s == "0" || s == "false" || s == "off" || s == "no") {
                std::cout << "[lerobot] export skipped (OPENARM_LEROBOT=" << s << ")"
                          << std::endl;
                return;
            }
        }
        // OpenArm Dataset v0.4 (docs.openarm.dev/dataset) then LeRobot convert.
        std::string script = lerobot_script("openarm_dataset_export.py");
        if (!std::filesystem::is_regular_file(script)) {
            script = lerobot_script("lerobot_collect.py");
        }
        if (!std::filesystem::is_regular_file(script)) {
            std::cerr << "[lerobot] export script not found; episode CSV kept under "
                      << record_dir << std::endl;
            return;
        }
        const char* py = std::getenv("OPENARM_LEROBOT_PYTHON");
        if (py == nullptr || py[0] == '\0') {
            py = "python3";
        }
        const char* task = std::getenv("OPENARM_LEROBOT_TASK");
        if (task == nullptr || task[0] == '\0') {
            task = "bimanual teleoperation";
        }
        const char* fps = std::getenv("OPENARM_LEROBOT_FPS");
        if (fps == nullptr || fps[0] == '\0') {
            fps = "30";
        }
        const char* repo = std::getenv("OPENARM_LEROBOT_REPO");
        if (repo == nullptr || repo[0] == '\0') {
            repo = "local/openarm_vr_teleop";
        }
        const std::string batch = record_dir;
        std::vector<char*> argv;
        std::vector<std::string> storage = {
            py, script, "--batch", batch, "--task", task, "--fps", fps, "--repo-id", repo};
        argv.reserve(storage.size() + 1);
        for (auto& item : storage) {
            argv.push_back(item.data());
        }
        argv.push_back(nullptr);
        pid_t pid = 0;
        const int rc = posix_spawnp(&pid, py, nullptr, nullptr, argv.data(), environ);
        if (rc != 0) {
            std::cerr << "[lerobot] failed to start " << py << ": " << std::strerror(rc)
                      << std::endl;
            return;
        }
        int status = 0;
        if (waitpid(pid, &status, 0) < 0) {
            std::cerr << "[lerobot] waitpid failed" << std::endl;
            return;
        }
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            std::cerr << "[lerobot] export failed for " << batch << std::endl;
            return;
        }
        std::cout << "[lerobot] OpenArm dataset " << batch << "/openarm_dataset"
                  << " → " << batch << "/lerobot" << std::endl;
    }

    void close_recorders() {
        pid_t cam = -1;
        bool attached = false;
        {
            std::lock_guard<std::mutex> lock(record_mutex_);
            recorder_left.close();
            recorder_right.close();
            close_lerobot_episode_locked();
            cam = camera_pid_;
            camera_pid_ = -1;
            attached = camera_view_attached_;
            camera_view_attached_ = false;
        }
        if (attached && !camera_view_command("release")) {
            std::cerr << "[lerobot] camera view did not acknowledge release" << std::endl;
        }
        stop_camera_process(cam);
    }

    void finalize_recording_if_needed() {
        const bool was_recording = recorder_left.active() || recorder_right.active();
        close_recorders();
        if (was_recording) {
            merge_active_session_for_replay();
        }
        export_lerobot_dataset();
    }

    ~QuestUiBridge() { finalize_recording_if_needed(); }

    void merge_active_session_for_replay() {
        if (active_left_csv.empty() || active_right_csv.empty() || active_session_dir.empty()) {
            return;
        }
        namespace fs = std::filesystem;
        if (!fs::is_regular_file(active_left_csv) || !fs::is_regular_file(active_right_csv)) {
            std::cerr << "[quest-ui] skip merge: missing left/right CSV" << std::endl;
            return;
        }
        // Header-only files also fail merge ("not enough valid frames").
        auto row_count = [](const std::string& path) -> std::size_t {
            std::ifstream in(path);
            std::size_t n = 0;
            std::string line;
            while (std::getline(in, line)) {
                ++n;
            }
            return n;
        };
        const auto left_rows = row_count(active_left_csv);
        const auto right_rows = row_count(active_right_csv);
        if (left_rows < 3 || right_rows < 3) {
            std::cerr << "[quest-ui] skip merge: need >=2 data rows per arm (got left_lines="
                      << left_rows << " right_lines=" << right_rows
                      << "). Split CSVs kept; re-record after rebuild if one side is empty."
                      << std::endl;
            return;
        }
        const std::string merged = active_session_dir + "/real_teleop_dual_merged.csv";
        // Resolve the merge helper from the bundled controller working directory.
        const std::string merge_py = "script/merge_dual_real_recordings.py";
        const std::string cmd = "python3 \"" + merge_py + "\" --left \"" + active_left_csv +
                                "\" --right \"" + active_right_csv + "\" --output \"" + merged +
                                "\" --crop min";
        std::cout << "[quest-ui] merging dual replay CSV..." << std::endl;
        const int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::cerr << "[quest-ui] merge failed (rc=" << rc << "). Split CSVs are still at:\n  "
                      << active_left_csv << "\n  " << active_right_csv << std::endl;
            return;
        }
        std::cout << "[quest-ui] replay-ready session: " << active_session_dir << std::endl;
        std::cout << "  merged : " << merged << std::endl;
        std::cout << "  replay : .venv/bin/python script/replay_teaching.py \"" << active_session_dir
                  << "\" --sim\n  or console: data management -> replay this episode" << std::endl;
    }

    static Eigen::Vector3d jog_delta(const std::string& dir, double step) {
        // Robot body frame (openarm_body_link0): +Z up, +Y left, +X forward.
        if (dir == "x+" || dir == "+x" || dir == "forward") {
            return {step, 0.0, 0.0};
        }
        if (dir == "x-" || dir == "-x" || dir == "back") {
            return {-step, 0.0, 0.0};
        }
        if (dir == "y+" || dir == "+y" || dir == "left") {
            return {0.0, step, 0.0};
        }
        if (dir == "y-" || dir == "-y" || dir == "right") {
            return {0.0, -step, 0.0};
        }
        if (dir == "z+" || dir == "+z" || dir == "up") {
            return {0.0, 0.0, step};
        }
        if (dir == "z-" || dir == "-z" || dir == "down") {
            return {0.0, 0.0, -step};
        }
        return Eigen::Vector3d::Zero();
    }

    static Eigen::Vector3d jog_omega(const std::string& dir, double rate) {
        // Robot body frame rotation of the TCP (same axes as X/Y/Z translation).
        if (dir == "rx+" || dir == "+rx") {
            return {rate, 0.0, 0.0};
        }
        if (dir == "rx-" || dir == "-rx") {
            return {-rate, 0.0, 0.0};
        }
        if (dir == "ry+" || dir == "+ry") {
            return {0.0, rate, 0.0};
        }
        if (dir == "ry-" || dir == "-ry") {
            return {0.0, -rate, 0.0};
        }
        if (dir == "rz+" || dir == "+rz") {
            return {0.0, 0.0, rate};
        }
        if (dir == "rz-" || dir == "-rz") {
            return {0.0, 0.0, -rate};
        }
        return Eigen::Vector3d::Zero();
    }

    // Explicit body-frame axes (not up×forward): OPENARM_HM_FORWARD_SIGN must not
    // silently swap panel Left/Right.
    static Eigen::Vector3d velocity_from_dirs(HumanMimicController* /*hm*/,
                                             const std::set<std::string>& dirs, double speed) {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        for (const auto& d : dirs) {
            v += jog_delta(d, 1.0);
        }
        const double n = v.norm();
        if (n < 1e-12) {
            return Eigen::Vector3d::Zero();
        }
        return (v / n) * speed;
    }

    static Eigen::Vector3d omega_from_dirs(const std::set<std::string>& dirs, double rate) {
        Eigen::Vector3d w = Eigen::Vector3d::Zero();
        for (const auto& d : dirs) {
            w += jog_omega(d, 1.0);
        }
        const double n = w.norm();
        if (n < 1e-12) {
            return Eigen::Vector3d::Zero();
        }
        return (w / n) * rate;
    }

    static void apply_panel_command(ArmTeleopBundle* arm, const Eigen::Vector3d& vel,
                                    const Eigen::Vector3d& omega) {
        if (arm == nullptr) {
            return;
        }
        if (arm->human_mimic != nullptr) {
            arm->human_mimic->set_panel_jog_velocity(vel);
            arm->human_mimic->set_panel_jog_angular_velocity(omega);
        }
    }

    void apply_nudge(ArmTeleopBundle* arm, const std::string& dir) {
        if (arm == nullptr) {
            return;
        }
        const Eigen::Vector3d d = jog_delta(dir, jog_step_m);
        if (!d.allFinite() || d.norm() <= 1e-12) {
            return;
        }
        if (arm->human_mimic != nullptr) {
            arm->human_mimic->nudge_tcp_robot(d);
            return;
        }
    }

    void push_jog_velocities() {
        if (couple_active) {
            // Left is slave — ignore left panel holds while coupled.
            jog_dirs_left.clear();
        }
        if (circle_active) {
            // Circle owns right tip — ignore right panel holds.
            jog_dirs_right.clear();
        }
        const Eigen::Vector3d vl =
            velocity_from_dirs(left != nullptr ? left->human_mimic.get() : nullptr, jog_dirs_left,
                               jog_speed_mps);
        const Eigen::Vector3d vr =
            velocity_from_dirs(right != nullptr ? right->human_mimic.get() : nullptr,
                               jog_dirs_right, jog_speed_mps);
        const Eigen::Vector3d wl = omega_from_dirs(jog_dirs_left, jog_omega_rps);
        const Eigen::Vector3d wr = omega_from_dirs(jog_dirs_right, jog_omega_rps);
        apply_panel_command(left, couple_active ? Eigen::Vector3d::Zero() : vl,
                            couple_active ? Eigen::Vector3d::Zero() : wl);
        apply_panel_command(right, vr, wr);
    }

    void dump_pose() {
        // Hand-eye / FK consumers need the motor-measured pose, not q_target.
        // human_mimic q_target carries steady-state tracking error vs encoder feedback.
        std::ostringstream line;
        line << std::fixed << std::setprecision(6) << "[positions]";
        auto append_measured = [&](const char* name, ArmTeleopBundle* arm) {
            line << " " << name << "=[";
            if (arm != nullptr && arm->state != nullptr) {
                const auto joints = collect_joint_states(arm->state);
                // Arm joints only (exclude gripper) to match historical left=/right= dumps.
                const size_t n = std::min<size_t>(joints.size(), 7);
                for (size_t i = 0; i < n; ++i) {
                    if (i) {
                        line << " ";
                    }
                    line << joints[i].position;
                }
            }
            line << "]";
        };
        auto append_target = [&](const char* name, ArmTeleopBundle* arm) {
            line << " " << name << "=[";
            if (arm != nullptr && arm->human_mimic != nullptr) {
                const auto& q = arm->human_mimic->q_target();
                for (size_t i = 0; i < q.size(); ++i) {
                    if (i) {
                        line << " ";
                    }
                    line << q[i];
                }
            }
            line << "]";
        };
        append_measured("left", left);
        append_measured("right", right);
        // Optional debug fields; existing parsers only read left=/right=.
        append_target("left_target", left);
        append_target("right_target", right);
        const std::string text = line.str();
        std::cout << text << std::endl;
        // Print max |q_meas - q_target| so hand-eye operators can see tracking error.
        auto max_track_err = [](ArmTeleopBundle* arm) -> double {
            if (arm == nullptr || arm->state == nullptr || arm->human_mimic == nullptr) {
                return 0.0;
            }
            const auto joints = collect_joint_states(arm->state);
            const auto& q_tgt = arm->human_mimic->q_target();
            const size_t n = std::min(std::min(joints.size(), q_tgt.size()), size_t{7});
            double m = 0.0;
            for (size_t i = 0; i < n; ++i) {
                m = std::max(m, std::abs(joints[i].position - q_tgt[i]));
            }
            return m;
        };
        std::cout << "[positions][track_err] left_max=" << max_track_err(left)
                  << " right_max=" << max_track_err(right) << " rad" << std::endl;
        std::ofstream out(dump_path, std::ios::app);
        if (out) {
            out << text << "\n";
        }
    }

    void handle_record_msg(const std::string& msg) {
        // Protocols: start | start,<name> | stop | success | fail | redo | undo | export | dump | shutdown
        std::string cmd;
        std::string arg;
        {
            const auto comma = msg.find(',');
            if (comma == std::string::npos) {
                cmd = msg;
            } else {
                cmd = msg.substr(0, comma);
                arg = msg.substr(comma + 1);
            }
            auto trim = [](std::string& s) {
                while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
                    s.erase(s.begin());
                }
                while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
                    s.pop_back();
                }
            };
            trim(cmd);
            trim(arg);
            for (char& c : cmd) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
        }

        if (cmd == "shutdown" || cmd == "exit" || cmd == "quit") {
            if (recorder_left.active() || recorder_right.active()) {
                close_recorders();
                merge_active_session_for_replay();
            }
            disengage_couple();
            std::cout << "[quest-ui] shutdown requested — exiting teleop after damping release"
                      << std::endl;
            keep_running = false;
            return;
        }

        if (cmd == "start") {
            if (recording_active()) {
                std::cerr << "[quest-ui] already recording; stop before starting another take"
                          << std::endl;
                return;
            }
            if (lerobot_awaiting_label_) {
                std::cerr << "[quest-ui] label the last take (success, fail, or redo) before "
                             "starting another"
                          << std::endl;
                return;
            }
            if (!ensure_batch_dir(arg, /*ensure_dirs=*/true)) {
                return;
            }
            ++segment;
            char seg[16];
            std::snprintf(seg, sizeof(seg), "%02d", segment);
            active_session_dir = record_dir + "/" + seg;
            active_left_csv =
                active_session_dir + "/real_teleop_split/left/real_teleop_left.csv";
            active_right_csv =
                active_session_dir + "/real_teleop_split/right/real_teleop_right.csv";
            std::error_code ec;
            std::filesystem::create_directories(
                std::filesystem::path(active_left_csv).parent_path(), ec);
            std::filesystem::create_directories(
                std::filesystem::path(active_right_csv).parent_path(), ec);
            const size_t jl = (left != nullptr && left->human_mimic != nullptr)
                                  ? left->human_mimic->arm_dof() + 1
                                  : 8;
            const size_t jr = (right != nullptr && right->human_mimic != nullptr)
                                  ? right->human_mimic->arm_dof() + 1
                                  : 8;
            bool opened = false;
            {
                std::lock_guard<std::mutex> lock(record_mutex_);
                first_meas_left_.clear();
                first_meas_right_.clear();
                joint_moved_left_.clear();
                joint_moved_right_.clear();
                stuck_warn_left_ = false;
                stuck_warn_right_ = false;
                if (left != nullptr) {
                    opened = recorder_left.open(active_left_csv, jl) || opened;
                }
                if (right != nullptr) {
                    opened = recorder_right.open(active_right_csv, jr) || opened;
                }
            }
            if (!opened) {
                std::cerr << "[quest-ui] failed to open recorders under " << active_session_dir
                          << std::endl;
                active_session_dir.clear();
                active_left_csv.clear();
                active_right_csv.clear();
                return;
            }
            std::cout << "[quest-ui] recording segment " << seg << " -> " << active_session_dir
                      << std::endl;
            std::cout << "  left =" << active_left_csv << std::endl;
            std::cout << "  right=" << active_right_csv << std::endl;

            std::cout << "  source=MIT command q/dq @ " << FREQUENCY
                      << " Hz (one sample per unilateral_step send)" << std::endl;
            open_lerobot_episode();
        } else if (cmd == "stop") {
            if (!recording_active()) {
                std::cerr << "[quest-ui] not recording" << std::endl;
                return;
            }
            close_recorders();
            std::cout << "[quest-ui] recording stopped" << std::endl;
            merge_active_session_for_replay();
            if (lerobot_awaiting_label_) {
                std::cout << "[quest-ui] choose success, fail, or redo for " << active_session_dir
                          << std::endl;
            }
        } else if (cmd == "success" || cmd == "ok") {
            write_episode_outcome("success");
        } else if (cmd == "fail" || cmd == "failure") {
            write_episode_outcome("failure");
        } else if (cmd == "redo" || cmd == "discard") {
            discard_last_episode();
        } else if (cmd == "undo") {
            undo_last_discard();
        } else if (cmd == "rerecord") {
            if (arg.empty()) {
                discard_last_episode();
            } else {
                rerecord_episode(arg);
            }
        } else if (cmd == "export") {
            lerobot_pending_export_ = !record_dir.empty();
            export_lerobot_dataset();
        } else if (cmd == "dump") {
            if (!ensure_batch_dir(arg.empty() && !record_dir.empty()
                                     ? std::filesystem::path(record_dir).filename().string()
                                     : arg,
                                 /*ensure_dirs=*/true)) {
                return;
            }
            dump_pose();
        }
    }

    static bool is_jog_dir(const std::string& d) {
        return d == "x+" || d == "x-" || d == "+x" || d == "-x" || d == "forward" ||
               d == "back" || d == "y+" || d == "y-" || d == "+y" || d == "-y" ||
               d == "left" || d == "right" || d == "z+" || d == "z-" || d == "+z" ||
               d == "-z" || d == "up" || d == "down" || d == "rx+" || d == "rx-" ||
               d == "+rx" || d == "-rx" || d == "ry+" || d == "ry-" || d == "+ry" ||
               d == "-ry" || d == "rz+" || d == "rz-" || d == "+rz" || d == "-rz";
    }

    void set_dir(std::set<std::string>* dirs, const std::string& dir, bool on) {
        if (dirs == nullptr || !is_jog_dir(dir)) {
            return;
        }
        if (on) {
            dirs->insert(dir);
        } else {
            dirs->erase(dir);
        }
    }

    void handle_jog_msg(const std::string& msg) {
        // hold,left,x+|y+|z+|rx+|ry+|rz+ | release,left,rx- | nudge,right,z+
        // Legacy aliases: up/down/left/right still map to +Z/-Z/+Y/-Y.
        // circle,start,<radius_mm> | circle,stop
        std::string s = msg;
        for (char& c : s) {
            if (c == ' ') {
                c = ',';
            }
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        std::vector<std::string> parts;
        std::stringstream ss(s);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) {
                parts.push_back(tok);
            }
        }
        if (parts.empty()) {
            return;
        }
        if (parts[0] == "couple") {
            const std::string mode = parts.size() >= 2 ? parts[1] : "toggle";
            if (mode == "on" || mode == "1" || mode == "true") {
                engage_couple();
            } else if (mode == "off" || mode == "0" || mode == "false") {
                disengage_couple();
            } else {
                if (couple_active) {
                    disengage_couple();
                } else {
                    engage_couple();
                }
            }
            return;
        }
        if (parts[0] == "circle") {
            const std::string mode = parts.size() >= 2 ? parts[1] : "stop";
            if (mode == "stop" || mode == "off" || mode == "0") {
                stop_circle("panel");
                return;
            }
            if (mode == "start" || mode == "on" || mode == "1") {
                double radius_mm = 50.0;
                if (parts.size() >= 3) {
                    try {
                        radius_mm = std::stod(parts[2]);
                    } catch (...) {
                        std::cerr << "[quest-ui] circle start: bad radius '" << parts[2] << "'"
                                  << std::endl;
                        return;
                    }
                }
                start_circle(radius_mm);
                return;
            }
            std::cerr << "[quest-ui] circle: expected start,<mm> or stop" << std::endl;
            return;
        }
        if (parts.size() < 3) {
            return;
        }
        const std::string& cmd = parts[0];
        const std::string& side = parts[1];
        const std::string& dir = parts[2];

        if (cmd == "hold" || cmd == "release") {
            const bool on = (cmd == "hold");
            if (side == "left" || side == "l") {
                set_dir(&jog_dirs_left, dir, on);
            } else if (side == "right" || side == "r") {
                set_dir(&jog_dirs_right, dir, on);
            } else if (side == "both") {
                set_dir(&jog_dirs_left, dir, on);
                set_dir(&jog_dirs_right, dir, on);
            }
            std::cout << "[quest-ui] jog " << cmd << " " << side << " " << dir << std::endl;
            return;
        }

        if (cmd != "nudge" || !is_jog_dir(dir)) {
            return;
        }
        if (side == "left" || side == "l") {
            apply_nudge(left, dir);
        } else if (side == "right" || side == "r") {
            apply_nudge(right, dir);
        } else if (side == "both") {
            apply_nudge(left, dir);
            apply_nudge(right, dir);
        }
    }

    void poll() {
        for (const auto& m : record_rx.receive_all()) {
            handle_record_msg(m);
        }
        for (const auto& m : jog_rx.receive_all()) {
            handle_jog_msg(m);
        }
        push_jog_velocities();
    }

    void append_recording(ArmTeleopBundle* arm, const std::vector<JointState>& /*reference*/) {
        // Legacy hook from the Quest/IK reference thread — recording now happens in
        // the 1000 Hz follower MIT thread (append_motor_sample).
        (void)arm;
    }

    // Called once per arm after unilateral_step() @ FREQUENCY (1000 Hz).
    // Saves the MIT command frame actually sent (post safety / gripper hold).
    void append_motor_sample(ArmTeleopBundle* arm) {
        if (arm == nullptr || arm->control == nullptr) return;
        std::vector<JointState> frame = arm->control->GetLastMitCommand();
        if (frame.empty()) {
            return;
        }

        append_frame(arm, frame);
    }

    // Mirror teaching supplies its actual left command and measured right demonstration.
    void append_frame(ArmTeleopBundle* arm, const std::vector<JointState>& frame) {
        if (arm == nullptr || frame.empty()) return;
        std::lock_guard<std::mutex> lock(record_mutex_);
        CsvRecorder* rec = arm->is_left ? &recorder_left : &recorder_right;
        if (!rec->active()) return;
        // Stuck-joint detector on commanded positions (frozen while others move).
        auto& first = arm->is_left ? first_meas_left_ : first_meas_right_;
        auto& moved = arm->is_left ? joint_moved_left_ : joint_moved_right_;
        bool& stuck_warned = arm->is_left ? stuck_warn_left_ : stuck_warn_right_;
        if (first.size() != frame.size()) {
            first = frame;
            moved.assign(frame.size(), 0.0);
        } else {
            double arm_span = 0.0;
            for (size_t i = 0; i < frame.size() && i < 7; ++i) {
                const double span = std::abs(frame[i].position - first[i].position);
                moved[i] = std::max(moved[i], span);
                arm_span = std::max(arm_span, moved[i]);
            }
            if (!stuck_warned && arm_span > 0.15) {
                for (size_t i = 0; i < frame.size() && i < 7; ++i) {
                    if (moved[i] < 1e-3) {
                        std::cerr << "[quest-ui][WARN] " << (arm->is_left ? "left" : "right")
                                  << " MIT cmd q" << i << " stuck near " << frame[i].position
                                  << " while other joints moved — check encoder/CAN"
                                  << std::endl;
                        stuck_warned = true;
                    }
                }
            }
        }
        rec->append(frame);

        // LeRobot sample: measured qpos is observation.state, MIT q is action.
        // Written at 30 Hz once both arms present in this tick window have reported.
        if (lerobot_csv_.is_open() && arm->state) {
            double* obs = arm->is_left ? lerobot_obs_l_ : lerobot_obs_r_;
            double* act = arm->is_left ? lerobot_act_l_ : lerobot_act_r_;
            copy_qpos8(collect_joint_states(arm->state), obs);
            copy_qpos8(frame, act);
            if (arm->is_left) {
                lerobot_got_l_ = true;
            } else {
                lerobot_got_r_ = true;
            }
            const bool need_l = left != nullptr;
            const bool need_r = right != nullptr;
            if ((!need_l || lerobot_got_l_) && (!need_r || lerobot_got_r_)) {
                const auto now = std::chrono::steady_clock::now();
                if (lerobot_next_.time_since_epoch().count() == 0 || now >= lerobot_next_) {
                    const double t =
                        std::chrono::duration<double>(now - lerobot_t0_).count();
                    lerobot_csv_ << t;
                    for (int i = 0; i < 8; ++i) {
                        lerobot_csv_ << "," << lerobot_obs_l_[i];
                    }
                    for (int i = 0; i < 8; ++i) {
                        lerobot_csv_ << "," << lerobot_act_l_[i];
                    }
                    for (int i = 0; i < 8; ++i) {
                        lerobot_csv_ << "," << lerobot_obs_r_[i];
                    }
                    for (int i = 0; i < 8; ++i) {
                        lerobot_csv_ << "," << lerobot_act_r_[i];
                    }
                    const double unix_s = std::chrono::duration<double>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count();
                    lerobot_csv_ << "," << unix_s << "\n";
                    ++lerobot_rows_;
                    lerobot_next_ = now + std::chrono::milliseconds(33);
                }
            }
        }
    }
};

bool prime_hardware_arm(ArmTeleopBundle* arm, size_t arm_dof, size_t hand_dof) {
    // Always re-seed hold to measured before stepping — default references are
    // zeros and would yank the forearm on the first MIT frame.
    if (arm == nullptr || arm->control == nullptr) {
        return false;
    }
    if (!arm->control->SeedHoldMeasured()) {
        std::cerr << "[quest-cartesian][ERROR] " << arm->label
                  << " no fresh CAN feedback; not entering the torque loop" << std::endl;
        keep_running = false;
        return false;
    }
    for (int i = 0; i < 5; ++i) {
        if (!arm->control->unilateral_step()) {
            std::cerr << "[quest-cartesian][ERROR] " << arm->label
                      << " follower step stopped before the torque loop: "
                      << arm->control->OutputDisabledReason() << std::endl;
            keep_running = false;
            return false;
        }
    }
    const auto current = collect_joint_states(arm->state);
    arm->state->set_all_references(current);
    arm->control->ResetSafetyFilter(current);
    (void)arm_dof;
    (void)hand_dof;
    return true;
}

void prime_sim_arm(ArmTeleopBundle* arm, size_t arm_dof, size_t hand_dof, bool table_ready) {
    if (table_ready && arm->sim_hardware) {
        const std::vector<double> q_arm = table_ready_arm_pose();
        const double q_grip = TABLE_READY_POSITION[7];
        arm->sim_hardware->set_home(q_arm, q_grip);

        std::vector<JointState> ready(arm_dof + hand_dof);
        for (size_t i = 0; i < arm_dof && i < q_arm.size(); ++i) {
            ready[i].position = q_arm[i];
            ready[i].velocity = 0.0;
            ready[i].effort = 0.0;
        }
        if (hand_dof > 0 && arm_dof < ready.size()) {
            ready[arm_dof].position = q_grip;
            ready[arm_dof].velocity = 0.0;
            ready[arm_dof].effort = 0.0;
        }
        arm->state->set_all_references(ready);
        std::cout << "  [" << arm->label << "] sim table-ready pose (elbow≈90°)" << std::endl;
    }
    for (int i = 0; i < 5; ++i) {
        arm->control->unilateral_step();
    }
    const auto current = collect_joint_states(arm->state);
    arm->state->set_all_references(current);
    arm->control->ResetSafetyFilter(current);
}

// ---------------------------------------------------------------------------
// Configurable startup motion (smoothly move arms to a pose / play a trajectory)
// ---------------------------------------------------------------------------
//
// File format (whitespace or comma separated, '#' comments, blank lines ignored):
//   [t]  side  q0 q1 q2 q3 q4 q5 q6  [grip]
//   - side : left | right | both
//   - t    : OPTIONAL leading time in seconds. If present on any row, rows are
//            played at those times; if absent, rows are spaced evenly over
//            --startup-move-sec. A single row per side => just move to that pose.
//   - grip : OPTIONAL 8th value (gripper). If omitted, the gripper is held.
// Example (target pose, both arms):   both 0 0 0 1.5708 0 0 0 0
struct StartupWaypoint {
    bool has_t = false;
    double t = 0.0;
    std::string side = "both";
    std::vector<double> q;  // 7 arm joints
    bool has_grip = false;
    double grip = 0.0;
};

bool parse_startup_line(const std::string& raw, StartupWaypoint* wp, std::string* err) {
    std::string line = raw;
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
        line = line.substr(0, hash);
    }
    for (char& c : line) {
        if (c == ',') {
            c = ' ';
        }
    }
    std::istringstream iss(line);
    std::vector<std::string> tok;
    std::string t;
    while (iss >> t) {
        tok.push_back(t);
    }
    if (tok.empty()) {
        return false;  // blank/comment: skip (not an error)
    }

    auto is_side = [](const std::string& s) {
        return s == "left" || s == "right" || s == "both";
    };
    auto to_num = [](const std::string& s, double* out) {
        try {
            size_t pos = 0;
            *out = std::stod(s, &pos);
            return pos == s.size();
        } catch (...) {
            return false;
        }
    };

    size_t idx = 0;
    // Optional leading time.
    if (!is_side(tok[0])) {
        double v = 0.0;
        if (to_num(tok[0], &v) && tok.size() > 1 && is_side(tok[1])) {
            wp->has_t = true;
            wp->t = v;
            idx = 1;
        }
    }
    // Side (default both if the row is pure numbers).
    if (idx < tok.size() && is_side(tok[idx])) {
        wp->side = tok[idx];
        ++idx;
    } else {
        wp->side = "both";
    }
    // Remaining tokens are joint values (+ optional grip).
    std::vector<double> nums;
    for (; idx < tok.size(); ++idx) {
        double v = 0.0;
        if (!to_num(tok[idx], &v)) {
            *err = "invalid number '" + tok[idx] + "'";
            return false;
        }
        nums.push_back(v);
    }
    if (nums.size() != 7 && nums.size() != 8) {
        *err = "expected 7 (arm) or 8 (arm+grip) joint values, got " +
               std::to_string(nums.size());
        return false;
    }
    wp->q.assign(nums.begin(), nums.begin() + 7);
    if (nums.size() == 8) {
        wp->has_grip = true;
        wp->grip = nums[7];
    }
    return true;
}

bool load_startup_waypoints(const std::string& path, std::vector<StartupWaypoint>* out,
                            std::string* err) {
    std::ifstream file(path);
    if (!file.good()) {
        *err = "cannot open startup file: " + path;
        return false;
    }
    std::string line;
    int lineno = 0;
    while (std::getline(file, line)) {
        ++lineno;
        StartupWaypoint wp;
        std::string line_err;
        if (parse_startup_line(line, &wp, &line_err)) {
            out->push_back(wp);
        } else if (!line_err.empty()) {
            *err = "line " + std::to_string(lineno) + ": " + line_err;
            return false;
        }
    }
    if (out->empty()) {
        *err = "no waypoints parsed from " + path;
        return false;
    }
    return true;
}

// Timed per-arm waypoint (arm joints + gripper), sorted by time.
struct TimedPose {
    double t = 0.0;
    std::vector<double> q;  // arm joints
    double grip = 0.0;
};

// Build a timed sequence for one arm: starts from the current measured pose at
// t=0, then the file waypoints applicable to this arm (side match or "both").
std::vector<TimedPose> build_arm_sequence(const std::string& arm_label,
                                          const std::vector<StartupWaypoint>& wps,
                                          const std::vector<double>& q_start,
                                          double grip_start, double move_sec, Dynamics* dyn) {
    std::vector<StartupWaypoint> mine;
    for (const auto& w : wps) {
        if (w.side == "both" || w.side == arm_label) {
            mine.push_back(w);
        }
    }
    std::vector<TimedPose> seq;
    TimedPose start;
    start.t = 0.0;
    start.q = q_start;
    start.grip = grip_start;
    seq.push_back(start);
    if (mine.empty()) {
        return seq;
    }

    const bool timed = mine.front().has_t;
    double prev_grip = grip_start;
    openarm::scurve::Limits lim = openarm::scurve::limits_from_env();
    double t_acc = 0.0;
    for (size_t i = 0; i < mine.size(); ++i) {
        TimedPose tp;
        tp.q = mine[i].q;
        if (dyn != nullptr) {
            std::vector<double> full = tp.q;
            full.resize(std::max<size_t>(full.size(), dyn->GetDof()), 0.0);
            dyn->ClampJointPositions(full.data());
            for (size_t k = 0; k < tp.q.size() && k < full.size(); ++k) {
                tp.q[k] = full[k];
            }
        }
        tp.grip = mine[i].has_grip ? mine[i].grip : prev_grip;
        prev_grip = tp.grip;
        if (timed) {
            tp.t = mine[i].t;
        } else {
            const TimedPose& prev = seq.back();
            const size_t n = std::max(prev.q.size(), tp.q.size()) + 1;
            std::vector<double> q0(n, 0.0);
            std::vector<double> q1(n, 0.0);
            for (size_t k = 0; k < n - 1; ++k) {
                q0[k] = (k < prev.q.size()) ? prev.q[k] : 0.0;
                q1[k] = (k < tp.q.size()) ? tp.q[k] : q0[k];
            }
            q0[n - 1] = prev.grip;
            q1[n - 1] = tp.grip;
            openarm::scurve::ProfileMulti profile;
            profile.plan(q0.data(), q1.data(), n, lim);
            const double floor_T =
                move_sec / static_cast<double>(std::max<size_t>(mine.size(), 1));
            t_acc += std::max(profile.duration(), floor_T);
            tp.t = t_acc;
        }
        seq.push_back(tp);
    }
    return seq;
}

struct TimedPoseSample {
    double t = 0.0;
    std::vector<double> q;
    std::vector<double> dq;
    double grip = 0.0;
    double dgrip = 0.0;
};

// Interpolate an arm pose from a timed sequence at query time `t` with per-segment
// rest-to-rest S-curve (jerk-limited), time-stretched to the segment wall clock.
TimedPoseSample sample_sequence_scurve(const std::vector<TimedPose>& seq, double t) {
    TimedPoseSample out;
    if (seq.empty()) {
        return out;
    }
    auto fill_rest = [&](const TimedPose& p) {
        out.t = p.t;
        out.q = p.q;
        out.dq.assign(p.q.size(), 0.0);
        out.grip = p.grip;
        out.dgrip = 0.0;
    };
    if (t <= seq.front().t) {
        fill_rest(seq.front());
        out.t = t;
        return out;
    }
    if (t >= seq.back().t) {
        fill_rest(seq.back());
        out.t = t;
        return out;
    }
    const openarm::scurve::Limits lim = openarm::scurve::limits_from_env();
    for (size_t i = 1; i < seq.size(); ++i) {
        if (t <= seq[i].t) {
            const TimedPose& a = seq[i - 1];
            const TimedPose& b = seq[i];
            const double T = std::max(1e-6, b.t - a.t);
            const double tau = (t - a.t) / T;
            const size_t n = std::max(a.q.size(), b.q.size()) + 1;
            std::vector<double> q0(n, 0.0);
            std::vector<double> q1(n, 0.0);
            for (size_t k = 0; k < n - 1; ++k) {
                q0[k] = (k < a.q.size()) ? a.q[k] : 0.0;
                q1[k] = (k < b.q.size()) ? b.q[k] : q0[k];
            }
            q0[n - 1] = a.grip;
            q1[n - 1] = b.grip;
            openarm::scurve::ProfileMulti profile;
            profile.plan(q0.data(), q1.data(), n, lim);
            std::vector<double> q(n, 0.0);
            std::vector<double> dq(n, 0.0);
            const double Tp = std::max(profile.duration(), 1e-9);
            const double scale = Tp / T;
            profile.sample(tau * Tp, q.data(), dq.data());
            for (double& v : dq) {
                v *= scale;
            }
            out.t = t;
            out.q.assign(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(n - 1));
            out.dq.assign(dq.begin(), dq.begin() + static_cast<std::ptrdiff_t>(n - 1));
            out.grip = q[n - 1];
            out.dgrip = dq[n - 1];
            return out;
        }
    }
    fill_rest(seq.back());
    return out;
}

// Smoothly drive the given arms through their startup sequences with S-curve
// rest-to-rest blends (current motor pose -> TABLE_READY / waypoints). Assumes the
// follower control thread is already running (it applies the references).
void run_startup_move(const std::vector<ArmTeleopBundle*>& arms,
                      const std::vector<StartupWaypoint>& wps, double move_sec) {
    if (arms.empty()) {
        return;
    }
    std::vector<std::vector<TimedPose>> seqs;
    std::vector<size_t> arm_dofs;
    std::vector<size_t> total_dofs;
    double t_end = 0.0;
    for (ArmTeleopBundle* arm : arms) {
        const auto cur = collect_joint_states(arm->state);
        const size_t arm_dof = arm->dynamics->GetDof();
        std::vector<double> q_start(arm_dof, 0.0);
        for (size_t i = 0; i < arm_dof && i < cur.size(); ++i) {
            q_start[i] = cur[i].position;
        }
        const double grip_start = (cur.size() > arm_dof) ? cur[arm_dof].position : 0.0;
        auto seq = build_arm_sequence(arm->label, wps, q_start, grip_start, move_sec,
                                      arm->dynamics);
        t_end = std::max(t_end, seq.back().t);
        seqs.push_back(std::move(seq));
        arm_dofs.push_back(arm_dof);
        total_dofs.push_back(cur.size());
    }
    if (t_end <= 1e-3) {
        return;
    }

    std::cout << "[quest-cartesian] startup move (S-curve): driving " << arms.size()
              << " arm(s) over " << t_end << "s from measured pose -> start pose..." << std::endl;

    const double rate_hz = FREQUENCY;
    const double dt = 1.0 / rate_hz;
    const auto t0 = std::chrono::steady_clock::now();
    double t = 0.0;
    while (keep_running && t <= t_end + dt) {
        for (size_t a = 0; a < arms.size(); ++a) {
            const TimedPoseSample tp = sample_sequence_scurve(seqs[a], t);
            std::vector<JointState> ref(total_dofs[a]);
            for (size_t i = 0; i < total_dofs[a]; ++i) {
                if (i < arm_dofs[a] && i < tp.q.size()) {
                    ref[i].position = tp.q[i];
                    ref[i].velocity = (i < tp.dq.size()) ? tp.dq[i] : 0.0;
                } else if (i == arm_dofs[a]) {
                    ref[i].position = tp.grip;
                    ref[i].velocity = tp.dgrip;
                }
                ref[i].effort = 0.0;
            }
            arms[a]->state->set_all_references(ref);
        }
        std::this_thread::sleep_until(t0 + std::chrono::duration<double>(t + dt));
        t += dt;
    }

    if (!keep_running) {
        std::cout << "[quest-cartesian] startup move interrupted." << std::endl;
        return;
    }

    // Settle at the final target (zero velocity hold).
    for (size_t a = 0; a < arms.size(); ++a) {
        const TimedPoseSample tp = sample_sequence_scurve(seqs[a], t_end);
        std::vector<JointState> ref(total_dofs[a]);
        for (size_t i = 0; i < total_dofs[a]; ++i) {
            if (i < arm_dofs[a] && i < tp.q.size()) {
                ref[i].position = tp.q[i];
            } else if (i == arm_dofs[a]) {
                ref[i].position = tp.grip;
            }
            ref[i].velocity = 0.0;
            ref[i].effort = 0.0;
        }
        arms[a]->state->set_all_references(ref);
        if (arms[a]->control != nullptr) {
            arms[a]->control->ResetSafetyFilter(ref);
        }
    }
    std::cout << "[quest-cartesian] startup move complete (S-curve)." << std::endl;
}

bool looks_like_dual_joint_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.good()) {
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#' || (line.size() >= 2 && line[0] == '/' && line[1] == '/')) {
            continue;
        }
        const auto lower = [&line]() {
            std::string s = line;
            for (char& c : s) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            return s;
        }();
        // Merged dual-arm recordings: time,q0,dq0,tau0,...,q15,...
        if (lower.find("time") != std::string::npos && lower.find("q0") != std::string::npos &&
            (lower.find("q8") != std::string::npos || lower.find("q15") != std::string::npos)) {
            return true;
        }
        // Heuristic: many numeric columns without left/right/both keyword.
        if (lower.find("left") == std::string::npos && lower.find("right") == std::string::npos &&
            lower.find("both") == std::string::npos) {
            int commas = 0;
            for (char c : line) {
                if (c == ',') {
                    ++commas;
                }
            }
            // 16 joints * (q,dq[,tau]) + time => typically >= 32 commas
            if (commas >= 32) {
                return true;
            }
        }
        break;
    }
    return false;
}

std::vector<JointState> slice_joint_states(const std::vector<JointState>& joints, size_t offset,
                                           size_t count) {
    if (offset + count > joints.size()) {
        throw std::runtime_error("startup dual-CSV frame joint count mismatch");
    }
    return {joints.begin() + static_cast<std::ptrdiff_t>(offset),
            joints.begin() + static_cast<std::ptrdiff_t>(offset + count)};
}

std::vector<JointState> concat_joint_states(const std::vector<JointState>& left,
                                            const std::vector<JointState>& right) {
    std::vector<JointState> out;
    out.reserve(left.size() + right.size());
    out.insert(out.end(), left.begin(), left.end());
    out.insert(out.end(), right.begin(), right.end());
    return out;
}

std::vector<JointState> zero_velocity_frame(const std::vector<JointState>& joints) {
    std::vector<JointState> out = joints;
    for (auto& j : out) {
        j.velocity = 0.0;
        j.effort = 0.0;
    }
    return out;
}

bool make_startup_waypoints(const Options& options, std::vector<StartupWaypoint>* wps,
                            std::string* err);

// Dual-arm merged CSV startup: S-curve current->first, play trajectory, hold end.
// Returns false on hard failure (caller should abort).
bool run_startup_dual_csv(ArmTeleopBundle* left_arm, ArmTeleopBundle* right_arm,
                          const std::string& path, double transition_sec, bool simulation = false) {
    if (left_arm == nullptr || right_arm == nullptr) {
        std::cerr << "[quest-cartesian][ERROR] dual-CSV startup requires bimanual arms"
                  << std::endl;
        return false;
    }

    constexpr size_t kDualJoints = 16;
    if (simulation) {
        TeachingTrajectory validation;
        validation.load(path);
        validation.convert_grippers(false, true);
        std::cout << "[quest-cartesian] phase=startup sim target=full_trajectory file="
                  << path << " duration=" << validation.duration() << std::endl;
    }
    auto left_now = collect_joint_states(left_arm->state);
    auto right_now = collect_joint_states(right_arm->state);
    if (left_now.size() < 8 || right_now.size() < 8) {
        std::cerr << "[quest-cartesian][ERROR] unexpected joint counts for dual startup (L="
                  << left_now.size() << " R=" << right_now.size() << ")" << std::endl;
        return false;
    }
    left_now.resize(8);
    right_now.resize(8);
    // Measured velocity is often noisy right after enable; start transition at rest.
    TeachingTrajectory::Pose current{};
    for (size_t i = 0; i < 8; ++i) {
        current[i] = left_now[i].position;
        current[i+8] = right_now[i].position;
    }
    StartupTrajectory replay;
    replay.reset(path, current, simulation, transition_sec,
        env_double_or("OPENARM_QUEST_STARTUP_HOLD_SEC", .6));

    std::cout << "[quest-cartesian] startup dual-CSV: " << path << std::endl;
    std::cout << "[quest-cartesian]   S-curve current->first"
              << (transition_sec > 0.0 ? (" (" + std::to_string(transition_sec) + "s)")
                                       : " (adaptive)")
              << ", then play trajectory, then hold for VR" << std::endl;

    // First hold measured pose so Control has a consistent reference before motion.
    left_arm->state->set_all_references(zero_velocity_frame(left_now));
    right_arm->state->set_all_references(zero_velocity_frame(right_now));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const double rate_hz = 500.0;
    const double dt = 1.0 / rate_hz;
    std::vector<JointState> last = concat_joint_states(zero_velocity_frame(left_now),
                                                      zero_velocity_frame(right_now));
    bool got_sample = false;
    const auto t0 = std::chrono::steady_clock::now();
    auto next_sim_log = t0;
    double t = 0.0;
    // Safety timeout: transition (<=10s) + trajectory (~minutes) — use generous bound.
    const double timeout_sec = 600.0;

    while (keep_running && t <= timeout_sec) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
        const auto sample = replay.sample(elapsed);
        for (size_t i = 0; i < kDualJoints; ++i) {
            last[i].position = sample.position[i];
            last[i].velocity = sample.velocity[i];
            last[i].effort = 0;
        }
        got_sample = true;
        left_arm->state->set_all_references(slice_joint_states(last, 0, 8));
        right_arm->state->set_all_references(slice_joint_states(last, 8, 8));
        const auto now = std::chrono::steady_clock::now();
        if (simulation && now >= next_sim_log) {
            std::ostringstream status;
            status << "[quest-cartesian] sim startup sample elapsed="
                   << std::chrono::duration<double>(now-t0).count() << " measured=";
            const auto measured = concat_joint_states(collect_joint_states(left_arm->state),
                                                       collect_joint_states(right_arm->state));
            for (size_t i = 0; i < measured.size(); ++i) {
                if (i) status << ',';
                status << measured[i].position;
            }
            std::cout << status.str() << std::endl;
            next_sim_log = now+std::chrono::milliseconds(500);
        }
        if (got_sample && elapsed >= replay.duration()) {
            break;
        }
        std::this_thread::sleep_until(t0 + std::chrono::duration<double>(t + dt));
        t += dt;
    }

    if (!keep_running) {
        std::cout << "[quest-cartesian] startup dual-CSV interrupted." << std::endl;
        return true;
    }

    // Stop cleanly: zero velocity at final pose, brief settle, then VR may clutch.
    const auto hold = zero_velocity_frame(last);
    left_arm->state->set_all_references(slice_joint_states(hold, 0, 8));
    right_arm->state->set_all_references(slice_joint_states(hold, 8, 8));
    std::cout << "[quest-cartesian] startup dual-CSV complete; holding final pose for VR teleop."
              << std::endl;
    return true;
}

bool run_startup_for_arms(const Options& options, const std::vector<ArmTeleopBundle*>& arms) {
    if (!options.startup_move || arms.empty()) {
        return true;
    }
    if (!options.startup_file.empty() && looks_like_dual_joint_csv(options.startup_file)) {
        if (arms.size() < 2) {
            std::cerr << "[quest-cartesian][ERROR] dual-CSV startup file requires --bimanual"
                      << std::endl;
            return false;
        }
        return run_startup_dual_csv(arms[0], arms[1], options.startup_file,
                                    options.startup_transition_sec, options.sim);
    }

    std::vector<StartupWaypoint> wps;
    std::string wperr;
    if (!make_startup_waypoints(options, &wps, &wperr)) {
        std::cerr << "[quest-cartesian][ERROR] " << wperr << std::endl;
        return false;
    }
    run_startup_move(arms, wps, options.startup_move_sec);
    return true;
}

void reseed_teleop_after_startup(const std::vector<ArmTeleopBundle*>& arms) {
    for (ArmTeleopBundle* arm : arms) {
        if (arm == nullptr || arm->state == nullptr || arm->control == nullptr) {
            continue;
        }
        // Controllers are constructed before the startup trajectory and thus
        // still contain the power-on pose. Re-anchor every command/safety
        // state to fresh feedback before the reference thread begins.
        const auto measured = collect_joint_states(arm->state);
        if (measured.empty()) {
            continue;
        }
        arm->state->set_all_references(measured);
        arm->control->ResetSafetyFilter(measured);

        std::cout << "[quest-cartesian] " << arm->label
                  << " teleop re-seeded from post-startup measured pose" << std::endl;
        if (arm->sim_hardware) {
            std::ostringstream pose;
            pose << "[quest-cartesian] sim startup reached side=" << arm->label << " q=";
            for (size_t i = 0; i < measured.size(); ++i) {
                if (i) pose << ',';
                pose << measured[i].position;
            }
            std::cout << pose.str() << std::endl;
        }
    }
}

// Resolve the startup waypoints: from file if provided, else a single
// TABLE_READY target (both arms), matching the sim posture.
bool make_startup_waypoints(const Options& options, std::vector<StartupWaypoint>* wps,
                            std::string* err) {
    if (!options.startup_file.empty()) {
        return load_startup_waypoints(options.startup_file, wps, err);
    }
    StartupWaypoint w;
    w.side = "both";
    w.q = table_ready_arm_pose();
    w.has_grip = true;
    w.grip = TABLE_READY_POSITION[7];
    wps->push_back(w);
    return true;
}

}  // namespace


void maybe_log_quest_trajectory(const std::string& label, Dynamics* dynamics,
                                      HumanMimicController* human_mimic, bool grip_active,
                                      const std::vector<JointState>& current,
                                      const std::vector<JointState>& reference) {
    auto& logger = TeleopTrajectoryLogger::shared();
    if (!logger.enabled() || dynamics == nullptr || human_mimic == nullptr) {
        return;
    }
    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_tcp = Eigen::Matrix3d::Identity();
    if (!human_mimic->commanded_tcp_pose(&p_tcp, &R_tcp)) {
        return;
    }
    const TeleopControlState state =
        human_mimic->quest_timeout_active() ? TeleopControlState::Hold
                                            : TeleopControlState::Active;
    logger.log(label, dynamics, state, grip_active, p_tcp, current, reference,
               &human_mimic->q_target(), human_mimic->requested_tcp_position(),
               human_mimic->last_solve_ms(), human_mimic->requested_tcp_rotation());
}

class FollowerControlThread : public PeriodicTimerThread {
public:
    FollowerControlThread(std::shared_ptr<RobotSystemState> follower_state, Control* control,
                          std::shared_ptr<MujocoTeleopHardware> sim_hardware = nullptr,
                          double hz = FREQUENCY)
        : PeriodicTimerThread(hz),
          follower_state_(std::move(follower_state)),
          control_(control),
          sim_hardware_(std::move(sim_hardware)) {}

protected:
    void before_start() override {
        std::cout << "[quest-cartesian] follower thread start" << std::endl;
        PeriodicTimerThread::before_start();
    }

    void after_stop() override { std::cout << "[quest-cartesian] follower thread stop" << std::endl; }

    void on_timer() override {
        if (!keep_running) {
            return;
        }
        if (control_ == nullptr || !control_->unilateral_step()) {
            const std::string reason =
                control_ != nullptr ? control_->OutputDisabledReason() : "missing control";
            std::cerr << "[quest-cartesian] follower step stopped: " << reason << std::endl;
            keep_running = false;
            return;
        }
        if (sim_hardware_ && sim_hardware_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    std::shared_ptr<RobotSystemState> follower_state_;
    Control* control_;
    std::shared_ptr<MujocoTeleopHardware> sim_hardware_;
};

// Drain Quest UDP and stamp receive time. IK is not solved here.
class QuestReceiveThread : public PeriodicTimerThread {
public:
    QuestReceiveThread(QuestPoseReceiver* receiver, std::string label, double hz = kQuestReceiveHz)
        : PeriodicTimerThread(hz), receiver_(receiver), label_(std::move(label)) {}

protected:
    void before_start() override {
        std::cout << "[quest-cartesian] Quest receive @" << kQuestReceiveHz << " Hz (" << label_
                  << ")" << std::endl;
    }

    void on_timer() override {
        if (!keep_running || receiver_ == nullptr) {
            return;
        }
        (void)receiver_->receive_latest();
    }

private:
    QuestPoseReceiver* receiver_ = nullptr;
    std::string label_;
};

// Reuse the latest Quest pose at the controller reference frequency.
class QuestReferenceThread : public PeriodicTimerThread {
public:
    QuestReferenceThread(std::shared_ptr<RobotSystemState> follower_state,
                             QuestPoseReceiver* quest_receiver, Dynamics* dynamics,
                             std::string label, double hz = kQuestReferenceHz,
                             HumanMimicController* human_mimic = nullptr)
        : PeriodicTimerThread(hz),
          follower_state_(std::move(follower_state)),
          human_mimic_(human_mimic),
          quest_receiver_(quest_receiver),
          dynamics_(dynamics),
          label_(std::move(label)),
          nominal_dt_(1.0 / hz) {}

protected:
    void before_start() override {
        last_step_time_ = std::chrono::steady_clock::now();
        std::cout << "[quest-cartesian] Quest reference thread start @" << (1.0 / nominal_dt_)
                  << " Hz" << std::endl;
    }

    void after_stop() override { std::cout << "[quest-cartesian] Quest reference thread stop" << std::endl; }

    void on_timer() override {
        if (!keep_running) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        double dt = nominal_dt_;
        if (last_step_time_.time_since_epoch().count() != 0) {
            dt = std::chrono::duration<double>(now - last_step_time_).count();
            dt = std::clamp(dt, 0.5 * nominal_dt_, 3.0 * nominal_dt_);
        }
        last_step_time_ = now;

        const auto current = collect_joint_states(follower_state_);
        // Use cached Quest sample only — do not drain UDP here (QuestReceiveThread owns that).
        auto quest_pose = quest_receiver_->latest();
        QuestPoseSample sample;
        if (quest_pose.has_value()) {
            sample = quest_pose.value();
        } else {
            sample.valid = false;
        }

        // Shared Unitree / PIKA IK and joint-reference path.
        if (human_mimic_ != nullptr) {
            std::vector<JointState> reference;
            const bool ok = human_mimic_->step(current, sample, dt, &reference);
            (void)ok;
            if (!reference.empty()) {
                follower_state_->set_all_references(reference);
            }
            maybe_log_quest_trajectory(label_, dynamics_, human_mimic_,
                                             sample.grip_active, current, reference);
            if (++log_tick_ >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                log_tick_ = 0;
                const auto stats = quest_receiver_->snapshot_stats();
                std::ostringstream oss;
                oss << "[" << label_ << "-human-mimic] udp=" << stats.packets_window
                    << "Hz age_ms=" << stats.last_age_ms
                    << " grip=" << (stats.last_grip ? 1 : 0)
                    << " trig=" << stats.last_trigger
                    << " clutch=" << (human_mimic_->clutch_active() ? 1 : 0)
                    << " btnClose=" << (human_mimic_->last_button_close() ? 1 : 0)
                    << " btnOpen=" << (human_mimic_->last_button_open() ? 1 : 0)
                    << " grip_mot=" << human_mimic_->gripper_motion_code()
                    << " grip_tgt=" << human_mimic_->gripper_target()
                    << " grip_q=" << human_mimic_->gripper_measured()
                    << " quest_to=" << (human_mimic_->quest_timeout_active() ? 1 : 0);
                emit_teleop_status(oss.str());
            }
            return;
        }
    }

private:
    std::shared_ptr<RobotSystemState> follower_state_;
    HumanMimicController* human_mimic_ = nullptr;
    QuestPoseReceiver* quest_receiver_;
    Dynamics* dynamics_ = nullptr;
    std::string label_;
    double nominal_dt_ = 1.0 / kQuestReferenceHz;
    std::chrono::steady_clock::time_point last_step_time_;
    uint64_t log_tick_ = 0;
};

class BimanualFollowerControlThread : public PeriodicTimerThread {
public:
    BimanualFollowerControlThread(ArmTeleopBundle* left, ArmTeleopBundle* right,
                                  std::shared_ptr<MujocoSimWorld> sim_world,
                                  QuestUiBridge* ui = nullptr, double hz = FREQUENCY)
        : PeriodicTimerThread(hz),
          left_(left),
          right_(right),
          sim_world_(std::move(sim_world)),
          ui_(ui) {}

    void set_ui(QuestUiBridge* ui) { ui_ = ui; }

protected:
    void on_timer() override {
        // A failed step sets keep_running, but this thread keeps waking until the
        // main thread joins it. Do not command either arm on later wakes.
        if (!keep_running) {
            return;
        }
        bool left_ok = true;
        bool right_ok = true;
        if (left_ != nullptr && left_->control != nullptr) {
            left_ok = left_->control->unilateral_step();
            if (ui_ != nullptr) {
                ui_->append_motor_sample(left_);
            }
        }
        // One side tripping feedback/safety must not keep servo-ing the other.
        if (left_ok && right_ != nullptr && right_->control != nullptr) {
            right_ok = right_->control->unilateral_step();
            if (ui_ != nullptr) {
                ui_->append_motor_sample(right_);
            }
        }
        if (!left_ok || !right_ok) {
            std::cerr << "[quest-cartesian] bimanual follower step stopped"
                      << " left="
                      << (left_ != nullptr && left_->control != nullptr
                              ? left_->control->OutputDisabledReason()
                              : "")
                      << " right="
                      << (right_ != nullptr && right_->control != nullptr
                              ? right_->control->OutputDisabledReason()
                              : "")
                      << std::endl;
            keep_running = false;
            return;
        }
        // Damiao status feedback (error codes / temps) → motor_status.log + motor_fault.log
        if (++fault_tick_ >= 50) {  // ~20 Hz poll; status file itself is rate-limited (~10 Hz)
            fault_tick_ = 0;
            const double l_tgt =
                (left_ != nullptr && left_->human_mimic != nullptr)
                    ? left_->human_mimic->gripper_target()
                    : 0.0;
            const double r_tgt =
                (right_ != nullptr && right_->human_mimic != nullptr)
                    ? right_->human_mimic->gripper_target()
                    : 0.0;
            const int l_mot =
                (left_ != nullptr && left_->human_mimic != nullptr)
                    ? left_->human_mimic->gripper_motion_code()
                    : 0;
            const int r_mot =
                (right_ != nullptr && right_->human_mimic != nullptr)
                    ? right_->human_mimic->gripper_motion_code()
                    : 0;
            motor_fault_monitor().sample_bimanual(
                left_ != nullptr ? left_->openarm : nullptr,
                right_ != nullptr ? right_->openarm : nullptr, l_tgt, r_tgt, l_mot, r_mot);
            maybe_warn_gripper_slip(left_);
            maybe_warn_gripper_slip(right_);
        }
        if (sim_world_ && sim_world_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    static void maybe_warn_gripper_slip(ArmTeleopBundle* arm) {
        if (arm == nullptr || arm->human_mimic == nullptr || arm->openarm == nullptr) {
            return;
        }
        const double track =
            arm->human_mimic->gripper_measured() - arm->human_mimic->gripper_target();
        const int err = motor_gripper_error_code(arm->openarm);
        // Only warn while supposedly holding (mot=0) and tracking is bad.
        if (arm->human_mimic->gripper_motion_code() != 0) {
            return;
        }
        if (std::abs(track) < 0.15 && !is_motor_fault_code(err)) {
            return;
        }
        static std::chrono::steady_clock::time_point last_left{};
        static std::chrono::steady_clock::time_point last_right{};
        auto& last = arm->is_left ? last_left : last_right;
        const auto now = std::chrono::steady_clock::now();
        if (last.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last).count() < 0.5) {
            return;
        }
        last = now;
        std::ostringstream oss;
        oss << "[gripper-slip][" << arm->label << "] track=" << track
            << " tgt=" << arm->human_mimic->gripper_target()
            << " q=" << arm->human_mimic->gripper_measured()
            << " tau=" << motor_gripper_tau(arm->openarm)
            << " motor_err=" << err << "(" << motor_fault_text(err) << ")"
            << " t_mos=" << motor_gripper_tmos(arm->openarm)
            << "C t_rot=" << motor_gripper_trotor(arm->openarm) << "C";
        motor_fault_monitor().log_event(oss.str());
        emit_teleop_status(oss.str());
    }

    ArmTeleopBundle* left_;
    ArmTeleopBundle* right_;
    std::shared_ptr<MujocoSimWorld> sim_world_;
    QuestUiBridge* ui_ = nullptr;
    uint64_t fault_tick_ = 0;
};

class BimanualQuestReferenceThread : public PeriodicTimerThread {
public:
    BimanualQuestReferenceThread(ArmTeleopBundle* left, ArmTeleopBundle* right,
                                     QuestUiBridge* ui = nullptr, double hz = kQuestReferenceHz)
        : PeriodicTimerThread(hz), left_(left), right_(right), ui_(ui),
          nominal_dt_(1.0 / hz) {}

protected:
    void before_start() override {
        last_step_time_ = std::chrono::steady_clock::now();
        std::cout << "[quest-cartesian] bimanual Quest reference thread start @"
                  << (1.0 / nominal_dt_) << " Hz" << std::endl;
    }

    void on_timer() override {
        if (ui_ != nullptr) {
            ui_->poll();
        }
        const auto now = std::chrono::steady_clock::now();
        double dt = nominal_dt_;
        if (last_step_time_.time_since_epoch().count() != 0) {
            dt = std::chrono::duration<double>(now - last_step_time_).count();
            dt = std::clamp(dt, 0.5 * nominal_dt_, 3.0 * nominal_dt_);
        }
        last_step_time_ = now;

        if (ui_ != nullptr && ui_->couple_active) {
            // Right master first, then left slave follows locked relative pose.
            bool circle_done = false;
            if (ui_->circle_active) {
                circle_done = ui_->advance_circle(dt);
            }
            step_arm(right_, dt);
            if (circle_done) {
                ui_->stop_circle("lap complete");
            }
            ui_->apply_couple_left_follow(dt);
            step_arm(left_, dt);
        } else {
            step_arm(left_, dt);
            step_arm(right_, dt);
        }
    }

private:
    void step_arm(ArmTeleopBundle* arm, double dt) {
        if (!keep_running) {
            return;
        }
        if (arm == nullptr || arm->quest_receiver == nullptr || arm->state == nullptr) {
            return;
        }
        if (arm->human_mimic == nullptr) {
            return;
        }
        const auto current = collect_joint_states(arm->state);
        auto quest_pose = arm->quest_receiver->latest();
        QuestPoseSample sample;
        if (quest_pose.has_value()) {
            sample = quest_pose.value();
        } else {
            sample.valid = false;
        }

        // Shared Unitree / PIKA IK and joint-reference path.
        if (arm->human_mimic != nullptr) {
            std::vector<JointState> reference;
            (void)arm->human_mimic->step(current, sample, dt, &reference);
            if (!reference.empty()) {
                arm->state->set_all_references(reference);
            }
            if (ui_ != nullptr) {
                ui_->append_recording(arm, reference);
            }
            maybe_log_quest_trajectory(arm->label, arm->dynamics,
                                             arm->human_mimic.get(), sample.grip_active,
                                             current, reference);
            if (++arm->log_tick >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                arm->log_tick = 0;
                const auto stats = arm->quest_receiver->snapshot_stats();
                std::ostringstream oss;
                oss << "[" << arm->label << "-hm] udp=" << stats.packets_window
                    << "Hz grip=" << (sample.grip_active ? 1 : 0)
                    << " clutch=" << (arm->human_mimic->clutch_active() ? 1 : 0)
                    << " btnClose=" << (arm->human_mimic->last_button_close() ? 1 : 0)
                    << " btnOpen=" << (arm->human_mimic->last_button_open() ? 1 : 0)
                    << " grip_mot=" << arm->human_mimic->gripper_motion_code()
                    << " grip_tgt=" << arm->human_mimic->gripper_target()
                    << " grip_q=" << arm->human_mimic->gripper_measured()
                    << " quest_to=" << (arm->human_mimic->quest_timeout_active() ? 1 : 0);
                append_gripper_motor_status(oss, arm);
                emit_teleop_status(oss.str());
            }
            return;
        }
    }

    ArmTeleopBundle* left_;
    ArmTeleopBundle* right_;
    QuestUiBridge* ui_ = nullptr;
    double nominal_dt_ = 1.0 / kQuestReferenceHz;
    std::chrono::steady_clock::time_point last_step_time_;
};

class SimRenderThread : public PeriodicTimerThread {
public:
    SimRenderThread(std::shared_ptr<MujocoSimWorld> world, double hz = 60.0)
        : PeriodicTimerThread(hz), world_(std::move(world)) {}

protected:
    void on_timer() override {
        world_->forward_and_render();
        if (world_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    std::shared_ptr<MujocoSimWorld> world_;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        Options options;
        if (!parse_args(argc, argv, &options)) {
            return 1;
        }
        TeleopTrajectoryLogger::shared().configure_from_env();
        if (!std::filesystem::exists(options.urdf_path)) {
            std::cerr << "[quest-cartesian][ERROR] URDF not found: " << options.urdf_path
                      << std::endl;
            return 1;
        }
        if (options.sim && !std::filesystem::exists(options.mujoco_xml)) {
            std::cerr << "[quest-cartesian][ERROR] MuJoCo XML not found: " << options.mujoco_xml
                      << std::endl;
            return 1;
        }


        const std::string root_link = "openarm_body_link0";
        setenv("OPENARM_DIAGNOSTIC_URDF", options.urdf_path.c_str(), 1);
        const FollowerParams follower_params = load_follower_params();
        const bool gravity_enabled = !options.sim && env_bool_or("OPENARM_QUEST_GRAVITY_FF", true);
        GravityTune gravity_left, gravity_right;
        GravityModel gravity_model;
        const char* gravity_env = std::getenv("OPENARM_GRAVITY_YAML");
        const std::string gravity_yaml = gravity_env && *gravity_env
            ? gravity_env : "config/gravity_comp.yaml";
        if (gravity_enabled &&
            !load_gravity_yaml(gravity_yaml, &gravity_left, &gravity_right, &gravity_model)) {
            throw std::runtime_error("cannot load teleop gravity calibration: " + gravity_yaml);
        }
        const double gravity_gain = env_double_or("OPENARM_QUEST_GRAVITY_GAIN",
                                                  gravity_model.tracking_ff_gain);
        const double gravity_ramp = env_double_or("OPENARM_QUEST_GRAVITY_RAMP_SEC", 1.0);

        // Build both calibrated link7 chains before enabling either physical arm.
        // IK uses its own hand_tcp chain so fixed tool offsets cannot shift mass COMs.
        const auto make_gravity = [&](const std::string& side) {
            std::shared_ptr<Dynamics> gravity_dynamics;
            const GravityTune& tune = side == "left" ? gravity_left : gravity_right;
            if (gravity_enabled) {
                if (!std::isfinite(gravity_gain) || gravity_gain < 0 || gravity_gain > 2 ||
                    !std::isfinite(gravity_ramp) || gravity_ramp < 0.1 || gravity_ramp > 10) {
                    throw std::runtime_error("invalid teleop gravity gain/ramp");
                }
                for (size_t i = 0; i < 7; ++i) {
                    if (!std::isfinite(tune.scale[i]) || !std::isfinite(tune.bias[i])) {
                        throw std::runtime_error("non-finite teleop gravity calibration");
                    }
                }
                gravity_dynamics = std::make_shared<Dynamics>(options.urdf_path, root_link,
                                                              "openarm_" + side + "_link7");
                configure_follower_gravity_model(gravity_dynamics.get(), gravity_model, side);
                if (!gravity_dynamics->Init()) {
                    throw std::runtime_error("failed to initialize teleop gravity model");
                }
                std::array<double, 7> q{}, g{};
                gravity_dynamics->GetGravity(q.data(), g.data());
                for (double value : g) {
                    if (!std::isfinite(value)) {
                        throw std::runtime_error("non-finite teleop gravity model");
                    }
                }
                std::cout << "[quest-gravity][" << side << "] enabled yaml=" << gravity_yaml
                          << " gain=" << gravity_gain << " ramp_sec=" << gravity_ramp
                          << " limits_Nm=20,20,7,7,2,2,2" << std::endl;
            }
            return gravity_dynamics;
        };
        const auto gravity_dynamics_left = options.bimanual || options.arm_side == "left_arm"
            ? make_gravity("left") : nullptr;
        const auto gravity_dynamics_right = options.bimanual || options.arm_side == "right_arm"
            ? make_gravity("right") : nullptr;

        std::cout << "=== OpenArm Quest Cartesian Control ===" << std::endl;
        std::cout << "Backend        : " << (options.sim ? "MuJoCo sim" : "CAN hardware")
                  << std::endl;
        std::cout << "Mode           : " << (options.bimanual ? "bimanual (6DOF pose)" : "single-arm")
                  << std::endl;
        std::cout << "Teleop mode    : "
                  << (options.teleop_mode == "unitree_style"
                          ? "UNITREE_STYLE (fixed world map, bounded 6D LM)"
                          : "PIKA_STYLE (6D DIFF_QP)")
                  << std::endl;
        std::cout << "Control        : Joint references @ " << kQuestReferenceHz
                  << " Hz; Quest receive @ " << kQuestReceiveHz
                  << " Hz; follower MIT @ " << FREQUENCY << " Hz" << std::endl;
        std::cout << "URDF           : " << options.urdf_path << std::endl;

        auto setup_arm =
            [&](ArmTeleopBundle* bundle, const std::string& label, const std::string& arm_side,
                const std::string& mujoco_side, const std::string& leaf_link,
                const std::string& can_iface, uint16_t quest_port,
                const std::shared_ptr<MujocoSimWorld>& world) -> bool {
                bundle->label = label;
                bundle->arm_side = arm_side;
                bundle->mujoco_side = mujoco_side;
                bundle->dynamics = new Dynamics(options.urdf_path, root_link, leaf_link);
                if (!bundle->dynamics->Init()) {
                    std::cerr << "[quest-cartesian][ERROR] failed to initialize dynamics for "
                              << label << std::endl;
                    return false;
                }

                const std::string side = arm_side == "left_arm" ? "left" : "right";
                const GravityTune& tune = side == "left" ? gravity_left : gravity_right;
                const auto gravity_dynamics = side == "left" ? gravity_dynamics_left
                                                             : gravity_dynamics_right;

                std::shared_ptr<TeleopHardware> hardware;
                size_t arm_dof = 0;
                size_t hand_dof = 0;
                if (options.sim) {
                    bundle->sim_hardware =
                        std::make_shared<MujocoTeleopHardware>(world, mujoco_side);
                    hardware = bundle->sim_hardware;
                    arm_dof = bundle->sim_hardware->arm_motor_count();
                    hand_dof = bundle->sim_hardware->gripper_motor_count();
                } else {
                    bundle->openarm =
                        openarm_init::OpenArmInitializer::initialize_openarm(can_iface, true);
                    arm_dof = bundle->openarm->get_arm().get_motors().size();
                    hand_dof = bundle->openarm->get_gripper().get_motors().size();
                    hardware = std::make_shared<CanTeleopHardware>(bundle->openarm);
                }

                bundle->state = std::make_shared<RobotSystemState>(arm_dof, hand_dof);
                bundle->control =
                    new Control(hardware, bundle->dynamics, bundle->dynamics, bundle->state,
                                1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side, arm_dof, hand_dof);
                configure_follower_control(bundle->control, follower_params, arm_dof + hand_dof,
                                           options.sim, arm_side);
                if (gravity_dynamics) {
                    bundle->control->SetFollowerGravity(gravity_dynamics, tune, gravity_gain,
                                                        gravity_ramp);
                }
                bundle->control->SetGripperFriction(gravity_model.gripper_friction_fc,
                                                    gravity_model.gripper_friction_fv,
                                                    gravity_model.gripper_friction_k);
                if (!options.sim && bundle->openarm != nullptr) {
                    bundle->control->BindCanBus(bundle->openarm);
                }

                if (options.sim) {
                    prime_sim_arm(bundle, arm_dof, hand_dof,
                                  options.sim_table_ready && !options.startup_move);
                } else if (options.auto_home) {
                    std::cout << "  [" << label << "] homing to default pose..." << std::endl;
                    // Hold measured first so AdjustPosition does not start from zero refs.
                    if (!bundle->control->SeedHoldMeasured()) {
                        std::cerr << "[quest-cartesian][ERROR] " << label
                                  << " refusing homing without fresh CAN feedback" << std::endl;
                        return false;
                    }
                    if (!bundle->control->AdjustPosition()) {
                        std::cerr << "[quest-cartesian][ERROR] " << label
                                  << " homing stopped: " << bundle->control->OutputDisabledReason()
                                  << std::endl;
                        return false;
                    }
                    if (!prime_hardware_arm(bundle, arm_dof, hand_dof)) {
                        return false;
                    }
                } else {
                    // Critical: seed measured pose before the first MIT step. Otherwise
                    // RobotSystemState references default to 0 and the elbow/forearm
                    // suddenly chases zero on enable (before any startup quintic).
                    if (!bundle->control->SeedHoldMeasured()) {
                        std::cerr << "[quest-cartesian][ERROR] " << label
                                  << " refusing startup without fresh CAN feedback" << std::endl;
                        return false;
                    }
                    if (!prime_hardware_arm(bundle, arm_dof, hand_dof)) {
                        return false;
                    }
                    std::cout << "  [" << label
                              << "] hold seeded from measured motor pose (no zero-ref jerk)"
                              << std::endl;
                }

                bundle->quest_receiver = std::make_unique<QuestPoseReceiver>();
                if (!bundle->quest_receiver->open(quest_port)) {
                    std::cerr << "[quest-cartesian][ERROR] failed to open Quest UDP port "
                              << quest_port << " for " << label << std::endl;
                    return false;
                }
                if (options.teleop_mode == "unitree_style") {
                    HumanMimicConfig hm_cfg =
                        make_unitree_style_config(options.urdf_path, arm_side,
                                                 options.orientation_mapping, options.sim);
                    bundle->human_mimic = std::make_unique<HumanMimicController>(
                        bundle->dynamics, hm_cfg, "[" + label + "] ");
                    if (!bundle->human_mimic->Init()) {
                        std::cerr << "[quest-cartesian][ERROR] failed to init UNITREE_STYLE for "
                                  << label << std::endl;
                        return false;
                    }
                    std::cout << "  [" << label
                              << "] teleop_mode=" << options.teleop_mode
                              << std::endl;
                } else if (options.teleop_mode == "pika_style") {
                    HumanMimicConfig pika_cfg =
                        make_pika_style_config(options.urdf_path, arm_side,
                                               options.orientation_mapping, options.sim);
                    bundle->human_mimic = std::make_unique<HumanMimicController>(
                        bundle->dynamics, pika_cfg, "[" + label + "] ");
                    if (!bundle->human_mimic->Init()) {
                        std::cerr << "[quest-cartesian][ERROR] failed to init PIKA_STYLE for "
                                  << label << std::endl;
                        return false;
                    }
                    std::cout << "  [" << label
                              << "] teleop_mode=PIKA_STYLE (6D DIFF_QP, no elbow/soft-reach)"
                              << std::endl;
                }
                std::cout << "  [" << label << "] leaf=" << leaf_link << " udp=" << quest_port;
                if (!options.sim) {
                    std::cout << " can=" << can_iface;
                }

                std::cout << std::endl;
                return true;
            };

        std::shared_ptr<MujocoSimWorld> sim_world;

        if (options.bimanual) {
            if (options.sim) {
                std::cout << "MuJoCo XML     : " << options.mujoco_xml << std::endl;
            } else {
                std::cout << "Follower CAN   : left=" << options.follower_can_left
                          << " right=" << options.follower_can_right << std::endl;
            }
            std::cout << "Quest UDP      : left=" << options.quest_port_left
                      << " right=" << options.quest_port_right << std::endl;

            ArmTeleopBundle left_arm;
            ArmTeleopBundle right_arm;
            if (options.sim) {
                sim_world = std::make_shared<MujocoSimWorld>(options.mujoco_xml,
                                                             "OpenArm Quest Bimanual Sim");
            }
            const std::string left_leaf = select_leaf_link(options.urdf_path, "left_arm");
            const std::string right_leaf = select_leaf_link(options.urdf_path, "right_arm");
            if (!setup_arm(&left_arm, "left", "left_arm", "left", left_leaf,
                           options.follower_can_left, options.quest_port_left, sim_world)) {
                return 1;
            }
            if (!setup_arm(&right_arm, "right", "right_arm", "right", right_leaf,
                           options.follower_can_right, options.quest_port_right, sim_world)) {
                if (!options.sim && left_arm.control != nullptr) {
                    left_arm.control->Shutdown();
                }
                return 1;
            }
            // Recording and coupled motion require the arm side in every mode.
            left_arm.is_left = true;
            right_arm.is_left = false;

            BimanualFollowerControlThread follower_thread(&left_arm, &right_arm, sim_world, nullptr,
                                                         FREQUENCY);
            QuestReceiveThread quest_left(left_arm.quest_receiver.get(), "left", kQuestReceiveHz);
            QuestReceiveThread quest_right(right_arm.quest_receiver.get(), "right", kQuestReceiveHz);

            QuestUiBridge quest_ui;
            quest_ui.left = &left_arm;
            quest_ui.right = &right_arm;
            options.quest_ui = env_bool_or("OPENARM_QUEST_UI", options.quest_ui);
            options.quest_jog_step_m =
                env_double_or("OPENARM_QUEST_JOG_STEP", options.quest_jog_step_m);
            options.quest_jog_speed_mps =
                env_double_or("OPENARM_QUEST_JOG_SPEED", options.quest_jog_speed_mps);
            options.quest_jog_omega_rps =
                env_double_or("OPENARM_QUEST_JOG_OMEGA", options.quest_jog_omega_rps);
            if (const char* rd = std::getenv("OPENARM_QUEST_RECORD_DIR");
                rd != nullptr && *rd != '\0' && options.quest_record_dir.empty()) {
                options.quest_record_dir = rd;
            }
            if (!quest_ui.open(options)) {
                std::cerr << "[quest-cartesian][ERROR] failed to open quest UI UDP ports"
                          << std::endl;
                return 1;
            }
            if (options.quest_ui) {
                follower_thread.set_ui(&quest_ui);
            }

            BimanualQuestReferenceThread reference_thread(
                &left_arm, &right_arm, options.quest_ui ? &quest_ui : nullptr, kQuestReferenceHz);
            SimRenderThread render_thread(sim_world, 60.0);

            // Left is primed before right starts. Right's CAN open and model init
            // routinely exceed the 150 ms feedback limit, so the first bimanual
            // tick used to disable the left arm and exit. Refresh both together.
            if (!options.sim) {
                const auto deadline = std::chrono::steady_clock::now() +
                                      std::chrono::milliseconds(kCanFeedbackStartupWaitMs);
                CanFeedbackVerdict left_feedback;
                CanFeedbackVerdict right_feedback;
                do {
                    if (left_arm.openarm != nullptr) {
                        left_arm.openarm->refresh_all();
                        left_arm.openarm->recv_all(2000);
                        left_feedback = evaluate_openarm_feedback(
                            *left_arm.openarm, std::chrono::steady_clock::now());
                    }
                    if (right_arm.openarm != nullptr) {
                        right_arm.openarm->refresh_all();
                        right_arm.openarm->recv_all(2000);
                        right_feedback = evaluate_openarm_feedback(
                            *right_arm.openarm, std::chrono::steady_clock::now());
                    }
                    if ((left_feedback.ok && right_feedback.ok) ||
                        std::chrono::steady_clock::now() >= deadline) {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                } while (std::chrono::steady_clock::now() < deadline);
                if (!left_feedback.ok || !right_feedback.ok) {
                    std::cerr << "[quest-cartesian][ERROR] CAN feedback stale before the "
                                 "control loop; not entering torque"
                              << " left=" << format_can_feedback_verdict(left_feedback, "left")
                              << " right=" << format_can_feedback_verdict(right_feedback, "right")
                              << std::endl;
                    if (left_arm.control != nullptr) {
                        left_arm.control->Shutdown();
                    }
                    if (right_arm.control != nullptr) {
                        right_arm.control->Shutdown();
                    }
                    return 1;
                }
            }

            follower_thread.start_thread();
            if (options.sim) {
                render_thread.start_thread();
            }
            if (options.startup_move) {
                if (!run_startup_for_arms(options, {&left_arm, &right_arm})) {
                    return 1;
                }
                reseed_teleop_after_startup({&left_arm, &right_arm});
            }
            if (options.sim && !keep_running) {
                follower_thread.stop_thread();
                render_thread.stop_thread();
                return 0;
            }
            quest_left.start_thread();
            quest_right.start_thread();
            reference_thread.start_thread();

            // Keep the console's readiness banner on its own line even when
            // periodic threads print concurrently.
            std::cout << "\n[quest-cartesian] bimanual running.\n" << std::flush;
            print_teleop_help();
            if (!options.sim) {
                print_real_hardware_banner();
            }

            GripperTelemSender grip_telem;
            const uint16_t grip_telem_port = static_cast<uint16_t>(
                std::clamp(env_double_or("OPENARM_QUEST_GRIP_TELEM_PORT", 47945.0), 0.0, 65535.0));
            if (!options.sim && grip_telem_port > 0) {
                grip_telem.open(grip_telem_port);
            }

            while (keep_running) {
                if (!options.sim) {
                    grip_telem.maybe_send(left_arm.openarm, right_arm.openarm, 0.05);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            reference_thread.stop_thread();
            quest_left.stop_thread();
            quest_right.stop_thread();
            follower_thread.stop_thread();
            if (options.sim) {
                render_thread.stop_thread();
            }
            TeleopTrajectoryLogger::shared().close();
            bool left_stopped = false;
            bool right_stopped = false;
            if (!options.sim) {
                left_stopped =
                    left_arm.control != nullptr && left_arm.control->OutputDisabled();
                right_stopped =
                    right_arm.control != nullptr && right_arm.control->OutputDisabled();
                if (left_stopped || right_stopped) {
                    std::cerr << "[quest-cartesian] skip damped Kp release; output already stopped"
                              << " left="
                              << (left_arm.control != nullptr
                                      ? left_arm.control->OutputDisabledReason()
                                      : "")
                              << " right="
                              << (right_arm.control != nullptr
                                      ? right_arm.control->OutputDisabledReason()
                                      : "")
                              << std::endl;
                } else {
                    send_kd_only_release(left_arm.openarm, right_arm.openarm, follower_params.kp,
                                         follower_params.kd);
                }
                left_arm.control->Shutdown();
                right_arm.control->Shutdown();
            }
            delete left_arm.control;
            delete right_arm.control;
            delete left_arm.dynamics;
            delete right_arm.dynamics;
            delete left_arm.openarm;
            delete right_arm.openarm;
            if (!options.sim && (left_stopped || right_stopped)) {
                return 1;
            }
            return 0;
        }

        const std::string leaf_link = select_leaf_link(options.urdf_path, options.arm_side);
        const std::string mujoco_side = (options.arm_side == "left_arm") ? "left" : "right";
        const std::string label = (options.arm_side == "left_arm") ? "left" : "right";
        std::cout << "Arm side       : " << options.arm_side << std::endl;
        if (!options.sim) {
            std::cout << "Follower CAN   : " << options.follower_can << std::endl;
        } else {
            std::cout << "MuJoCo XML     : " << options.mujoco_xml << std::endl;
            std::cout << "MuJoCo side    : " << mujoco_side << std::endl;
        }
        std::cout << "Quest UDP port : " << options.quest_port << std::endl;
        std::cout << "Leaf link      : " << leaf_link << std::endl;

        ArmTeleopBundle arm;
        if (options.sim) {
            sim_world = std::make_shared<MujocoSimWorld>(options.mujoco_xml,
                                                         "OpenArm Quest Cartesian Sim");
        }
        if (!setup_arm(&arm, label, options.arm_side, mujoco_side, leaf_link, options.follower_can,
                       options.quest_port, sim_world)) {
            return 1;
        }

        std::cout << "Teleop mode    : "
                  << (options.teleop_mode == "unitree_style"
                          ? "UNITREE_STYLE (fixed world map, bounded 6D LM)"
                          : "PIKA_STYLE (6D DIFF_QP)")
                  << std::endl;

        arm.is_left = (options.arm_side == "left_arm");

        FollowerControlThread follower_thread(arm.state, arm.control, arm.sim_hardware, FREQUENCY);
        QuestReceiveThread quest_thread(arm.quest_receiver.get(), label, kQuestReceiveHz);
        QuestReferenceThread reference_thread(
            arm.state, arm.quest_receiver.get(), arm.dynamics, label,
            kQuestReferenceHz, arm.human_mimic.get());
        SimRenderThread render_thread(sim_world, 60.0);

        follower_thread.start_thread();
        if (options.sim) {
            render_thread.start_thread();
        }
        if (options.startup_move) {
            if (!run_startup_for_arms(options, {&arm})) {
                return 1;
            }
            reseed_teleop_after_startup({&arm});
        }
        if (options.sim && !keep_running) {
            follower_thread.stop_thread();
            render_thread.stop_thread();
            return 0;
        }
        quest_thread.start_thread();
        reference_thread.start_thread();

        std::cout << "\n[quest-cartesian] running.\n" << std::flush;
        print_teleop_help();
        if (!options.sim) {
            print_real_hardware_banner();
        }

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        reference_thread.stop_thread();
        quest_thread.stop_thread();
        follower_thread.stop_thread();
        if (options.sim) {
            render_thread.stop_thread();
        }
        TeleopTrajectoryLogger::shared().close();
        if (!options.sim) {
            if (arm.control != nullptr && arm.control->OutputDisabled()) {
                std::cerr << "[quest-cartesian] skip damped Kp release; output already stopped: "
                          << arm.control->OutputDisabledReason() << std::endl;
            } else {
                send_kd_only_release(arm.openarm, nullptr, follower_params.kp, follower_params.kd);
            }
            arm.control->Shutdown();
        }
        delete arm.control;
        delete arm.dynamics;
        delete arm.openarm;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[quest-cartesian][ERROR] " << e.what() << std::endl;
        return 1;
    }
}
