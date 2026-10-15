#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <vector>

namespace gravity_calibration {

constexpr double MAX_JOINT_SPEED = 0.10;
constexpr double TRANSIT_JOINT_SPEED = 0.20;
constexpr std::array<double, 7> APPROACH_OFFSET{0.15, 0.15, 0.12, 0.12, 0.12, 0.10, 0.12};
constexpr double kLimitMargin = 0.05;
// Left-arm URDF limits; right is the joint-space mirror.
constexpr std::array<double, 7> kLeftLower{-3.490659, -3.316125, -1.570796, 0.0,
                                           -2.792527, -0.785398, -1.570796};
constexpr std::array<double, 7> kLeftUpper{1.396263, 0.174533, 1.570796, 2.443461,
                                           2.792527, 0.785398, 1.570796};


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
    // Arm close to the body, elbow nearly straight, j1 forward raise 80°→0°.
    // The poses above all keep j2 abducted, which leaves j1 unconstrained here.
    // j4 = 0.20 keeps goal − δ off the j4 lower limit.
    {"s_flex80", 344, Pose{-1.40, -0.15, 0.0, 0.20, 0.0, 0.0, 0.0}},
    {"s_flex60", 343, Pose{-1.05, -0.15, 0.0, 0.20, 0.0, 0.0, 0.0}},
    {"s_flex40", 342, Pose{-0.70, -0.15, 0.0, 0.20, 0.0, 0.0, 0.0}},
    {"s_flex20", 341, Pose{-0.35, -0.15, 0.0, 0.20, 0.0, 0.0, 0.0}},
    {"s_flex0", 340, Pose{0.0, -0.15, 0.0, 0.20, 0.0, 0.0, 0.0}},
    // Shoulder close to hanging vertically, including both sides of j1 = 0.
    // Bent-elbow poses cover the reported zero-g lift posture; the last pose
    // checks that a local correction also works with the elbow nearly straight.
    {"s_vertical_n", 350, Pose{-0.12, -0.05, 0.0, 0.45, 0.0, 0.0, 0.0}},
    {"s_vertical_0", 351, Pose{0.0, -0.05, 0.0, 0.45, 0.0, 0.0, 0.0}},
    {"s_vertical_p", 352, Pose{0.12, -0.05, 0.0, 0.45, 0.0, 0.0, 0.0}},
    {"s_vertical_ext", 353, Pose{0.0, -0.10, 0.0, 0.20, 0.0, 0.0, 0.0}},
    {"s_home2", 330, Pose{0.0, -0.70, 0.0, 1.20, 0.0, 0.0, 0.0}},
};

// Visual left↔right mirror in joint space for OpenArm V1:
// - j1/j2: negate (shoulder); j3/j4: same (elbow)
// - j5/j6/j7: negate (wrist); URDF j7 axis flip alone is not enough for visual mirror
inline Pose mirror_right(const Pose& left) {
    Pose right = left;
    right[0] = -left[0];
    right[1] = -left[1];
    right[4] = -left[4];
    right[5] = -left[5];
    right[6] = -left[6];
    return right;
}

inline double ramp_duration(const Pose& a, const Pose& b, double speed = MAX_JOINT_SPEED) {
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, std::abs(b[i] - a[i]));
    }
    return std::max(1.0, 1.5 * worst / speed);
}

// Left-frame pose shifted by sign·δ, clamped inside the URDF limits.
inline Pose approach_offset(const Pose& left_goal, int sign) {
    Pose out = left_goal;
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = std::clamp(left_goal[i] + sign * APPROACH_OFFSET[i],
                            kLeftLower[i] + kLimitMargin, kLeftUpper[i] - kLimitMargin);
    }
    return out;
}

// Choose the nearer release side first, retaining BOTH sides and all waypoints.
inline std::array<int, 2> approach_order(const Pose& at_left, const Pose& at_right,
                                       const Pose& goal) {
    auto cost = [&](int sign) {
        const Pose off = approach_offset(goal, sign);
        return std::max(ramp_duration(at_left, off, TRANSIT_JOINT_SPEED),
                        ramp_duration(at_right, mirror_right(off), TRANSIT_JOINT_SPEED));
    };
    return cost(-1) < cost(1) ? std::array<int, 2>{-1, 1} : std::array<int, 2>{1, -1};
}

// Early completion requires a full quiet observation window on BOTH arms.
// Dwell still runs to its original 3 s limit if the plateau is not stable.
class StableHoldWindow {
public:
    bool update(double elapsed, bool quiet, const Pose& q_left, const Pose& q_right,
                const Pose& torque_left, const Pose& torque_right) {
        if (!quiet) {
            samples_.clear();
            return false;
        }
        samples_.push_back({elapsed, q_left, q_right, torque_left, torque_right});
        while (samples_.size() > 1 && elapsed - samples_[1].time >= 1.0) {
            samples_.pop_front();
        }
        if (elapsed < 1.2 || samples_.size() < 80 ||
            elapsed - samples_.front().time < 1.0) {
            return false;
        }
        for (size_t joint = 0; joint < 7; ++joint) {
            for (int side = 0; side < 2; ++side) {
                double q_min = 1e9, q_max = -1e9, tau_min = 1e9, tau_max = -1e9;
                for (const auto& sample : samples_) {
                    const double q = side == 0 ? sample.ql[joint] : sample.qr[joint];
                    const double tau = side == 0 ? sample.tl[joint] : sample.tr[joint];
                    if (!std::isfinite(q) || !std::isfinite(tau)) return false;
                    q_min = std::min(q_min, q);
                    q_max = std::max(q_max, q);
                    tau_min = std::min(tau_min, tau);
                    tau_max = std::max(tau_max, tau);
                }
                if (q_max - q_min > 0.008 || tau_max - tau_min > (joint < 4 ? 0.05 : 0.02)) {
                    return false;
                }
            }
        }
        return true;
    }

private:
    struct Sample { double time; Pose ql, qr, tl, tr; };
    std::deque<Sample> samples_;
};


}  // namespace gravity_calibration
