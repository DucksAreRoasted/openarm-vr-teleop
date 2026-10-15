#pragma once

#include <Eigen/Dense>
#include <cstdint>

struct QuestPoseSample {
    int64_t timestamp_ms = 0;           // source timestamp from Quest bridge
    int64_t receive_timestamp_ns = 0;   // steady_clock receive time (ns)
    uint64_t sequence_id = 0;
    bool grip_active = false;
    double trigger = 0.0;
    // Face buttons: right A/B, left X/Y (mapped to close/open in controller).
    bool button_close = false;
    bool button_open = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
};

enum class TeleopControlState {
    Idle,
    Active,
    Degraded,
    Hold,
    Estop,
};
