// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <time.h>
#include <unistd.h>

#include <array>
#include <iostream>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <vector>

constexpr double PI = 3.14159265358979323846;

constexpr size_t NJOINTS = 8;
constexpr size_t NMOTORS = 8;

constexpr int ROLE_LEADER = 1;
constexpr int ROLE_FOLLOWER = 2;

constexpr const char* CAN0 = "can0";
constexpr const char* CAN1 = "can1";
constexpr const char* CAN2 = "can2";
constexpr const char* CAN3 = "can3";

constexpr bool TANHFRIC = true;

constexpr double FREQUENCY = 1000.0;
constexpr double CUTOFF_FREQUENCY = 90.0;

constexpr double ELBOWLIMIT = 0.0;
// Both current J5 units are qualified for a conservative ±160° operating range
// (measured mechanical contacts are approximately ±165.5°).
// description and controller safety configuration can share the same value.
constexpr double J5_POSITION_LIMIT = (8.0 / 9.0) * PI;

inline const std::array<double, NMOTORS> INITIAL_POSITION = {0, 0, 0, 0, 0, 0, 0, 0};

// Table-ready / desk posture (7 arm joints + gripper). Elbow = joint4 (index 3) ≈ 90°.
// Matches openarm_mujoco/v2/cell.xml keyframe "home" arm portion.
inline const std::array<double, NMOTORS> TABLE_READY_POSITION = {
    0.0, 0.0, 0.0, 1.570796, 0.0, 0.0, 0.0, 0.0};

// Historical software position window. This is NOT the URDF mechanical limit.
// Elbow (index 3, openarm_*_joint4) is π here; assets/v1.urdf upper is 2.443461.
// SafetyFilter does not apply these arrays. Do not enable them as a position clamp.
inline const std::array<double, NMOTORS> position_limit_max = {
    (2.0 / 3.0) * PI, PI, PI / 2.0, PI, J5_POSITION_LIMIT, PI / 2.0, PI / 2.0, PI};
inline const std::array<double, NMOTORS> position_limit_min = {
    -(2.0 / 3.0) * PI, -PI / 2.0, -PI / 2.0, ELBOWLIMIT, -J5_POSITION_LIMIT,
    -PI / 2.0, -PI / 2.0, -PI};

// Backward-compatible aliases (Leader and Follower limits were identical)
#define position_limit_max_L position_limit_max
#define position_limit_max_F position_limit_max
#define position_limit_min_L position_limit_min
#define position_limit_min_F position_limit_min

// Joint velocity limit (rad/s). A value <= 0 means that joint's velocity limit
// is not enabled. Simulation and real hardware use this same reading: do not
// treat 0 as "fill in a default". Left at 0 on purpose; do not invent numbers.
inline const std::array<double, NMOTORS> velocity_limit = {0, 0, 0, 0, 0, 0, 0, 0};
#define velocity_limit_L velocity_limit
#define velocity_limit_F velocity_limit

// Feedforward effort limit (Nm). A value <= 0 means that joint's effort clamp
// is not enabled. Simulation and real hardware use this same reading.
// Left at 0 on purpose; do not invent numbers.
inline const std::array<double, NMOTORS> effort_limit = {0, 0, 0, 0, 0, 0, 0, 0};
#define effort_limit_L effort_limit
#define effort_limit_F effort_limit

// CAN state-feedback freshness. Same numeric standard as mirror teaching's
// validate_feedback(), kept here so other real-arm paths can share it.
// A motor is invalid when it has never received a state frame, the frame is
// older than kCanFeedbackMaxAgeMs, or error_code > kCanFeedbackMaxErrorCode.
// error_code 含义待硬件确认: 0 and 1 are accepted until the Damiao map is
// confirmed on hardware. Values above the threshold are rejected.
constexpr double kCanFeedbackMaxAgeMs = 150.0;
constexpr int kCanFeedbackMaxErrorCode = 1;
// How long startup may poll before refusing to enter a torque loop.
// The freshness threshold stays kCanFeedbackMaxAgeMs; this is only the wait.
constexpr int kCanFeedbackStartupWaitMs = 1000;

// Motor configuration structure
struct MotorConfig {
    std::vector<openarm::damiao_motor::MotorType> arm_motor_types;
    std::vector<uint32_t> arm_send_can_ids;
    std::vector<uint32_t> arm_recv_can_ids;
    openarm::damiao_motor::MotorType gripper_motor_type;
    uint32_t gripper_send_can_id;
    uint32_t gripper_recv_can_id;
};

// Global default motor configuration
inline const MotorConfig DEFAULT_MOTOR_CONFIG = {
    // Standard 7-DOF arm motor configuration
    {openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
     openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
     openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
     openarm::damiao_motor::MotorType::DM4310},

    // Standard CAN IDs for arm motors
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
    {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},

    // Standard gripper configuration
    openarm::damiao_motor::MotorType::DM4310,
    0x08,
    0x18};

// opening function
inline void printOpenArmBanner() {
    std::cout << R"(

                                     ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗
                                    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║
                                    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║
                                    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║
                                    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║
                                     ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝

██████╗ ██╗██╗      █████╗ ████████╗███████╗██████╗  █████╗ ██╗          ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗██╗██╗██╗
██╔══██╗██║██║     ██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗██║         ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║██║██║██║
██████╔╝██║██║     ███████║   ██║   █████╗  ██████╔╝███████║██║         ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║██║██║██║
██╔══██╗██║██║     ██╔══██║   ██║   ██╔══╝  ██╔══██╗██╔══██║██║         ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ╚═╝╚═╝╚═╝╚═╝
██████╔╝██║███████╗██║  ██║   ██║   ███████╗██║  ██║██║  ██║███████╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗██╗██╗██╗██╗
╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝╚═╝╚═╝╚═╝

    )" << std::endl;
}
