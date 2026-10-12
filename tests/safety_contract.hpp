#pragma once

// Safety-test thresholds. Change them only in this header.
// Agreed with weiqiang on 2026-10-12. C++ tests include this file; Python
// tests parse the same constants so the suite has a single source.

// human_mimic drops the clutch when Quest age reaches this (seconds).
inline constexpr double kQuestTimeoutSec = 0.15;

// human_mimic freezes after this many consecutive whole-arm IK failures.
// The controller compares with `solve_fail_count_ > kSolveFailFreezeCount`.
inline constexpr int kSolveFailFreezeCount = 30;

// CAN feedback older than this is stale. Equality at the boundary stays fresh
// (`age_ms > kCanFeedbackMaxAgeMs`), matching mirror teaching.
inline constexpr double kCanFeedbackMaxAgeMs = 150.0;

// error_code above this is treated as a fault. 待硬件确认: 0 and 1 stay
// acceptable until the Damiao status-code meanings are confirmed.
inline constexpr int kCanErrorCodeMaxOk = 1;

// URDF openarm_left_joint4 / openarm_right_joint4 upper limit (rad).
// Not the wider pi value in openarm_constants.hpp position_limit_max.
inline constexpr int kElbowJointIndex = 3;
inline constexpr double kElbowUrdfUpperRad = 2.443461;
