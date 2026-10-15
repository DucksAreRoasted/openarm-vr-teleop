#pragma once

// CAN state-feedback check shared by hardware control paths.
//
// Invalid when any of these is true:
//   - last_state_received() is the default epoch (no reply has ever arrived)
//   - age_ms > max_age_ms (default kCanFeedbackMaxAgeMs, 150)
//   - error_code > max_error_code (default kCanFeedbackMaxErrorCode, 1)
// error_code 含义待硬件确认.

#include <openarm_constants.hpp>

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

struct CanFeedbackSample {
    std::chrono::steady_clock::time_point last_received{};
    int error_code = 0;
    uint32_t send_can_id = 0;
    uint32_t recv_can_id = 0;
};

struct CanFeedbackLimits {
    double max_age_ms = kCanFeedbackMaxAgeMs;
    int max_error_code = kCanFeedbackMaxErrorCode;
};

enum class CanFeedbackFault {
    None = 0,
    NeverReceived,
    Stale,
    ErrorCode,
};

struct CanFeedbackVerdict {
    bool ok = false;
    CanFeedbackFault fault = CanFeedbackFault::NeverReceived;
    double age_ms = -1.0;
    int error_code = 0;
    size_t index = 0;
    uint32_t send_can_id = 0;
    uint32_t recv_can_id = 0;
    double max_age_ms = kCanFeedbackMaxAgeMs;
    int max_error_code = kCanFeedbackMaxErrorCode;
};

// Pure check. `now` is injected so tests do not need a motor or a CAN socket.
inline CanFeedbackVerdict evaluate_can_feedback(
    const std::vector<CanFeedbackSample>& samples,
    std::chrono::steady_clock::time_point now, const CanFeedbackLimits& limits = {}) {
    CanFeedbackVerdict verdict;
    verdict.max_age_ms = limits.max_age_ms;
    verdict.max_error_code = limits.max_error_code;
    if (samples.empty()) {
        verdict.ok = false;
        verdict.fault = CanFeedbackFault::NeverReceived;
        verdict.age_ms = -1.0;
        return verdict;
    }

    double worst_age_ms = -1.0;
    for (size_t index = 0; index < samples.size(); ++index) {
        const CanFeedbackSample& sample = samples[index];
        const bool missing = sample.last_received.time_since_epoch().count() == 0;
        const double age_ms =
            missing ? -1.0
                    : std::chrono::duration<double, std::milli>(now - sample.last_received).count();
        if (!missing && age_ms > worst_age_ms) {
            worst_age_ms = age_ms;
        }

        CanFeedbackFault fault = CanFeedbackFault::None;
        if (missing) {
            fault = CanFeedbackFault::NeverReceived;
        } else if (sample.error_code > limits.max_error_code) {
            fault = CanFeedbackFault::ErrorCode;
        } else if (age_ms > limits.max_age_ms) {
            fault = CanFeedbackFault::Stale;
        }
        if (fault != CanFeedbackFault::None) {
            verdict.ok = false;
            verdict.fault = fault;
            verdict.age_ms = age_ms;
            verdict.error_code = sample.error_code;
            verdict.index = index;
            verdict.send_can_id = sample.send_can_id;
            verdict.recv_can_id = sample.recv_can_id;
            return verdict;
        }
    }

    verdict.ok = true;
    verdict.fault = CanFeedbackFault::None;
    verdict.age_ms = worst_age_ms;
    return verdict;
}

inline std::string format_can_feedback_verdict(const CanFeedbackVerdict& verdict,
                                               std::string_view context) {
    const char* kind = "stale CAN feedback";
    if (verdict.fault == CanFeedbackFault::NeverReceived) {
        kind = "no CAN feedback";
    } else if (verdict.fault == CanFeedbackFault::ErrorCode) {
        kind = "motor fault";
    } else if (verdict.fault == CanFeedbackFault::None) {
        kind = "CAN feedback ok";
    }
    std::ostringstream message;
    message << context << ": " << kind << " index=" << verdict.index
            << " joint=" << verdict.send_can_id << " rx_id=0x" << std::hex << verdict.recv_can_id
            << std::dec << " age_ms=" << verdict.age_ms << " status_code=" << verdict.error_code
            << " max_age_ms=" << verdict.max_age_ms
            << " max_error_code=" << verdict.max_error_code
            << " (error_code 含义待硬件确认)";
    return message.str();
}

template <class Motors>
void append_motor_feedback_samples(const Motors& motors, std::vector<CanFeedbackSample>* out) {
    if (out == nullptr) {
        return;
    }
    for (const auto& motor : motors) {
        CanFeedbackSample sample;
        sample.last_received = motor.last_state_received();
        sample.error_code = motor.get_error_code();
        sample.send_can_id = motor.get_send_can_id();
        sample.recv_can_id = motor.get_recv_can_id();
        out->push_back(sample);
    }
}

// Arm motors first, then the gripper motor. Same 8-motor set mirror teaching checks.
template <class Arm>
CanFeedbackVerdict evaluate_openarm_feedback(Arm& arm,
                                            std::chrono::steady_clock::time_point now,
                                            const CanFeedbackLimits& limits = {}) {
    std::vector<CanFeedbackSample> samples;
    append_motor_feedback_samples(arm.get_arm().get_motors(), &samples);
    append_motor_feedback_samples(arm.get_gripper().get_motors(), &samples);
    return evaluate_can_feedback(samples, now, limits);
}

template <class Arm>
bool can_feedback_allows_torque(Arm* arm, std::string_view context, std::string* detail,
                                const CanFeedbackLimits& limits = {}) {
    if (arm == nullptr) {
        if (detail != nullptr) {
            *detail = std::string(context) + ": no CAN bus (error_code 含义待硬件确认)";
        }
        return false;
    }
    const CanFeedbackVerdict verdict =
        evaluate_openarm_feedback(*arm, std::chrono::steady_clock::now(), limits);
    if (verdict.ok) {
        return true;
    }
    if (detail != nullptr) {
        *detail = format_can_feedback_verdict(verdict, context);
    }
    return false;
}

// Poll refresh/recv until every motor is fresh or the timeout expires.
// Does not send a position or torque frame. Does not open a socket.
template <class Arm>
CanFeedbackVerdict refresh_until_fresh(
    Arm* arm, std::chrono::milliseconds timeout = std::chrono::milliseconds(kCanFeedbackStartupWaitMs),
    const CanFeedbackLimits& limits = {}) {
    CanFeedbackVerdict verdict;
    if (arm == nullptr) {
        verdict.ok = false;
        verdict.fault = CanFeedbackFault::NeverReceived;
        verdict.age_ms = -1.0;
        verdict.max_age_ms = limits.max_age_ms;
        verdict.max_error_code = limits.max_error_code;
        return verdict;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
        arm->refresh_all();
        arm->recv_all(2000);
        verdict = evaluate_openarm_feedback(*arm, std::chrono::steady_clock::now(), limits);
        if (verdict.ok || std::chrono::steady_clock::now() >= deadline) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return verdict;
}
