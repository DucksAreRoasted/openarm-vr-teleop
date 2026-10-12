// Calls evaluate_can_feedback from PR #4. No socket and no motor.
// Compile this only when src/openarm_port/can_feedback.hpp declares the function.
// Thresholds come from that header's CanFeedbackLimits defaults.

#include <openarm_port/can_feedback.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

using clock = std::chrono::steady_clock;

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

CanFeedbackSample sample_aged(clock::time_point now, int age_ms, int error_code) {
    CanFeedbackSample sample;
    sample.last_received = now - std::chrono::milliseconds(age_ms);
    sample.error_code = error_code;
    sample.send_can_id = 4;
    sample.recv_can_id = 0x14;
    return sample;
}

}  // namespace

int main() {
    const CanFeedbackLimits limits;
    const auto now = clock::time_point(std::chrono::milliseconds(100000));
    const int age_limit = static_cast<int>(limits.max_age_ms);

    const CanFeedbackVerdict empty = evaluate_can_feedback({}, now, limits);
    if (empty.ok || empty.fault != CanFeedbackFault::NeverReceived) {
        return fail("empty feedback was treated as valid");
    }

    CanFeedbackSample never_heard;
    never_heard.error_code = 0;
    const CanFeedbackVerdict missing = evaluate_can_feedback({never_heard}, now, limits);
    if (missing.ok || missing.fault != CanFeedbackFault::NeverReceived) {
        return fail("epoch timestamp was treated as a real reply");
    }

    const CanFeedbackVerdict at_limit =
        evaluate_can_feedback({sample_aged(now, age_limit, 0)}, now, limits);
    if (!at_limit.ok || at_limit.fault != CanFeedbackFault::None) {
        return fail("feedback aged exactly max_age_ms was rejected");
    }

    const CanFeedbackVerdict stale =
        evaluate_can_feedback({sample_aged(now, age_limit + 1, 0)}, now, limits);
    if (stale.ok || stale.fault != CanFeedbackFault::Stale) {
        return fail("feedback older than max_age_ms was accepted");
    }

    const CanFeedbackVerdict status_ok =
        evaluate_can_feedback({sample_aged(now, 0, limits.max_error_code)}, now, limits);
    if (!status_ok.ok) {
        return fail("error_code == max_error_code was rejected");
    }

    const CanFeedbackVerdict faulted =
        evaluate_can_feedback({sample_aged(now, 0, limits.max_error_code + 1)}, now, limits);
    if (faulted.ok || faulted.fault != CanFeedbackFault::ErrorCode) {
        return fail("error_code above the limit was accepted");
    }

    // The checker reports the fault before the age, matching the header order.
    const CanFeedbackVerdict fault_and_stale = evaluate_can_feedback(
        {sample_aged(now, age_limit + 40, limits.max_error_code + 1)}, now, limits);
    if (fault_and_stale.ok || fault_and_stale.fault != CanFeedbackFault::ErrorCode) {
        return fail("a faulted stale sample was not reported as ErrorCode");
    }

    CanFeedbackSample future = sample_aged(now, 0, 0);
    future.last_received = now + std::chrono::seconds(1);
    const CanFeedbackVerdict ahead = evaluate_can_feedback({future}, now, limits);
    if (!ahead.ok || ahead.fault != CanFeedbackFault::None) {
        return fail("a future timestamp was treated as stale");
    }

    const CanFeedbackSample fresh = sample_aged(now, 1, 0);
    const CanFeedbackSample old = sample_aged(now, age_limit + 5, 0);
    const CanFeedbackVerdict second = evaluate_can_feedback({fresh, old}, now, limits);
    if (second.ok || second.index != 1 || second.fault != CanFeedbackFault::Stale) {
        return fail("the stale sample index was not reported");
    }

    // The template is instantiated as a whole, so the unused arm methods must exist.
    // Only the null pointer is called; nothing opens a socket.
    struct StubPart {
        struct StubMotor {
            clock::time_point last_state_received() const { return {}; }
            int get_error_code() const { return 0; }
            uint32_t get_send_can_id() const { return 0; }
            uint32_t get_recv_can_id() const { return 0; }
        };
        std::vector<StubMotor> get_motors() const { return {}; }
    };
    struct StubArm {
        StubPart get_arm() { return {}; }
        StubPart get_gripper() { return {}; }
    };
    std::string detail;
    if (can_feedback_allows_torque(static_cast<StubArm*>(nullptr), "startup", &detail)) {
        return fail("a null arm was allowed to enter a torque loop");
    }
    if (detail.find("startup") == std::string::npos) {
        return fail("null-arm rejection did not name the caller");
    }
    return 0;
}
