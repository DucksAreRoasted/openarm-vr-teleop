// Classifies Damiao motor feedback the way mirror teaching already does.
// Does not open a socket. Teleop / zero-g / MoveIt do not call this yet;
// those paths are skipped from the Python wrapper until that PR lands.
//
// error_code > kCanErrorCodeMaxOk is 待硬件确认: 0 and 1 stay acceptable.

#include "safety_contract.hpp"

#include <openarm/damiao_motor/dm_motor.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {

using openarm::damiao_motor::Motor;
using openarm::damiao_motor::MotorType;

// update_state() is protected. A test subclass is the public way to inject a reply.
class ProbeMotor : public Motor {
public:
    using Motor::Motor;
    void inject(double q, double dq, double tau, int tmos, int trotor, int error_code) {
        update_state(q, dq, tau, tmos, trotor, error_code);
    }
};

bool feedback_invalid(const Motor& motor) {
    const bool missing = motor.last_state_received().time_since_epoch().count() == 0;
    double age_ms = -1.0;
    if (!missing) {
        age_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                           motor.last_state_received())
                     .count();
    }
    return missing || age_ms > kCanFeedbackMaxAgeMs || motor.get_error_code() > kCanErrorCodeMaxOk;
}

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

}  // namespace

int main() {
    ProbeMotor never_heard(MotorType::DM4310, 0x01, 0x11);
    if (!feedback_invalid(never_heard)) {
        return fail("a motor that never replied was treated as valid");
    }

    ProbeMotor healthy(MotorType::DM4310, 0x01, 0x11);
    healthy.inject(0.2, 0.0, 0.0, 30, 31, 0);
    if (feedback_invalid(healthy)) {
        return fail("a fresh reply with error_code 0 was rejected");
    }

    ProbeMotor status_one(MotorType::DM4310, 0x01, 0x11);
    status_one.inject(0.2, 0.0, 0.0, 30, 31, kCanErrorCodeMaxOk);
    if (feedback_invalid(status_one)) {
        return fail("error_code == kCanErrorCodeMaxOk was rejected");
    }

    // 待硬件确认: the temporary rule rejects every code above 1.
    ProbeMotor faulted(MotorType::DM4310, 0x01, 0x11);
    faulted.inject(0.2, 0.0, 0.0, 30, 31, kCanErrorCodeMaxOk + 1);
    if (!feedback_invalid(faulted)) {
        return fail("error_code > kCanErrorCodeMaxOk was accepted");
    }

    ProbeMotor stale(MotorType::DM4310, 0x01, 0x11);
    stale.inject(0.2, 0.0, 0.0, 30, 31, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(kCanFeedbackMaxAgeMs) + 80));
    if (!feedback_invalid(stale)) {
        return fail("feedback older than the age limit was accepted");
    }
    return 0;
}
