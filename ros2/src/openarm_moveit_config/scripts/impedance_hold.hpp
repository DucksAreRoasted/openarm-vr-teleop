#pragma once

#include <array>
#include <cmath>

struct HoldIntegralState {
    std::array<double, 7> tau{};
    std::array<double, 7> frozen{};
    double decay_s = 0.0;

    void reset() {
        tau.fill(0.0);
        frozen.fill(0.0);
        decay_s = 0.0;
    }

    void freeze() { frozen = tau; }
};

// A bounded, position-dependent spring: no accumulating state or release gate.
// Near the target its added stiffness is cap/width; far away force saturates.
inline double nonlinear_spring_torque(double error, double cap, double width) {
    return cap == 0.0 ? 0.0 : cap * std::tanh(error / width);
}
