#include "impedance_hold.hpp"
#include "impedance_profile.hpp"
#include <controller/gravity_comp.hpp>

#include <cassert>
#include <iostream>
#include <limits>

// Firmware PD runs continuously; the extra spring is updated by the 100 Hz host.
// Static friction is deliberately larger than the baseline force at 0.02 rad.
double simulate(const ImpedanceProfile& profile, bool enabled, double inertia, int sign) {
    double q = 0, v = 0, extra = 0;
    double tail_min = 1e9, tail_max = -1e9;
    for (int tick = 0; tick < 60000; ++tick) {
        if (tick == 6000) q = sign * 0.02;
        if (tick % 20 == 0) {
            extra = nonlinear_spring_torque(-q, enabled ? profile.restore_tau_max[5] : 0,
                                             profile.restore_width);
        }
        const double kd = profile.kd[5] + (enabled ? profile.restore_kd[5] : 0);
        const double drive = -profile.kp[5] * q - kd * v + extra;
        const double stiction = profile.name == "P2" ? 0.16 : 0.12;
        if (std::abs(v) < 0.0005 && std::abs(drive) <= stiction) {
            v = 0;
        } else {
            const double friction = std::copysign(stiction * 0.7,
                std::abs(v) > 0.0005 ? v : drive);
            v += (drive - friction) / inertia * 0.0005;
        }
        q += v * 0.0005;
        assert(std::isfinite(q) && std::abs(q) < 0.1);
        assert(std::abs(extra) <= profile.restore_tau_max[5] + 1e-12);
        if (tick > 50000) {
            tail_min = std::min(tail_min, q);
            tail_max = std::max(tail_max, q);
        }
    }
    assert(tail_max - tail_min < 0.0005);  // no repeated slip in final five seconds
    return std::abs(q);
}

int main(int argc, char** argv) {
    assert(argc == 2);
    const auto profiles = load_impedance_profiles(argv[1]);
    HoldIntegralState integral;
    integral.tau[5] = 0.4;
    integral.freeze();
    integral.decay_s = 1;
    integral.reset();  // new trajectory / hold here / soft stop
    integral.tau = integral.frozen;  // next Frozen control cycle
    for (double value : integral.tau) assert(value == 0);
    assert(integral.decay_s == 0);
    integral.tau[5] = -0.15;
    integral.freeze();
    assert(integral.frozen[5] == -0.15);  // switching at the same pose remains supported

    GravityTune tune;
    tune.scale[5] = 1.052632;
    tune.bias[5] = 0.06;
    GravityModel model;
    model.ff_gain = 0.95;
    model.tracking_ff_gain = 1.2;  // independent tracking setting must not enter gravity term
    assert(compose_gravity_torque(0.8, tune, 5, model.ff_gain) ==
           compose_tau_ff(0.8, tune, 5, 0, model.ff_gain));

    for (const auto* name : {"P2", "P3"}) {
        const auto& p = profiles.at(name);
        assert(p.restore_tau_max == builtin_impedance_profiles().at(name).restore_tau_max);
        assert(nonlinear_spring_torque(0, p.restore_tau_max[5], p.restore_width) == 0);
        double previous = -1;
        for (int i = -100; i <= 100; ++i) {
            const double error = i * 0.01;
            const double force = nonlinear_spring_torque(error, p.restore_tau_max[5], p.restore_width);
            assert(std::abs(force) <= p.restore_tau_max[5]);
            assert(force >= previous && force * error >= 0);
            assert(force == -nonlinear_spring_torque(-error, p.restore_tau_max[5], p.restore_width));
            previous = force;
        }
        // Constant external displacement cannot build torque over time.
        const double held = nonlinear_spring_torque(0.2, p.restore_tau_max[5], p.restore_width);
        for (int i = 0; i < 10000; ++i) {
            assert(nonlinear_spring_torque(0.2, p.restore_tau_max[5], p.restore_width) == held);
        }
        assert(nonlinear_spring_torque(0.2, 0, p.restore_width) == 0);
        for (double inertia : {0.002, 0.01, 0.04}) {
            for (int sign : {-1, 1}) {
                const double before = simulate(p, false, inertia, sign);
                const double after = simulate(p, true, inertia, sign);
                std::cout << name << " inertia=" << inertia << " sign=" << sign
                          << " before=" << before << " after=" << after << " rad\n";
                assert(after < before * 0.75);
            }
        }
        for (int fault = 0; fault < 4; ++fault) {
            auto invalid = p;
            if (fault == 0) invalid.restore_tau_max[5] = 0.51;
            if (fault == 1) invalid.restore_kd[5] = 5;
            if (fault == 2) invalid.restore_width = 0;
            if (fault == 3) invalid.restore_width = std::numeric_limits<double>::quiet_NaN();
            bool rejected = false;
            try { validate_impedance_profile(invalid); }
            catch (const std::runtime_error&) { rejected = true; }
            assert(rejected);
        }
    }
    for (const auto* name : {"P0", "P1", "P-stop", "drag"}) {
        for (double cap : profiles.at(name).restore_tau_max) assert(cap == 0);
    }
}
