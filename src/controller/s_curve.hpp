// Rest-to-rest jerk-limited S-curve (7-segment, with short-move degeneration).
// Header-only so teleop core and MoveIt real_arm can share it.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace openarm {
namespace scurve {

struct Limits {
    double v_max = 0.60;  // rad/s
    double a_max = 1.20;  // rad/s^2
    double j_max = 6.00;  // rad/s^3
};

inline double env_double(const char* name, double fallback, double lo, double hi) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, lo, hi);
}

inline Limits limits_from_env() {
    Limits lim;
    lim.v_max = env_double("OPENARM_SCURVE_VMAX", lim.v_max, 0.02, 5.0);
    lim.a_max = env_double("OPENARM_SCURVE_AMAX", lim.a_max, 0.05, 20.0);
    lim.j_max = env_double("OPENARM_SCURVE_JMAX", lim.j_max, 0.5, 200.0);
    return lim;
}

inline bool env_flag_enabled(const char* name, bool default_on) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return default_on;
    }
    if (raw[0] == '0' && raw[1] == '\0') {
        return false;
    }
    if ((raw[0] == 'n' || raw[0] == 'N') && (raw[1] == 'o' || raw[1] == 'O') && raw[2] == '\0') {
        return false;
    }
    if ((raw[0] == 'o' || raw[0] == 'O') && (raw[1] == 'f' || raw[1] == 'F') &&
        (raw[2] == 'f' || raw[2] == 'F') && raw[3] == '\0') {
        return false;
    }
    return true;
}

// Symmetric rest-to-rest profile for one DoF.
class Profile1D {
public:
    void plan(double q0, double q1, const Limits& lim_in) {
        q0_ = q0;
        const double d_raw = q1 - q0;
        dir_ = (d_raw >= 0.0) ? 1.0 : -1.0;
        const double d = std::abs(d_raw);
        if (d < 1e-12) {
            Tj_ = Ta_ = Tv_ = 0.0;
            v_ = a_ = 0.0;
            T_ = 0.0;
            return;
        }

        Limits lim = lim_in;
        lim.v_max = std::max(lim.v_max, 1e-6);
        lim.a_max = std::max(lim.a_max, 1e-6);
        lim.j_max = std::max(lim.j_max, 1e-6);

        double Tj = lim.a_max / lim.j_max;
        double Ta = 0.0;
        double v = lim.v_max;
        double a = lim.a_max;
        const double v_aj = lim.a_max * lim.a_max / lim.j_max;

        if (lim.v_max > v_aj) {
            Ta = lim.v_max / lim.a_max - Tj;
            if (Ta < 0.0) {
                Ta = 0.0;
            }
        } else {
            Tj = std::sqrt(lim.v_max / lim.j_max);
            Ta = 0.0;
            a = lim.j_max * Tj;
            v = lim.v_max;
        }

        double d_ad = v * (2.0 * Tj + Ta);
        double Tv = 0.0;
        if (d > d_ad) {
            Tv = (d - d_ad) / v;
        } else {
            Tv = 0.0;
            const double A = 1.0 / lim.a_max;
            const double B = lim.a_max / lim.j_max;
            const double disc = B * B + 4.0 * A * d;
            v = (-B + std::sqrt(std::max(disc, 0.0))) / (2.0 * A);
            Ta = v / lim.a_max - B;
            Tj = B;
            a = lim.a_max;
            if (Ta < 0.0 || v <= v_aj + 1e-12) {
                Tj = std::cbrt(d / (2.0 * lim.j_max));
                Ta = 0.0;
                a = lim.j_max * Tj;
                v = lim.j_max * Tj * Tj;
            }
        }

        Tj_ = std::max(Tj, 0.0);
        Ta_ = std::max(Ta, 0.0);
        Tv_ = std::max(Tv, 0.0);
        v_ = std::max(v, 0.0);
        a_ = std::max(a, 0.0);
        T_ = 4.0 * Tj_ + 2.0 * Ta_ + Tv_;
    }

    double duration() const { return T_; }

    void evaluate(double t, double* q, double* dq) const {
        if (T_ <= 1e-12) {
            if (q) {
                *q = q0_;
            }
            if (dq) {
                *dq = 0.0;
            }
            return;
        }
        t = std::clamp(t, 0.0, T_);
        const double j = (Tj_ > 1e-12) ? (a_ / Tj_) : 0.0;

        const double segments_T[] = {Tj_, Ta_, Tj_, Tv_, Tj_, Ta_, Tj_};
        const double segments_J[] = {+j, 0.0, -j, 0.0, -j, 0.0, +j};

        double p = 0.0;
        double vel = 0.0;
        double acc = 0.0;
        double elapsed = 0.0;
        for (int i = 0; i < 7; ++i) {
            const double seg = segments_T[i];
            if (seg <= 1e-15) {
                continue;
            }
            const double j_seg = segments_J[i];
            if (t >= elapsed + seg - 1e-15) {
                const double dt = seg;
                const double a0 = acc;
                const double v0 = vel;
                const double p0 = p;
                p = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * j_seg * dt * dt * dt;
                vel = v0 + a0 * dt + 0.5 * j_seg * dt * dt;
                acc = a0 + j_seg * dt;
                elapsed += seg;
            } else {
                const double dt = t - elapsed;
                const double a0 = acc;
                const double v0 = vel;
                const double p0 = p;
                p = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * j_seg * dt * dt * dt;
                vel = v0 + a0 * dt + 0.5 * j_seg * dt * dt;
                break;
            }
        }

        if (q) {
            *q = q0_ + dir_ * p;
        }
        if (dq) {
            *dq = dir_ * vel;
        }
    }

private:
    double q0_ = 0.0;
    double dir_ = 1.0;
    double Tj_ = 0.0;
    double Ta_ = 0.0;
    double Tv_ = 0.0;
    double v_ = 0.0;
    double a_ = 0.0;
    double T_ = 0.0;
};

// Multi-DoF: plan each joint, sync to max duration via time stretch.
class ProfileMulti {
public:
    void plan(const double* q0, const double* q1, size_t n, const Limits& lim) {
        joints_.assign(n, Profile1D{});
        stretch_.assign(n, 1.0);
        T_ = 0.0;
        for (size_t i = 0; i < n; ++i) {
            joints_[i].plan(q0[i], q1[i], lim);
            T_ = std::max(T_, joints_[i].duration());
        }
        if (T_ <= 1e-12) {
            return;
        }
        for (size_t i = 0; i < n; ++i) {
            const double Ti = joints_[i].duration();
            stretch_[i] = (Ti > 1e-12) ? (Ti / T_) : 0.0;
        }
    }

    double duration() const { return T_; }

    void sample(double t, double* q, double* dq) const {
        t = std::clamp(t, 0.0, std::max(T_, 0.0));
        for (size_t i = 0; i < joints_.size(); ++i) {
            const double Ti = joints_[i].duration();
            double qi = 0.0;
            double dqi = 0.0;
            if (Ti <= 1e-12) {
                joints_[i].evaluate(0.0, &qi, &dqi);
                dqi = 0.0;
            } else {
                const double local_t = t * stretch_[i];
                joints_[i].evaluate(local_t, &qi, &dqi);
                dqi *= stretch_[i];
            }
            if (q) {
                q[i] = qi;
            }
            if (dq) {
                dq[i] = dqi;
            }
        }
    }

private:
    std::vector<Profile1D> joints_;
    std::vector<double> stretch_;
    double T_ = 0.0;
};

// Unit-interval S-shape (C2, zero end vel/acc) when keeping MoveIt knot times.
inline double unit_smooth(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0);
}

inline double unit_smooth_dot(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 30.0 * s * s * (s * (s - 2.0) + 1.0);
}

}  // namespace scurve
}  // namespace openarm
