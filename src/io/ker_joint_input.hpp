#pragma once

#include <controller/teaching_replay.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

// Local USB bridge protocol: KER2,sequence,monotonic_seconds,left_pressed,right_pressed,left[8],right[8].
// Positions use follower radians, including the V1 gripper angle (-.98 .. .62).
class KerJointInput {
public:
    using Pose = TeachingTrajectory::Pose;
    static double clock_seconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    bool accept(const std::string& packet, double now, const Pose& lower, const Pose& upper,
                bool simulation) {
        std::stringstream stream(packet);
        std::string cell;
        auto next = [&]() {
            if (!std::getline(stream, cell, ',')) throw std::runtime_error("KER: truncated packet");
            return cell;
        };
        if (next() != "KER2") throw std::runtime_error("KER: invalid packet version (KER2 trigger flags required)");
        const auto seq_text = next();
        if (seq_text.empty() || seq_text.find_first_not_of("0123456789") != std::string::npos)
            throw std::runtime_error("KER: invalid sequence");
        const auto sequence = std::stoull(seq_text);
        auto number = [&]() {
            const auto value = next();
            size_t end = 0;
            const double result = std::stod(value, &end);
            if (end != value.size() || !std::isfinite(result)) throw std::runtime_error("KER: invalid number");
            return result;
        };
        const double stamp = number();
        auto pressed = [&]() {
            const auto value = next();
            if (value != "0" && value != "1") throw std::runtime_error("KER: invalid trigger flag");
            return value == "1";
        };
        const bool left_pressed = pressed(), right_pressed = pressed();
        Pose target{};
        std::ostringstream violations;
        violations << std::setprecision(9);
        for (size_t i = 0; i < target.size(); ++i) {
            double q = number();
            if (i%8 == 7) {
                if (q < -.98 || q > .62) throw std::runtime_error("KER: gripper outside stroke");
                if (simulation) q = (q+.98)/1.6*.044;
            }
            if (q < lower[i]-1e-6 || q > upper[i]+1e-6) {
                if (violations.tellp() > 0) violations << "; ";
                violations << (i < 8 ? "left" : "right") << " joint " << i%8+1
                           << " q=" << q << " rad (" << q*180/3.141592653589793 << " deg)"
                           << " limits=[" << lower[i] << "," << upper[i] << "] rad";
            }
            target[i] = std::clamp(q, lower[i], upper[i]);
        }
        if (std::getline(stream, cell, ',') || (!packet.empty() && packet.back() == ','))
            throw std::runtime_error("KER: extra packet columns");
        // Queued, replayed or reordered packets must not refresh the watchdog.
        if (now-stamp > .25 || stamp-now > .01 ||
            (received_ && (sequence <= sequence_ || stamp <= stamp_))) return false;
        const auto violation = violations.str();
        if (!violation.empty() && armed_)
            throw std::runtime_error("KER: target outside URDF limits: " + violation);
        pose_error_ = violation;
        if (!violation.empty()) {
            // The user may still be positioning an unpowered KER at startup.
            // Accept its heartbeat, but never arm or expose the clipped target.
            holding_ = false;
            stamp_ = stamp;
            sequence_ = sequence;
            received_ = true;
            return true;
        }
        // Arming is latched for this controller session only. Gaps, one released
        // trigger or reordered packets cannot contribute to the continuous hold.
        if (!armed_) {
            if (!received_ || stamp-stamp_ > .25 || !left_pressed || !right_pressed)
                holding_ = false;
            if (left_pressed && right_pressed) {
                if (!holding_) { holding_ = true; hold_start_ = stamp; }
                if (stamp-hold_start_ >= 2.0) armed_ = true;
            }
        }
        target_ = target;
        stamp_ = stamp;
        sequence_ = sequence;
        received_ = true;
        return true;
    }
    bool fresh(double now) const { return received_ && now-stamp_ <= .25; }
    bool armed() const { return armed_; }
    double arming_hold_seconds(double now) const {
        return holding_ && fresh(now) ? std::clamp(stamp_-hold_start_, 0.0, 2.0) : 0.0;
    }
    bool pose_valid() const { return received_ && pose_error_.empty(); }
    const std::string& pose_error() const { return pose_error_; }
    const Pose& target() const { return target_; }
private:
    Pose target_{};
    double stamp_ = 0;
    uint64_t sequence_ = 0;
    bool received_ = false;
    bool holding_ = false, armed_ = false;
    double hold_start_ = 0;
    std::string pose_error_;
};
