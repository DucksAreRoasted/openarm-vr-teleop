#pragma once

#include <controller/gripper_follow.hpp>
#include <controller/s_curve.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class TeachingTrajectory {
public:
    using Pose = std::array<double, 16>;
    struct Sample { double time; Pose position{}, velocity{}; };
    void load(const std::string& path) {
        frames_.clear();
        std::ifstream file(path);
        if (!file) throw std::runtime_error("cannot open trajectory: " + path);
        std::string line;
        if (!std::getline(file, line)) throw std::runtime_error("empty trajectory");
        const auto header = split(line);
        const size_t stride = header.size() == 49 ? 3 : header.size() == 33 ? 2 : 0;
        if (!stride || header[0] != "time") throw std::runtime_error("expected dual-arm time,q0,dq0,...,q15,dq15 CSV");
        for (size_t i = 0; i < 16; ++i) {
            if (header[1+i*stride] != "q"+std::to_string(i) ||
                header[2+i*stride] != "dq"+std::to_string(i) ||
                (stride == 3 && header[3+i*stride] != "tau"+std::to_string(i))) {
                throw std::runtime_error("trajectory columns must be left 8, then right 8");
            }
        }
        size_t row = 1;
        while (std::getline(file, line)) {
            ++row;
            if (line.empty() || line == "\r") continue;
            try {
                const auto cells = split(line);
                if (cells.size() != header.size()) throw std::runtime_error("wrong column count");
                std::vector<double> values;
                for (const auto& cell : cells) {
                    size_t end = 0;
                    const double value = std::stod(cell, &end);
                    if (end != cell.size() || !std::isfinite(value)) throw std::runtime_error("invalid numeric value");
                    values.push_back(value);
                }
                Sample frame{values[0]};
                if (frame.time < 0 || (!frames_.empty() && frame.time <= frames_.back().time)) {
                    throw std::runtime_error("timestamps must increase strictly");
                }
                for (size_t i = 0; i < 16; ++i) frame.position[i] = values[1+i*stride];
                frames_.push_back(frame);
            } catch (const std::exception& error) {
                throw std::runtime_error("CSV row " + std::to_string(row) + ": " + error.what());
            }
        }
        if (frames_.size() < 2) throw std::runtime_error("trajectory needs at least two frames");
        invalidate_spline();
        const double start = frames_.front().time;
        for (auto& frame : frames_) frame.time -= start;
        for (size_t n = 1; n < frames_.size(); ++n) for (size_t i = 0; i < 16; ++i) {
            if (!std::isfinite((frames_[n].position[i]-frames_[n-1].position[i]) /
                              (frames_[n].time-frames_[n-1].time))) {
                throw std::runtime_error("trajectory segment velocity overflow");
            }
        }
    }
    // Nominal stroke is −0.98..0.62 rad (open..closed), or 0..0.044 m in simulation.
    // Encoder readings can sit a few degrees past a stop; clamp those onto the
    // stroke. A larger miss is a unit or recording error and is still rejected.
    size_t convert_grippers(bool source_meters, bool simulation) {
        const double tolerance = source_meters ? 0.002 : 0.08;
        size_t clamped = 0;
        for (auto& frame : frames_) for (size_t i : {7u, 15u}) {
            const double lo = source_meters ? 0 : -.98, hi = source_meters ? .044 : .62;
            double q = frame.position[i];
            if (q < lo - tolerance || q > hi + tolerance) {
                throw std::runtime_error("gripper outside selected source units/stroke");
            }
            if (q < lo || q > hi) {
                q = std::clamp(q, lo, hi);
                ++clamped;
            }
            if (source_meters != simulation) {
                const double ratio = (q - lo) / (hi - lo);
                frame.position[i] = simulation ? .044 * ratio : -.98 + 1.6 * ratio;
            } else {
                frame.position[i] = q;
            }
        }
        invalidate_spline();
        return clamped;
    }
    size_t validate_limits(const Pose& lower, const Pose& upper) {
        size_t normalized = 0;
        for (size_t row = 0; row < frames_.size(); ++row) for (size_t i = 0; i < 16; ++i) {
            const double q = frames_[row].position[i];
            // Drag teaching stores raw encoder readings, which can sit a fraction
            // of a degree past the URDF stop. Clamp that back onto the bound.
            // Larger violations are still rejected. Grippers stay exact.
            const double tolerance = i%8 == 7 ? 0 : 0.01;
            if (q < lower[i] || q > upper[i]) {
                if (q >= lower[i]-tolerance && q <= upper[i]+tolerance) {
                    frames_[row].position[i] = std::clamp(q, lower[i], upper[i]);
                    ++normalized;
                    continue;
                }
                throw std::runtime_error("frame " + std::to_string(row+1) +
                    (i < 8 ? " left joint " : " right joint ") + std::to_string(i%8+1) +
                    " outside replay joint limits: q=" + std::to_string(q) + " limits=[" +
                    std::to_string(lower[i]) + "," + std::to_string(upper[i]) + "]");
            }
        }
        invalidate_spline();
        return normalized;
    }
    // Clamped cubic through every recorded joint position. Knot times stay put,
    // so playback speed matches the recording. End velocity is 0; interior
    // velocity is continuous. The 5 ms replay loop samples this directly.
    Sample sample(double time) const {
        if (frames_.empty() || !std::isfinite(time)) throw std::runtime_error("invalid replay sample");
        ensure_spline();
        if (time <= 0) return endpoint(frames_.front());
        if (time >= duration()) return endpoint(frames_.back());
        auto high = std::upper_bound(frames_.begin(), frames_.end(), time,
                                    [](double t, const Sample& frame) { return t < frame.time; });
        const auto& b = *high;
        const auto& a = *(high-1);
        const size_t index = static_cast<size_t>(high - frames_.begin() - 1);
        const double span = b.time - a.time;
        const double mix = (time - a.time) / span;
        Sample out{time};
        for (size_t i = 0; i < 16; ++i) {
            openarm::scurve::cubic_hermite(a.position[i], b.position[i], slopes_[i][index],
                                           slopes_[i][index + 1], span, mix,
                                           &out.position[i], &out.velocity[i]);
            if (out.position[i] < joint_min_[i] || out.position[i] > joint_max_[i]) {
                out.position[i] = std::clamp(out.position[i], joint_min_[i], joint_max_[i]);
                out.velocity[i] = 0;
            }
        }
        return out;
    }
    double duration() const { return frames_.back().time; }
    size_t size() const { return frames_.size(); }
private:
    static std::vector<std::string> split(const std::string& line) {
        std::vector<std::string> cells;
        std::stringstream stream(line);
        std::string cell;
        while (std::getline(stream, cell, ',')) {
            const auto begin = cell.find_first_not_of(" \t\r");
            const auto end = cell.find_last_not_of(" \t\r");
            cells.push_back(begin == std::string::npos ? "" : cell.substr(begin, end-begin+1));
        }
        if (!line.empty() && line.back() == ',') cells.emplace_back();
        return cells;
    }
    void invalidate_spline() const {
        for (auto& slope : slopes_) slope.clear();
    }
    static Sample endpoint(Sample frame) {
        frame.velocity.fill(0);
        return frame;
    }
    void ensure_spline() const {
        if (slopes_[0].size() == frames_.size()) return;
        std::vector<double> times(frames_.size());
        for (size_t joint = 0; joint < 16; ++joint) {
            joint_min_[joint] = frames_.front().position[joint];
            joint_max_[joint] = joint_min_[joint];
        }
        for (size_t row = 0; row < frames_.size(); ++row) {
            times[row] = frames_[row].time;
            for (size_t joint = 0; joint < 16; ++joint) {
                joint_min_[joint] = std::min(joint_min_[joint], frames_[row].position[joint]);
                joint_max_[joint] = std::max(joint_max_[joint], frames_[row].position[joint]);
            }
        }
        for (size_t joint = 0; joint < 16; ++joint) {
            std::vector<double> values(frames_.size());
            for (size_t row = 0; row < frames_.size(); ++row) values[row] = frames_[row].position[joint];
            slopes_[joint] = openarm::scurve::clamped_cubic_slopes(times, values);
        }
    }
    std::vector<Sample> frames_;
    mutable std::array<std::vector<double>, 16> slopes_{};
    mutable Pose joint_min_{}, joint_max_{};
};

class TeachingReplayReference {
public:
    using Pose = TeachingTrajectory::Pose;
    TeachingReplayReference(Pose initial, Pose lower, Pose upper)
        : position_(initial), lower_(lower), upper_(upper) {}
    // Timed startup samples go straight to the common follower Control, just as
    // in Quest. Its safety filter, torque and feedback checks remain active.
    void follow_startup(const TeachingTrajectory::Sample& goal, double dt) {
        if (!std::isfinite(dt) || dt <= 0 || dt > .05) throw std::runtime_error("replay servo deadline missed");
        for (size_t i = 0; i < 16; ++i) {
            if (!std::isfinite(goal.position[i]) || !std::isfinite(goal.velocity[i]) ||
                goal.position[i] < lower_[i] || goal.position[i] > upper_[i]) {
                throw std::runtime_error("invalid startup reference");
            }
        }
        position_ = goal.position;
        velocity_ = goal.velocity;
        grippers_ = {};
    }
    void step(const TeachingTrajectory::Sample& goal, const Pose& measured,
              const Pose& measured_velocity, double dt, bool aligning) {
        if (!std::isfinite(dt) || dt <= 0 || dt > .05) throw std::runtime_error("replay servo deadline missed");
        for (size_t i = 0; i < 16; ++i) {
            if (!std::isfinite(goal.position[i]) || !std::isfinite(goal.velocity[i]) ||
                !std::isfinite(measured[i]) || !std::isfinite(measured_velocity[i]) ||
                goal.position[i] < lower_[i] || goal.position[i] > upper_[i]) {
                throw std::runtime_error("invalid replay reference");
            }
            const bool grip = i%8 == 7;
            const double vmax = aligning ? .3 : grip ? 1.0 : .6;
            const double amax = aligning ? .6 : grip ? 4.0 : 1.2;
            const double desired = std::clamp((aligning ? 0 : goal.velocity[i]) +
                (aligning ? 2 : 8)*(goal.position[i]-position_[i]), -vmax, vmax);
            velocity_[i] += std::clamp(desired-velocity_[i], -amax*dt, amax*dt);
            const double next = position_[i]+velocity_[i]*dt;
            position_[i] = std::clamp(next, lower_[i], upper_[i]);
            if (next != position_[i]) velocity_[i] = 0;
        }
        for (size_t arm = 0; arm < 2; ++arm) {
            const size_t i = arm*8+7;
            const auto command = grippers_[arm].step(goal.position[i], position_[i], velocity_[i],
                measured[i], measured_velocity[i], lower_[i], upper_[i], dt);
            position_[i] = command.position; velocity_[i] = command.velocity;
        }
    }
    const Pose& position() const { return position_; }
    const Pose& velocity() const { return velocity_; }
private:
    Pose position_, lower_, upper_, velocity_{};
    std::array<GripperFollow, 2> grippers_;
};
