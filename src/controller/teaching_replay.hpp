#pragma once

#include <controller/gripper_follow.hpp>
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
        const double start = frames_.front().time;
        for (auto& frame : frames_) frame.time -= start;
        for (size_t n = 1; n < frames_.size(); ++n) for (size_t i = 0; i < 16; ++i) {
            if (!std::isfinite((frames_[n].position[i]-frames_[n-1].position[i]) /
                              (frames_[n].time-frames_[n-1].time))) {
                throw std::runtime_error("trajectory segment velocity overflow");
            }
        }
    }
    void convert_grippers(bool source_meters, bool simulation) {
        for (auto& frame : frames_) for (size_t i : {7u, 15u}) {
            const double lo = source_meters ? 0 : -.98, hi = source_meters ? .044 : .62;
            const double q = frame.position[i];
            if (q < lo || q > hi) throw std::runtime_error("gripper outside selected source units/stroke");
            if (source_meters != simulation) {
                const double ratio = (q-lo)/(hi-lo);
                frame.position[i] = simulation ? .044*ratio : -.98+1.6*ratio;
            }
        }
    }
    size_t validate_limits(const Pose& lower, const Pose& upper) {
        size_t normalized = 0;
        for (size_t row = 0; row < frames_.size(); ++row) for (size_t i = 0; i < 16; ++i) {
            const double q = frames_[row].position[i];
            // The recorder rounds arm positions to six decimal places. Accept
            // only that quantization error, then restore the exact URDF bound.
            const double tolerance = i%8 == 7 ? 0 : 1e-6;
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
        return normalized;
    }
    Sample sample(double time) const {
        if (frames_.empty() || !std::isfinite(time)) throw std::runtime_error("invalid replay sample");
        if (time <= 0) return frames_.front();
        if (time >= duration()) return frames_.back();
        auto high = std::upper_bound(frames_.begin(), frames_.end(), time,
                                    [](double t, const Sample& frame) { return t < frame.time; });
        const auto& b = *high;
        const auto& a = *(high-1);
        const double interval = b.time-a.time, alpha = (time-a.time)/interval;
        Sample out{time};
        for (size_t i = 0; i < 16; ++i) {
            out.position[i] = a.position[i]+alpha*(b.position[i]-a.position[i]);
            out.velocity[i] = (b.position[i]-a.position[i])/interval;
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
    std::vector<Sample> frames_;
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
