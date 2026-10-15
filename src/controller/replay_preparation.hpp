#pragma once

#include <controller/startup_trajectory.hpp>
#include <utility>

// Preparation follows the user's complete startup CSV, with smooth connections
// at either end on initial preparation; later returns skip the startup CSV.
class ReplayPreparation {
public:
    using Pose = TeachingTrajectory::Pose;
    explicit ReplayPreparation(std::string startup_file = {}, double transition_sec = 5.0, double hold_sec = .6)
        : startup_file_(std::move(startup_file)), transition_sec_(transition_sec), hold_sec_(hold_sec) {}

    void reset(const Pose& current, const Pose& target, bool simulation, bool include_startup = true) {
        target_ = target;
        body_ = 0;
        last_ = current;
        if (include_startup && !startup_file_.empty()) {
            startup_.reset(startup_file_, current, simulation, transition_sec_, hold_sec_);
            body_ = startup_.duration();
            last_ = startup_.sample(body_).position;
        }
        exit_ = transition_duration(last_, target_, simulation, !startup_file_.empty());
    }
    double duration() const { return body_ + exit_; }
    bool startup_active(double time) const { return !startup_file_.empty() && time < body_; }
    const char* phase(double time) const { return startup_active(time) ? "startup" : "connecting"; }
    double startup_duration() const { return body_; }
    TeachingTrajectory::Sample sample(double time) {
        if (startup_active(time)) return startup_.sample(time);
        time -= body_;
        return blend(last_, target_, time, exit_);
    }

private:
    static double transition_duration(const Pose& from, const Pose& to, bool simulation, bool recorded_path) {
        double duration = 0;
        for (size_t i = 0; i < from.size(); ++i) {
            const bool grip = i%8 == 7;
            const double distance = std::abs(to[i]-from[i]);
            // Use existing playback limits for recorded startup paths. Legacy
            // direct first-frame alignment retains its conservative limits.
            const double velocity = grip ? (simulation ? .02 : .7) : (recorded_path ? .6 : .3);
            const double acceleration = grip ? (simulation ? .04 : 1.4) : (recorded_path ? 1.2 : .6);
            duration = std::max({duration, 1.875*distance/velocity,
                                std::sqrt(5.78*distance/acceleration)});
        }
        return duration;
    }
    static TeachingTrajectory::Sample blend(const Pose& from, const Pose& to,
                                            double time, double duration) {
        const double u = duration > 1e-6 ? std::clamp(time/duration, 0.0, 1.0) : 1;
        const double weight = u*u*u*(10+u*(-15+6*u));
        const double velocity = duration > 1e-6 ? 30*u*u*(1-u)*(1-u)/duration : 0;
        TeachingTrajectory::Sample result{};
        result.time = time;
        for (size_t i = 0; i < from.size(); ++i) {
            result.position[i] = from[i]+weight*(to[i]-from[i]);
            result.velocity[i] = velocity*(to[i]-from[i]);
        }
        return result;
    }
    std::string startup_file_;
    StartupTrajectory startup_;
    double transition_sec_, hold_sec_;
    Pose target_{}, last_{};
    double body_ = 0, exit_ = 0;
};
