#pragma once

#include <controller/teaching_replay.hpp>
#include <controller/s_curve.hpp>
#include <controller/replay_feedback.hpp>
#include <io/teleop_io.hpp>

// Shared Quest/replay startup timeline. Motor radians are the source units.
// Connections use CsvReplay's S-curve; fast sections share a local time map.
class StartupTrajectory {
public:
    using Pose = TeachingTrajectory::Pose;
    void reset(const std::string& path, const Pose& current, bool simulation,
               double transition_sec = 5.0, double hold_sec = .6) {
        if (!std::isfinite(transition_sec) || transition_sec < 0 || transition_sec > 10 ||
            !std::isfinite(hold_sec)) throw std::runtime_error("invalid startup timing");
        TeachingTrajectory validation;
        validation.load(path);
        validation.convert_grippers(false, simulation);
        simulation_ = simulation;
        hold_ = std::clamp(hold_sec, 0.0, 5.0);
        if (!replay_.load(path, 16)) throw std::runtime_error("cannot load startup trajectory: "+path);
        std::vector<double> speeds(16);
        for (size_t i = 0; i < speeds.size(); ++i) {
            // Actual speed can overshoot recorded dq; leave feedback-stop
            // headroom instead of repeatedly raising the hard stop threshold.
            speeds[i] = .65*replay_feedback_speed_limit(i%8);
        }
        replay_.retime_speed_limits(speeds);
        std::vector<JointState> initial(16);
        for (size_t i = 0; i < 16; ++i) {
            initial[i].position = simulation && i%8 == 7 ? current[i]/.044*1.6-.98 : current[i];
        }
        if (replay_.prepend_transition_from_with_result(initial, 16, transition_sec) == CsvReplay::TransitionResult::Failed) {
            throw std::runtime_error("cannot connect startup trajectory");
        }
    }
    double duration() const { return replay_.duration()+hold_; }
    TeachingTrajectory::Sample sample(double time) {
        auto joints = replay_.sample_at(std::clamp(time, 0.0, replay_.duration()));
        if (!joints) throw std::runtime_error("startup has no sample at requested time");
        TeachingTrajectory::Sample result{};
        result.time = time;
        for (size_t i = 0; i < 16; ++i) {
            result.position[i] = (*joints)[i].position;
            result.velocity[i] = time >= replay_.duration() ? 0 : (*joints)[i].velocity;
            if (simulation_ && i%8 == 7) {
                result.position[i] = (result.position[i]+.98)/1.6*.044;
                result.velocity[i] *= .044/1.6;
            }
        }
        return result;
    }
private:
    CsvReplay replay_;
    bool simulation_ = false;
    double hold_ = .6;
};
