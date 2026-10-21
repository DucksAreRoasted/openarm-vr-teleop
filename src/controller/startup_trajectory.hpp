#pragma once

#include <controller/teaching_replay.hpp>
#include <controller/s_curve.hpp>
#include <controller/replay_feedback.hpp>
#include <io/teleop_io.hpp>

#include <array>
#include <iostream>

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
        if (file_is_mirror()) {
            // A joint-space blend from an unmatched right arm swings both hands
            // to the same side. Gather the right arm onto the left arm's mirror
            // first, then enter the file. Both ends of that second blend are
            // mirrors, so the path stays one.
            std::vector<JointState> synced = initial;
            mirror_right_onto(&synced);
            if (replay_.prepend_transition_from_with_result(synced, 16, transition_sec) == CsvReplay::TransitionResult::Failed ||
                replay_.prepend_transition_from_with_result(initial, 16, transition_sec) == CsvReplay::TransitionResult::Failed) {
                throw std::runtime_error("cannot connect startup trajectory");
            }
            std::cout << "[startup] right arm gathers onto the mirror of the left, then follows it" << std::endl;
        } else if (replay_.prepend_transition_from_with_result(initial, 16, transition_sec) == CsvReplay::TransitionResult::Failed) {
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
    // j1 j2 j3 j4 j5 j6 j7 gripper. j3 flips with the shoulders; leaving it
    // the same swings both forearms to one side.
    static constexpr std::array<double, 8> kMirrorSign{-1, -1, -1, 1, -1, -1, -1, 1};

    bool file_is_mirror() {
        auto matches = [&](double time) {
            const auto sample = replay_.sample_at(time);
            if (!sample) return false;
            for (size_t joint = 0; joint < kMirrorSign.size(); ++joint) {
                if (std::abs((*sample)[8 + joint].position - kMirrorSign[joint] * (*sample)[joint].position) > 1e-3) {
                    return false;
                }
            }
            return true;
        };
        const double end = replay_.duration();
        return matches(0) && matches(end * 0.5) && matches(end);
    }

    static void mirror_right_onto(std::vector<JointState>* pose) {
        for (size_t joint = 0; joint < kMirrorSign.size(); ++joint) {
            (*pose)[8 + joint].position = kMirrorSign[joint] * (*pose)[joint].position;
            (*pose)[8 + joint].velocity = kMirrorSign[joint] * (*pose)[joint].velocity;
        }
    }

    CsvReplay replay_;
    bool simulation_ = false;
    double hold_ = .6;
};
