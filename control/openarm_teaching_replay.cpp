#include <controller/mirror_teaching_arm.hpp>
#include <controller/teaching_replay.hpp>
#include <csignal>
#include <poll.h>
#include <filesystem>
#include <iostream>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
using Pose = TeachingTrajectory::Pose;
Pose join(const MirrorTeachingArm::Pose& left, const MirrorTeachingArm::Pose& right) {
    Pose result{};
    std::copy(left.begin(), left.end(), result.begin());
    std::copy(right.begin(), right.end(), result.begin()+8);
    return result;
}
MirrorTeachingArm::Pose half(const Pose& pose, size_t offset) {
    MirrorTeachingArm::Pose result{};
    std::copy_n(pose.begin()+offset, 8, result.begin());
    return result;
}
double arm_error(const Pose& a, const Pose& b) {
    double error = 0;
    for (size_t i = 0; i < 16; ++i) if (i%8 != 7) error = std::max(error, std::abs(a[i]-b[i]));
    return error;
}
}

int main(int argc, char** argv) {
    try {
        bool simulation = true, source_meters = false, check_only = false;
        double speed = .5;
        int repeat = 1, cycle = 1;
        std::string csv, urdf = "assets/v1.urdf", xml, yaml = "config/gravity_comp.yaml";
        std::string left_can = "can1", right_can = "can0";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&]() { if (++i >= argc) throw std::runtime_error("missing " + arg + " value"); return std::string(argv[i]); };
            if (arg == "--csv") csv = value();
            else if (arg == "--urdf") urdf = value();
            else if (arg == "--xml") xml = value();
            else if (arg == "--gravity-yaml") yaml = value();
            else if (arg == "--left-can") left_can = value();
            else if (arg == "--right-can") right_can = value();
            else if (arg == "--speed") speed = std::stod(value());
            else if (arg == "--repeat") {
                const auto text = value();
                size_t end = 0;
                repeat = std::stoi(text, &end);
                if (end != text.size() || repeat < 0) throw std::runtime_error("repeat must be a nonnegative integer");
            }
            else if (arg == "--source-meters") source_meters = true;
            else if (arg == "--sim") simulation = true;
            else if (arg == "--real") simulation = false;
            else if (arg == "--check") check_only = true;
            else throw std::runtime_error("unknown replay option: " + arg);
        }
        if (csv.empty() || !std::isfinite(speed) || speed < .1 || speed > 1) {
            throw std::runtime_error("--csv is required; speed must be 0.1..1.0");
        }
        if (!simulation && (left_can.empty() || right_can.empty() || left_can == right_can)) {
            throw std::runtime_error("two distinct CAN interfaces are required");
        }
        // Validate the entire input offline, before constructing/enabling hardware.
        TeachingTrajectory trajectory;
        trajectory.load(csv);
        trajectory.convert_grippers(source_meters, simulation);
        Pose lower{}, upper{};
        for (size_t arm = 0; arm < 2; ++arm) {
            const std::string side = arm == 0 ? "left" : "right";
            Dynamics dynamics(urdf, "openarm_body_link0", "openarm_"+side+"_link7");
            std::vector<double> lo, hi;
            if (!dynamics.Init() || !dynamics.GetJointPositionLimits(&lo, &hi) || lo.size() != 7) {
                throw std::runtime_error("cannot read replay URDF limits");
            }
            std::copy(lo.begin(), lo.end(), lower.begin()+arm*8);
            std::copy(hi.begin(), hi.end(), upper.begin()+arm*8);
            lower[arm*8+7] = simulation ? 0 : -.98;
            upper[arm*8+7] = simulation ? .044 : .62;
        }
        const size_t normalized = trajectory.validate_limits(lower, upper);
        if (normalized) {
            std::cout << "[replay] NORMALIZED boundary_values=" << normalized
                      << " tolerance_rad=1e-6 (CSV rounding; exact URDF bounds restored)" << std::endl;
        }
        std::cout << "[replay] CHECKED frames=" << trajectory.size() << " duration=" << trajectory.duration()
                  << " speed=" << speed << " (recorded torques ignored; gravity recomputed)" << std::endl;
        if (check_only) return 0;
        GravityTune lt, rt;
        GravityModel model;
        if (!load_gravity_yaml(yaml, &lt, &rt, &model)) throw std::runtime_error("cannot load gravity configuration");
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        std::shared_ptr<MujocoSimWorld> world;
        if (simulation) {
            if (xml.empty()) throw std::runtime_error("simulation requires --xml");
            world = std::make_shared<MujocoSimWorld>(xml, "OpenArm teaching replay");
        }
        MirrorTeachingArm left("left", left_can, urdf, lt, model, world, true);
        if (!running) return 0;
        MirrorTeachingArm right("right", right_can, urdf, rt, model, world, true);
        TeachingReplayReference reference(join(left.pose(), right.pose()), lower, upper);
        bool aligning = true, paused = false, complete = false;
        double position = 0, phase_time = 0, settled = 0;
        auto previous = std::chrono::steady_clock::now()-std::chrono::milliseconds(5);
        auto next = std::chrono::steady_clock::now(), next_log = next;
        std::string input;
        std::cout << "[replay] ALIGNING: slow transition from measured pose to first frame" << std::endl;
        while (running) {
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now-previous).count();
            previous = now;
            // Poll without changing inherited terminal flags used by the shell.
            pollfd commands{STDIN_FILENO, POLLIN, 0};
            if (poll(&commands, 1, 0) > 0 && (commands.revents & POLLIN)) {
                char buffer[256];
                const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
                if (count > 0) input.append(buffer, static_cast<size_t>(count));
                for (size_t end; (end = input.find('\n')) != std::string::npos;) {
                    const auto command = input.substr(0, end); input.erase(0, end+1);
                    if (command == "pause") paused = true;
                    if (command == "resume") paused = false;
                    if (command == "stop") running = 0;
                }
            }
            if (!running) break;
            left.read(); right.read();
            auto goal = trajectory.sample(position);
            const bool catching_up = arm_error(goal.position, reference.position()) > .03;
            for (auto& v : goal.velocity) v *= (!aligning && !paused && !complete && !catching_up) ? speed : 0;
            const auto measured = join(left.pose(), right.pose());
            if (paused) {
                goal.position = reference.position();
                goal.velocity.fill(0);
            }
            reference.step(goal, measured, join(left.velocity(), right.velocity()), dt, aligning);
            left.send(half(reference.position(), 0), half(reference.velocity(), 0), true);
            right.send(half(reference.position(), 8), half(reference.velocity(), 8), true);
            if (!paused) phase_time += dt;
            if (aligning && !paused) {
                if (phase_time > 120) throw std::runtime_error("alignment timed out");
                if (arm_error(goal.position, reference.position()) < .015 && arm_error(goal.position, measured) < .12) {
                    aligning = false; phase_time = 0;
                    std::cout << "[replay] PLAYING" << std::endl;
                }
            } else if (!paused && !complete) {
                if (!catching_up) position = std::min(trajectory.duration(), position+dt*speed);
                if (position >= trajectory.duration()) {
                    const bool at_end = arm_error(goal.position, reference.position()) < .015 && arm_error(goal.position, measured) < .12;
                    settled = at_end ? settled+dt : 0;
                    if (settled > .3) {
                        if (repeat == 0 || cycle < repeat) {
                            ++cycle;
                            position = 0; phase_time = 0; settled = 0;
                            aligning = true;
                            std::cout << "[replay] ALIGNING: cycle=" << cycle
                                      << " slow return to first frame" << std::endl;
                        } else {
                            complete = true;
                            std::cout << "[replay] COMPLETE: holding final pose; Stop releases both arms" << std::endl;
                        }
                    }
                }
            }
            if (now >= next_log) {
                std::cout << "[replay] PROGRESS time=" << position << " duration=" << trajectory.duration()
                          << " phase=" << (complete ? "complete" : paused ? "paused" : aligning ? "aligning" : "playing")
                          << " error=" << arm_error(goal.position, measured) << std::endl;
                next_log = now+std::chrono::milliseconds(500);
            }
            if (world) { world->forward_and_render(); if (world->window_should_close()) break; }
            next += std::chrono::milliseconds(5);
            std::this_thread::sleep_until(next);
        }
        left.stop(); right.stop();
        std::cout << "[replay] STOPPED: both arms disabled" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[replay][ERROR] " << error.what() << std::endl;
        return 1;
    }
}
