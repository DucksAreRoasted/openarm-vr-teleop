#include <controller/replay_arm.hpp>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <controller/teaching_replay.hpp>
#include <controller/replay_preparation.hpp>
#include <controller/replay_preparation_job.hpp>
#include <csignal>
#include <poll.h>
#include <filesystem>
#include <iostream>
#include <limits>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
using Pose = TeachingTrajectory::Pose;
Pose join(const ReplayArm::Pose& left, const ReplayArm::Pose& right) {
    Pose result{};
    std::copy(left.begin(), left.end(), result.begin());
    std::copy(right.begin(), right.end(), result.begin()+8);
    return result;
}
ReplayArm::Pose half(const Pose& pose, size_t offset) {
    ReplayArm::Pose result{};
    std::copy_n(pose.begin()+offset, 8, result.begin());
    return result;
}
double arm_error(const Pose& a, const Pose& b) {
    double error = 0;
    for (size_t i = 0; i < 16; ++i) if (i%8 != 7) error = std::max(error, std::abs(a[i]-b[i]));
    return error;
}

// Only change the next run while holding an endpoint, never mid-motion.
void update_repeat(const std::string& command, bool holding, int& repeat) {
    if (command.rfind("repeat ", 0) != 0) return;
    try {
        const auto value = command.substr(7);
        size_t end = 0;
        const int requested = std::stoi(value, &end);
        if (!holding || end != value.size() || requested < 0 || requested > 10000)
            throw std::invalid_argument("invalid repeat setting or phase");
        repeat = requested;
        std::cout << "[replay] REPEAT_SET loops=" << repeat << std::endl;
    } catch (const std::exception&) {
        std::cout << "[replay] REPEAT_REJECTED: hold first or final frame; expected 0..10000" << std::endl;
    }
}

void finish_loop(const std::string& command, bool playing, int cycle, int& repeat) {
    if (command != "finish_loop") return;
    if (playing) {
        repeat = cycle;
        std::cout << "[replay] LOOP_END_REQUESTED loop=" << cycle
                  << ": finish current cycle and hold final pose" << std::endl;
    } else {
        std::cout << "[replay] LOOP_END_REJECTED: playback has not started" << std::endl;
    }
}

// Data preview restores recorded joint states. It never constructs a CAN arm or
// passes through the real-arm servo's velocity limits and catch-up pauses.
void preview(const TeachingTrajectory& trajectory, const std::string& xml,
             double speed, int repeat, bool wait_for_start, const std::string& startup_file, double transition_sec, double hold_sec) {
    if (xml.empty()) throw std::runtime_error("simulation requires --xml");
    auto world = std::make_shared<MujocoSimWorld>(xml, "OpenArm trajectory preview");
    MujocoTeleopHardware left(world, "left"), right(world, "right");
    Pose initial{};
    initial[3] = initial[11] = 1.570796;
    left.set_home(std::vector<double>(initial.begin(), initial.begin()+7), initial[7]);
    right.set_home(std::vector<double>(initial.begin()+8, initial.begin()+15), initial[15]);
    const auto first = trajectory.sample(0);
    ReplayPreparation preparation(startup_file, transition_sec, hold_sec);
    preparation.reset(initial, first.position, true);
    double alignment_duration = preparation.duration();
    using Clock = std::chrono::steady_clock;
    auto anchor = Clock::now(), next_log = anchor;
    double offset = 0;
    int cycle = 1;
    bool paused = false, complete = false, aligning = alignment_duration > 1e-6;
    bool wait_at_start = wait_for_start, ready = wait_at_start && !aligning;
    std::string input;
    if (aligning) {
        std::cout << "[replay] ALIGNING mode=preview transition_seconds=" << alignment_duration << std::endl;
    } else if (ready) {
        std::cout << "[replay] READY: holding first frame; waiting for play" << std::endl;
    } else {
        std::cout << "[replay] PLAYING mode=preview speed=" << speed << std::endl;
    }
    while (running) {
        const auto now = Clock::now();
        bool finished_now = false;
        double position = offset + (paused || complete || ready ? 0 :
            std::chrono::duration<double>(now-anchor).count()*(aligning ? 1 : speed));
        pollfd commands{STDIN_FILENO, POLLIN, 0};
        if (poll(&commands, 1, 0) > 0 && (commands.revents & POLLIN)) {
            char buffer[256];
            const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (count > 0) input.append(buffer, static_cast<size_t>(count));
            for (size_t end; (end = input.find('\n')) != std::string::npos;) {
                const auto command = input.substr(0, end); input.erase(0, end+1);
                update_repeat(command, complete || ready, repeat);
                finish_loop(command, !complete && !ready && !wait_at_start, cycle, repeat);
                if (command == "pause" && !paused && !ready) { offset = position; paused = true; }
                if (command == "resume" && paused && !ready) { anchor = now; paused = false; }
                if (command == "play" && ready) {
                    ready = false; wait_at_start = false; paused = false;
                    position = offset = 0; anchor = now; next_log = now;
                    std::cout << "[replay] PLAYING mode=preview loop=1 speed=" << speed << std::endl;
                }
                if (command == "prepare" && complete) {
                    initial = trajectory.sample(trajectory.duration()).position;
                    preparation.reset(initial, first.position, true, false);
                    alignment_duration = preparation.duration();
                    position = offset = 0; anchor = now; next_log = now;
                    cycle = 1; paused = false; complete = false;
                    wait_at_start = true; aligning = alignment_duration > 1e-6; ready = !aligning;
                    if (aligning) {
                        std::cout << "[replay] ALIGNING mode=preview cycle=1 transition_seconds="
                                  << alignment_duration << std::endl;
                    } else {
                        std::cout << "[replay] READY: holding first frame; waiting for play" << std::endl;
                    }
                }
                if (command == "restart" && complete && !wait_for_start) {
                    position = offset = 0;
                    anchor = now; next_log = now;
                    cycle = 1; paused = false; complete = false;
                    std::cout << "[replay] PLAYING mode=preview loop=1 restart=1 speed=" << speed << std::endl;
                }
                if (command.rfind("seek ", 0) == 0) {
                    if (aligning || ready) {
                        std::cout << "[replay] SEEK_REJECTED: " << (aligning ? "aligning to first frame" : "waiting for play") << std::endl;
                        continue;
                    }
                    try {
                        const auto value = command.substr(5);
                        size_t end = 0;
                        const double requested = std::stod(value, &end);
                        if (end != value.size() || !std::isfinite(requested)) {
                            throw std::invalid_argument("invalid seek time");
                        }
                        position = offset = std::clamp(requested, 0.0, trajectory.duration());
                        anchor = now;
                        // After completion, scrubbing opens a paused preview.
                        if (complete && position < trajectory.duration()) {
                            complete = false; paused = true;
                        }
                        next_log = now;
                        std::cout << "[replay] SEEKED time=" << position << std::endl;
                    } catch (const std::exception&) {
                        std::cout << "[replay] SEEK_REJECTED: expected finite seconds" << std::endl;
                    }
                }
                if (command == "stop") running = 0;
            }
        }
        if (!running) break;
        if (aligning && !paused && position >= alignment_duration) {
            aligning = false;
            position = offset = 0; anchor = now; next_log = now;
            ready = wait_at_start;
            if (ready) std::cout << "[replay] READY: holding first frame; waiting for play" << std::endl;
            else std::cout << "[replay] PLAYING mode=preview speed=" << speed << std::endl;
        }
        const double duration = trajectory.duration();
        if (!aligning && !complete && !ready && position >= duration) {
            // Preserve overshoot across loops, including when rendering stalls.
            const double crossed = std::floor(position/duration);
            if (repeat != 0 && crossed >= repeat-cycle+1) {
                cycle = repeat; position = offset = duration; complete = true;
                finished_now = true;
                std::cout << "[replay] COMPLETE: holding final pose" << std::endl;
            } else {
                if (crossed > std::numeric_limits<int>::max()-cycle) {
                    throw std::runtime_error("preview loop count overflow");
                }
                cycle += static_cast<int>(crossed);
                position = offset = std::fmod(position, duration); anchor = now;
                std::cout << "[replay] PLAYING loop=" << cycle << std::endl;
            }
        }
        auto goal = trajectory.sample(aligning ? 0 : position);
        if (aligning) goal = preparation.sample(position);
        auto restore = [&](MujocoTeleopHardware& arm, size_t start) {
            std::vector<openarm::damiao_motor::MITParam> commands;
            for (size_t i = start; i < start+7; ++i) {
                commands.push_back({1, 0, goal.position[i],
                    paused || complete || ready ? 0 : goal.velocity[i]*(aligning ? 1 : speed), 0});
            }
            arm.send_arm(commands);
            arm.send_gripper({{1, 0, goal.position[start+7], 0, 0}});
        };
        restore(left, 0); restore(right, 8);
        if (now >= next_log || finished_now) {
            std::cout << "[replay] PROGRESS time=" << (aligning ? 0 : position) << " duration=" << duration
                      << " phase=" << (complete ? "complete" : ready ? "ready" : paused ? "paused" : aligning ? (startup_file.empty() ? "aligning" : preparation.phase(position)) : "playing")
                      << " loop=" << cycle << " loops=" << (repeat == 0 ? "inf" : std::to_string(repeat))
                      << " preparation_time=" << (aligning ? position : alignment_duration)
                      << " startup_duration=" << preparation.startup_duration()
                      << " preparation_duration=" << preparation.duration()
                      << " q0=" << goal.position[0]
                      << " mode=preview" << std::endl;
            next_log = now+std::chrono::milliseconds(500);
        }
        world->forward_and_render();
        if (world->window_should_close()) break;
        // A late render skips display samples; it does not accumulate servo debt.
        std::this_thread::sleep_until(now+std::chrono::milliseconds(5));
    }
    std::cout << "[replay] STOPPED: preview closed" << std::endl;
}
}

int main(int argc, char** argv) {
    try {
        bool simulation = true, source_meters = false, check_only = false, wait_for_start = false;
        double speed = 1.0;
        double transition_sec = openarm::scurve::env_double("OPENARM_QUEST_STARTUP_TRANSITION_SEC", 5, 0, 10);
        double hold_sec = openarm::scurve::env_double("OPENARM_QUEST_STARTUP_HOLD_SEC", .6, 0, 5);
        int repeat = 1, cycle = 1;
        std::string startup_file;
        std::string csv, urdf = "assets/v1.urdf", xml;
        const char* gravity_env = std::getenv("OPENARM_GRAVITY_YAML");
        std::string yaml = gravity_env && *gravity_env ? gravity_env : "config/gravity_comp.yaml";
        std::string left_can = "can1", right_can = "can0";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&]() { if (++i >= argc) throw std::runtime_error("missing " + arg + " value"); return std::string(argv[i]); };
            if (arg == "--csv") csv = value();
            else if (arg == "--startup-file") startup_file = value();
            else if (arg == "--startup-transition-sec") transition_sec = std::stod(value());
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
            else if (arg == "--wait-for-start") wait_for_start = true;
            else throw std::runtime_error("unknown replay option: " + arg);
        }
        if (csv.empty() || !std::isfinite(speed) || speed < .1 || speed > 1) {
            throw std::runtime_error("--csv is required; speed must be 0.1..1.0");
        }
        if (!simulation && (left_can.empty() || right_can.empty() || left_can == right_can)) {
            throw std::runtime_error("two distinct CAN interfaces are required");
        }
        if (!simulation) {
            std::cout << "[replay] SAFETY_STOP_LIMITS rad_per_sec J1-J6="
                      << kReplayArmFeedbackSpeedStop << " J7=" << kReplayWristFeedbackSpeedStop
                      << " gripper=" << kReplayGripperFeedbackSpeedStop
                      << " (absolute measured speed; exceeding limit stops both arms)" << std::endl;
        }
        if (!std::isfinite(transition_sec) || transition_sec < 0 || transition_sec > 10) {
            throw std::runtime_error("startup transition seconds must be 0..10");
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
                  << " speed=" << speed << (simulation ? " (recorded pose preview)" :
                      " (recorded torques ignored; gravity recomputed)") << std::endl;
        TeachingTrajectory startup;
        if (!startup_file.empty()) {
            startup.load(startup_file);
            // The Quest startup CSV uses motor radians, independently of the
            // selected recording's gripper units.
            startup.convert_grippers(false, simulation);
            startup.validate_limits(lower, upper);
            std::cout << "[replay] STARTUP_CHECKED file=" << startup_file
                      << " frames=" << startup.size() << " duration=" << startup.duration() << std::endl;
        }
        if (check_only) return 0;
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        if (simulation) {
            preview(trajectory, xml, speed, repeat, wait_for_start, startup_file, transition_sec, hold_sec);
            return 0;
        }
        GravityTune lt, rt;
        GravityModel model;
        if (!load_gravity_yaml(yaml, &lt, &rt, &model)) throw std::runtime_error("cannot load gravity configuration");
        setenv("OPENARM_GRAVITY_YAML", yaml.c_str(), 1);
        setenv("OPENARM_DIAGNOSTIC_URDF", urdf.c_str(), 1);
        ReplayPreparationJob preparation_job;
        ReplayArm left("left", left_can, urdf, lt, model);
        if (!running) return 0;
        ReplayArm right("right", right_can, urdf, rt, model);
        TeachingReplayReference reference(join(left.pose(), right.pose()), lower, upper);
        ReplayPreparation preparation(startup_file, transition_sec, hold_sec);
        preparation.reset(reference.position(), trajectory.sample(0).position, false);
        double preparation_time = 0;
        bool preparing = !startup_file.empty();
        bool aligning = true, paused = false, complete = false;
        bool wait_at_start = wait_for_start, ready = false;
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
                    update_repeat(command, complete || ready, repeat);
                    finish_loop(command, !complete && !ready && !wait_at_start, cycle, repeat);
                    if (command == "pause" && !ready) paused = true;
                    if (command == "resume" && !ready) paused = false;
                    if (command == "play" && ready) {
                        ready = false; wait_at_start = false; paused = false;
                        phase_time = 0; next_log = now;
                        std::cout << "[replay] PLAYING" << std::endl;
                    }
                    if ((command == "prepare" || (command == "restart" && !wait_for_start)) && complete) {
                        position = 0; phase_time = 0; settled = 0;
                        cycle = 1; paused = false; complete = false; aligning = true;
                        ready = false; wait_at_start = command == "prepare";
                        const Pose held = reference.position();
                        const Pose target = trajectory.sample(0).position;
                        preparation_job.start([startup_file, transition_sec, hold_sec, held, target]() {
                            ReplayPreparation result(startup_file, transition_sec, hold_sec);
                            result.reset(held, target, false, false);
                            return result;
                        });
                        preparation_time = 0;
                        preparing = false;
                        next_log = now;
                        std::cout << "[replay] ALIGNING: cycle=1 restart=1 preparing return; holding final pose" << std::endl;
                    }
                    if (command == "stop") running = 0;
                }
            }
            if (!running) break;
            if (preparation_job.take(&preparation)) {
                preparing = true;
                preparation_time = phase_time = 0;
                next_log = now;
                std::cout << "[replay] ALIGNING: return trajectory ready; moving to first frame" << std::endl;
            }
            left.read(); right.read();
            auto goal = preparing ? preparation.sample(preparation_time) : trajectory.sample(position);
            if (preparation_job.busy()) {
                goal.position = reference.position();
                goal.velocity.fill(0);
            }
            const bool timed_startup = preparing && preparation.startup_active(preparation_time);
            const bool catching_up = !timed_startup && arm_error(goal.position, reference.position()) > .03;
            for (auto& v : goal.velocity) {
                v *= (!paused && !catching_up && preparing) ? 1 :
                    ((!aligning && !paused && !complete && !ready && !catching_up) ? speed : 0);
            }
            const auto measured = join(left.pose(), right.pose());
            if (paused) {
                goal.position = reference.position();
                goal.velocity.fill(0);
            }
            if (timed_startup && !paused) {
                reference.follow_startup(goal, dt);
            } else {
                reference.step(goal, measured, join(left.velocity(), right.velocity()), dt, aligning && !preparing);
            }
            const int diagnostic_phase = (aligning || preparing) ? 1 : (paused || ready || complete) ? 3 : 2;
            left.set_diagnostic_context(half(goal.position, 0), diagnostic_phase, position);
            right.set_diagnostic_context(half(goal.position, 8), diagnostic_phase, position);
            left.send(half(reference.position(), 0), half(reference.velocity(), 0));
            right.send(half(reference.position(), 8), half(reference.velocity(), 8));
            if (!paused) phase_time += dt;
            if (aligning && !paused && !preparation_job.busy()) {
                if (phase_time > 120 + (preparing ? preparation.duration()*10 : 0)) {
                    throw std::runtime_error("alignment timed out");
                }
                // The shared startup uses wall time, as in Quest. Only the
                // separate connection to the recording waits for catch-up.
                if (preparing && (timed_startup || (!catching_up && arm_error(goal.position, measured) < .12))) {
                    preparation_time = std::min(preparation.duration(), preparation_time+dt);
                }
                if ((!preparing || preparation_time >= preparation.duration()) &&
                    arm_error(trajectory.sample(0).position, reference.position()) < .015 &&
                    arm_error(trajectory.sample(0).position, measured) < .12) {
                    preparing = false;
                    aligning = false; phase_time = 0;
                    ready = wait_at_start;
                    if (ready) std::cout << "[replay] READY: holding first frame; waiting for play" << std::endl;
                    else std::cout << "[replay] PLAYING" << std::endl;
                }
            } else if (!aligning && !paused && !complete && !ready) {
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
                          << " phase=" << (complete ? "complete" : ready ? "ready" : paused ? "paused" : preparation_job.busy() ? "preparing" : aligning ? (preparing ? preparation.phase(preparation_time) : "aligning") : "playing")
                          << " loop=" << cycle << " loops=" << (repeat == 0 ? "inf" : std::to_string(repeat))
                          << " preparation_time=" << preparation_time
                          << " startup_duration=" << preparation.startup_duration()
                          << " preparation_duration=" << preparation.duration()
                          << " error=" << arm_error(goal.position, measured) << std::endl;
                next_log = now+std::chrono::milliseconds(500);
            }
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
