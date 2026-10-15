#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unistd.h>

// One producer (servo), one consumer (disk). Fixed storage, try-lock producer;
// diagnostic loss is preferable to delaying a motor command. Never drives hardware.
class ControlDiagnostics {
public:
    struct Joint {
        double source_q = 0, cmd_q = 0, cmd_dq = 0, q = 0, dq = 0, torque = 0;
        double kp = 0, kd = 0, gravity_raw = 0, gravity_requested = 0, ff = 0;
        double ff_limit = 0, protocol_limit = 0, age_ms = 0;
        int status = 0, mos = 0, rotor = 0;
    };
    struct Frame {
        double mono = 0, unix_time = 0, dt = 0, trajectory_time = -1;
        int phase = 0;
        std::array<Joint, 7> joints{};
    };
    static std::unique_ptr<ControlDiagnostics> create(const std::string& side) {
        const char* enabled = std::getenv("OPENARM_CONTROL_DIAGNOSTICS");
        if (enabled && std::string(enabled) == "0") return nullptr;
        try { return std::unique_ptr<ControlDiagnostics>(new ControlDiagnostics(side)); }
        catch (const std::exception& e) {
            std::cerr << "[diagnostics] disabled: " << e.what() << std::endl;
            return nullptr;
        }
    }
    ~ControlDiagnostics() {
        stopping_.store(true);
        if (worker_.joinable()) worker_.join();
        std::ofstream stats(path_.string()+".stats");
        stats << "dropped_frames=" << dropped_.load() << "\nwrite_failed=" << !out_.good() << '\n';
    }
    // Call once every servo step, including those not sampled. Captures true cycle dt.
    bool sample_due(Frame* frame) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = previous_.time_since_epoch().count() ?
            std::chrono::duration<double>(now-previous_).count() : 0;
        previous_ = now;
        if (now < next_sample_) return false;
        next_sample_ = now + std::chrono::milliseconds(10);
        frame->mono = std::chrono::duration<double>(now.time_since_epoch()).count();
        frame->unix_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        frame->dt = dt;
        return true;
    }
    void submit(const Frame& frame) noexcept {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || count_ == queue_.size()) { ++dropped_; return; }
        queue_[(head_+count_) % queue_.size()] = frame;
        ++count_;
    }
private:
    explicit ControlDiagnostics(const std::string& side) {
        const char* override_dir = std::getenv("OPENARM_CONTROL_DIAGNOSTICS_DIR");
        const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
        std::filesystem::path dir = override_dir && *override_dir ? override_dir :
            "recordings/control_diagnostics/run_"+std::to_string(getpid());
        std::filesystem::create_directories(dir);
        path_ = dir / (side+"_"+std::to_string(stamp)+".csv");
        // Snapshot before the servo thread starts; gains actually used are in CSV.
        for (const std::string file : {"config/follower.yaml", "config/gravity_comp.yaml", "assets/v1.urdf"}) {
            const char* gravity = std::getenv("OPENARM_GRAVITY_YAML");
            const char* urdf = std::getenv("OPENARM_DIAGNOSTIC_URDF");
            const std::filesystem::path source = file=="config/gravity_comp.yaml" && gravity && *gravity ? gravity :
                file=="assets/v1.urdf" && urdf && *urdf ? urdf : file;
            std::error_code error;
            std::filesystem::copy_file(source, path_.string()+"."+source.filename().string(), error);
        }
        std::ofstream settings(path_.string()+".settings");
        for (const char* key : {"OPENARM_QUEST_MIT_SCALE", "OPENARM_QUEST_GRAVITY_FF", "OPENARM_QUEST_GRAVITY_GAIN",
                               "OPENARM_QUEST_GRAVITY_RAMP_SEC", "OPENARM_GRAVITY_YAML", "OPENARM_DIAGNOSTIC_URDF"}) {
            const char* value=std::getenv(key);
            settings << key << '=' << (value ? value : "<default>") << '\n';
        }
        out_.open(path_);
        if (!out_) throw std::runtime_error("cannot open " + path_.string());
        out_.precision(12);
        out_ << "mono,unix,dt,trajectory_time,phase,joint,source_q,cmd_q,cmd_dq,q,dq,torque,kp,kd,gravity_raw,gravity_requested,ff,ff_limit,protocol_limit,age_ms,status,mos,rotor,pd,total_estimate\n";
        out_.flush();
        std::cout << "[diagnostics] " << path_ << " (100 Hz, asynchronous)" << std::endl;
        worker_ = std::thread([this] { drain(); });
    }
    void drain() {
        auto next_flush = std::chrono::steady_clock::now();
        for (;;) {
            Frame f;
            bool have = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (count_) { f=queue_[head_]; head_=(head_+1)%queue_.size(); --count_; have=true; }
            }
            if (have) {
                for (size_t i=0; i<7; ++i) {
                    const auto& j=f.joints[i];
                    const double pd=j.kp*(j.cmd_q-j.q)+j.kd*(j.cmd_dq-j.dq);
                    out_ << f.mono << ',' << f.unix_time << ',' << f.dt << ',' << f.trajectory_time << ',' << f.phase << ',' << i+1
                         << ',' << j.source_q << ',' << j.cmd_q << ',' << j.cmd_dq << ',' << j.q << ',' << j.dq << ',' << j.torque
                         << ',' << j.kp << ',' << j.kd << ',' << j.gravity_raw << ',' << j.gravity_requested << ',' << j.ff
                         << ',' << j.ff_limit << ',' << j.protocol_limit << ',' << j.age_ms << ',' << j.status << ',' << j.mos
                         << ',' << j.rotor << ',' << pd << ',' << pd+j.ff << '\n';
                }
            } else {
                if (stopping_.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            const auto now=std::chrono::steady_clock::now();
            if (now >= next_flush) { out_.flush(); next_flush=now+std::chrono::seconds(1); }
        }
        out_.flush();
    }
    std::filesystem::path path_;
    std::ofstream out_;
    std::mutex mutex_;
    std::array<Frame, 256> queue_{};
    size_t head_=0, count_=0;
    std::atomic<size_t> dropped_{0};
    std::atomic<bool> stopping_{false};
    std::thread worker_;
    std::chrono::steady_clock::time_point previous_{}, next_sample_{};
};
