#pragma once

// Talks to script/unitree_openarm/ik_server.py. The 1000 Hz teleop thread only
// posts a target and reads the newest solution. IPOPT runs on this object's thread.

#include <Eigen/Dense>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class UnitreeIpoptClient {
public:
    UnitreeIpoptClient() = default;
    UnitreeIpoptClient(const UnitreeIpoptClient&) = delete;
    UnitreeIpoptClient& operator=(const UnitreeIpoptClient&) = delete;
    ~UnitreeIpoptClient() { stop(); }

    // True when this interpreter can import casadi and pinocchio.casadi.
    static bool imports_solver(const std::string& python);
    bool start(const std::string& python, const std::string& script, const std::string& urdf,
               const std::string& side);
    void stop();
    bool active() const { return ready_.load(); }

    void submit(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                const std::vector<double>& seed);
    bool latest(std::vector<double>* joints, double max_age_s = 0.30) const;

private:
    void loop();
    bool write_request(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                       const std::vector<double>& seed);
    bool read_line(std::string* line, int timeout_ms);

    struct Request {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        std::vector<double> seed;
        uint64_t sequence = 0;
    };

    int write_fd_ = -1;
    int read_fd_ = -1;
    int pid_ = -1;
    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    Request pending_;
    std::string pending_input_;
    std::vector<double> solution_;
    std::chrono::steady_clock::time_point solution_time_{};
    bool have_solution_ = false;
};
