#pragma once

#include <robot_state.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

// Optional loopback telemetry. The producer never waits for the consumer.
class JointAngleStream {
public:
    static std::unique_ptr<JointAngleStream> create(const std::string& side) {
        const char* raw = std::getenv("OPENARM_JOINT_ANGLE_PORT");
        if (!raw || (side != "left" && side != "right" &&
                     side != "left_arm" && side != "right_arm")) return nullptr;
        char* end = nullptr;
        long port = std::strtol(raw, &end, 10);
        if (*end || port < 1 || port > 65535) return nullptr;
        return std::unique_ptr<JointAngleStream>(new JointAngleStream(
            side.find("left") == 0 ? "left" : "right", static_cast<int>(port)));
    }

    ~JointAngleStream() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        if (socket_ >= 0) close(socket_);
    }

    void update(const std::vector<JointState>& joints) {
        if (joints.size() < 7) return;
        std::array<double, 7> angles;
        for (size_t i = 0; i < 7; ++i) angles[i] = joints[i].position;
        update(angles);
    }

    void update(const std::array<double, 7>& angles) {
        for (double angle : angles) if (!std::isfinite(angle)) return;
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        angles_ = angles;
        sampled_ = std::chrono::steady_clock::now();
        ++sequence_;
    }

private:
    JointAngleStream(std::string side, int port) : side_(std::move(side)) {
        socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(port);
        destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (socket_ >= 0) worker_ = std::thread([this] { run(); });
    }

    void run() {
        uint64_t sent = 0;
        while (running_) {
            std::array<double, 7> angles;
            uint64_t sequence;
            double age;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                angles = angles_;
                sequence = sequence_;
                age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - sampled_).count();
            }
            if (sequence != sent && age < 0.5) {
                std::ostringstream packet;
                packet.precision(12);
                packet << "{\"side\":\"" << side_ << "\",\"q\":[";
                for (size_t i = 0; i < angles.size(); ++i) {
                    if (i) packet << ',';
                    packet << angles[i];
                }
                packet << "],\"age\":" << age << '}';
                const auto data = packet.str();
                sendto(socket_, data.data(), data.size(), MSG_DONTWAIT,
                       reinterpret_cast<sockaddr*>(&destination_), sizeof(destination_));
                sent = sequence;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    std::string side_;
    int socket_ = -1;
    sockaddr_in destination_{};
    std::atomic<bool> running_{true};
    std::thread worker_;
    std::mutex mutex_;
    std::array<double, 7> angles_{};
    uint64_t sequence_ = 0;
    std::chrono::steady_clock::time_point sampled_{};
};
