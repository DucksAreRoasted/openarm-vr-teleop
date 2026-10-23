#include "controller/unitree_ipopt_client.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

bool UnitreeIpoptClient::imports_solver(const std::string& python) {
    std::string py = python;
    std::string flag = "-c";
    std::string statement = "import casadi; from pinocchio import casadi";
    char* argv[] = {py.data(), flag.data(), statement.data(), nullptr};
    const int devnull = open("/dev/null", O_WRONLY);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    if (devnull >= 0) {
        posix_spawn_file_actions_adddup2(&actions, devnull, STDOUT_FILENO);
        posix_spawn_file_actions_adddup2(&actions, devnull, STDERR_FILENO);
        posix_spawn_file_actions_addclose(&actions, devnull);
    }
    pid_t child = 0;
    const int spawned = posix_spawnp(&child, py.c_str(), &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    if (devnull >= 0) {
        close(devnull);
    }
    if (spawned != 0) {
        return false;
    }
    int status = 0;
    if (waitpid(child, &status, 0) < 0) {
        return false;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

bool UnitreeIpoptClient::start(const std::string& python, const std::string& script,
                               const std::string& urdf, const std::string& side) {
    stop();
    int to_child[2] = {-1, -1};
    int from_child[2] = {-1, -1};
    if (pipe(to_child) != 0 || pipe(from_child) != 0) {
        return false;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, to_child[0], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions, from_child[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, to_child[0]);
    posix_spawn_file_actions_addclose(&actions, to_child[1]);
    posix_spawn_file_actions_addclose(&actions, from_child[0]);
    posix_spawn_file_actions_addclose(&actions, from_child[1]);

    std::string python_copy = python;
    std::string script_copy = script;
    std::string side_copy = side;
    std::string urdf_copy = urdf;
    char* argv[] = {python_copy.data(), script_copy.data(), side_copy.data(), urdf_copy.data(),
                    nullptr};
    pid_t child = 0;
    const int spawned = posix_spawn(&child, python_copy.c_str(), &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    close(to_child[0]);
    close(from_child[1]);
    if (spawned != 0) {
        close(to_child[1]);
        close(from_child[0]);
        std::cerr << "[unitree-ik] 启动失败：" << std::strerror(spawned) << std::endl;
        return false;
    }
    write_fd_ = to_child[1];
    read_fd_ = from_child[0];
    pid_ = static_cast<int>(child);

    std::string ready;
    if (!read_line(&ready, 60000) || ready != "ready") {
        std::cerr << "[unitree-ik] " << side << " 求解器没有就绪"
                  << (ready.empty() ? "" : "：" + ready) << std::endl;
        stop();
        return false;
    }
    running_ = true;
    ready_ = true;
    worker_ = std::thread([this] { loop(); });
    std::cout << "[unitree-ik] " << side << " IPOPT 已接入 " << urdf << std::endl;
    return true;
}

void UnitreeIpoptClient::stop() {
    running_ = false;
    ready_ = false;
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    if (write_fd_ >= 0) {
        close(write_fd_);
        write_fd_ = -1;
    }
    if (read_fd_ >= 0) {
        close(read_fd_);
        read_fd_ = -1;
    }
    pending_input_.clear();
    if (pid_ > 0) {
        int status = 0;
        for (int i = 0; i < 20 && waitpid(pid_, &status, WNOHANG) == 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (waitpid(pid_, &status, WNOHANG) == 0) {
            kill(pid_, SIGKILL);
            waitpid(pid_, &status, 0);
        }
        pid_ = -1;
    }
}

void UnitreeIpoptClient::submit(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                                const std::vector<double>& seed) {
    if (!ready_.load() || seed.size() < 7) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.position = position;
        pending_.rotation = rotation;
        pending_.seed = seed;
        ++pending_.sequence;
    }
    cv_.notify_one();
}

bool UnitreeIpoptClient::latest(std::vector<double>* joints, double max_age_s) const {
    if (joints == nullptr || !ready_.load()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_solution_ || solution_.size() != 7) {
        return false;
    }
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - solution_time_)
                           .count();
    if (age > max_age_s) {
        return false;
    }
    *joints = solution_;
    return true;
}

void UnitreeIpoptClient::loop() {
    uint64_t seen = 0;
    while (running_.load()) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(50), [&] {
                return !running_.load() || pending_.sequence != seen;
            });
            if (!running_.load()) {
                break;
            }
            if (pending_.sequence == seen) {
                continue;
            }
            request = pending_;
        }
        if (!write_request(request.position, request.rotation, request.seed)) {
            break;
        }
        std::string line;
        if (!read_line(&line, 3000)) {
            break;
        }
        std::vector<double> solved;
        if (line != "fail") {
            std::istringstream input(line);
            double value = 0.0;
            while (input >> value) {
                solved.push_back(value);
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            seen = request.sequence;
            if (solved.size() == 7) {
                solution_ = std::move(solved);
                solution_time_ = std::chrono::steady_clock::now();
                have_solution_ = true;
            }
        }
    }
    ready_ = false;
}

bool UnitreeIpoptClient::write_request(const Eigen::Vector3d& position,
                                       const Eigen::Matrix3d& rotation,
                                       const std::vector<double>& seed) {
    if (write_fd_ < 0 || seed.size() < 7) {
        return false;
    }
    std::ostringstream line;
    line.setf(std::ios::fixed);
    line.precision(8);
    line << position.x() << ' ' << position.y() << ' ' << position.z();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            line << ' ' << rotation(row, col);
        }
    }
    for (int i = 0; i < 7; ++i) {
        line << ' ' << seed[static_cast<size_t>(i)];
    }
    line << '\n';
    const std::string text = line.str();
    size_t sent = 0;
    while (sent < text.size()) {
        const ssize_t wrote = ::write(write_fd_, text.data() + sent, text.size() - sent);
        if (wrote < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        sent += static_cast<size_t>(wrote);
    }
    return true;
}

bool UnitreeIpoptClient::read_line(std::string* line, int timeout_ms) {
    if (line == nullptr || read_fd_ < 0) {
        return false;
    }
    line->clear();
    if (!pending_input_.empty()) {
        line->swap(pending_input_);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    char buffer[256];
    while (line->find('\n') == std::string::npos) {
        const int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now())
                .count());
        if (remain <= 0) {
            return false;
        }
        pollfd poll_fd{};
        poll_fd.fd = read_fd_;
        poll_fd.events = POLLIN;
        const int polled = poll(&poll_fd, 1, remain);
        if (polled < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (polled == 0) {
            return false;
        }
        const ssize_t got = ::read(read_fd_, buffer, sizeof(buffer));
        if (got < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (got == 0) {
            return false;
        }
        line->append(buffer, buffer + got);
    }
    const auto end = line->find('\n');
    pending_input_ = line->substr(end + 1);
    line->resize(end);
    if (!line->empty() && line->back() == '\r') {
        line->pop_back();
    }
    return true;
}
