#include <io/teleop_trajectory_logger.hpp>

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>

TeleopTrajectoryLogger& TeleopTrajectoryLogger::shared() {
    static TeleopTrajectoryLogger instance;
    return instance;
}

void TeleopTrajectoryLogger::configure_from_env() {
    const char* path = std::getenv("OPENARM_TELEOP_TRAJ_LOG");
    if (path == nullptr || path[0] == '\0') {
        enabled_ = false;
        return;
    }

    log_dir_ = path;
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    if (ec) {
        std::cerr << "[traj-log] failed to create log dir: " << log_dir_ << std::endl;
        enabled_ = false;
        return;
    }

    enabled_ = true;
    t0_initialized_ = false;
    std::cout << "[traj-log] recording TCP/joint trajectories to: " << log_dir_ << std::endl;
}

const char* TeleopTrajectoryLogger::state_name(TeleopControlState state) {
    switch (state) {
        case TeleopControlState::Active:
            return "active";
        case TeleopControlState::Degraded:
            return "degraded";
        case TeleopControlState::Hold:
            return "hold";
        case TeleopControlState::Estop:
            return "estop";
        case TeleopControlState::Idle:
        default:
            return "idle";
    }
}

std::ofstream& TeleopTrajectoryLogger::stream_for(const std::string& arm_label, size_t dof) {
    auto& file = files_[arm_label];
    if (!file.is_open()) {
        const std::filesystem::path csv_path =
            std::filesystem::path(log_dir_) / (arm_label + "_tcp.csv");
        file.open(csv_path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "[traj-log] failed to open " << csv_path << std::endl;
            enabled_ = false;
            return file;
        }
        file << "t_s,state,grip,tcp_x,tcp_y,tcp_z,tgt_x,tgt_y,tgt_z";
        for (size_t i = 0; i < dof; ++i) {
            file << ",q" << i << ",ref" << i << ",dq" << i;
        }
        for (size_t i = 0; i < dof; ++i) {
            file << ",target" << i;
        }
        file << ",requested_x,requested_y,requested_z,ik_ms,steady_s,orientation_error_rad\n";
        header_written_[arm_label] = true;
        std::cout << "[traj-log] writing " << csv_path << std::endl;
    }
    return file;
}

void TeleopTrajectoryLogger::log(const std::string& arm_label, Dynamics* dynamics,
                                 TeleopControlState state, bool grip,
                                 const Eigen::Vector3d& tcp_target,
                                 const std::vector<JointState>& measured,
                                 const std::vector<JointState>& reference,
                                 const std::vector<double>* joint_target,
                                 const Eigen::Vector3d* requested_tcp, double ik_ms,
                                 const Eigen::Matrix3d* requested_rotation) {
    if (!enabled_ || dynamics == nullptr || measured.empty()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!t0_initialized_) {
        t0_ = now;
        t0_initialized_ = true;
    }
    const double t_s =
        std::chrono::duration<double>(now - t0_).count();

    const size_t dof = measured.size();
    auto& file = stream_for(arm_label, dof);
    if (!file.is_open()) {
        return;
    }

    std::vector<double> q_meas(dof, 0.0);
    for (size_t i = 0; i < dof; ++i) {
        q_meas[i] = measured[i].position;
    }

    Eigen::Matrix3d r_tcp = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    dynamics->GetEECordinate(q_meas.data(), r_tcp, p_tcp);

    file << std::fixed << std::setprecision(6) << t_s << ',' << state_name(state) << ','
         << (grip ? 1 : 0) << ',' << p_tcp.x() << ',' << p_tcp.y() << ',' << p_tcp.z() << ','
         << tcp_target.x() << ',' << tcp_target.y() << ',' << tcp_target.z();

    for (size_t i = 0; i < dof; ++i) {
        const double q = measured[i].position;
        const double ref = i < reference.size() ? reference[i].position : q;
        const double dq = measured[i].velocity;
        file << ',' << q << ',' << ref << ',' << dq;
    }
    for (size_t i = 0; i < dof; ++i) {
        const double target =
            joint_target != nullptr && i < joint_target->size() ? (*joint_target)[i]
                                                                : std::numeric_limits<double>::quiet_NaN();
        file << ',' << target;
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d requested = requested_tcp != nullptr
                                         ? *requested_tcp : Eigen::Vector3d::Constant(nan);
    file << ',' << requested.x() << ',' << requested.y() << ',' << requested.z()
         << ',' << ik_ms << ',' << std::setprecision(9)
         << std::chrono::duration<double>(now.time_since_epoch()).count();
    const double orientation_error = requested_rotation != nullptr
        ? Eigen::AngleAxisd(Eigen::Quaterniond(*requested_rotation * r_tcp.transpose()).normalized()).angle()
        : nan;
    file << ',' << orientation_error << '\n';
}

void TeleopTrajectoryLogger::close() {
    for (auto& [label, file] : files_) {
        if (file.is_open()) {
            file.flush();
            file.close();
        }
        (void)label;
    }
    files_.clear();
    header_written_.clear();
    enabled_ = false;
    t0_initialized_ = false;
}
