// Tests the real mapping and bounded IK; no CAN, sockets or simulation window.
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "controller/quest_types.hpp"
#include "controller/dynamics.hpp"
#include "controller/human_mimic_ik_solver.hpp"
#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"
#include "controller/joint_reference_generator.hpp"
#include "controller/reference_smoother.hpp"
#include "controller/unitree_style_profile.hpp"
#include "controller/virtual_arm_skeleton.hpp"
#include "robot_state.hpp"
#define private public
#include "controller/human_mimic_controller.hpp"
#undef private

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "expected URDF path");
        require(!HumanMimicConfig{}.consistent_world_mapping, "default profile changed");
        for (const std::string side : {"left", "right"}) {
            Dynamics dynamics(argv[1], "openarm_body_link0", "openarm_" + side + "_link7");
            require(dynamics.Init(), "dynamics initialization");
            HumanMimicConfig cfg;
            cfg.urdf_path = argv[1];
            cfg.shoulder_link = "openarm_" + side + "_link1";
            cfg.elbow_link = "openarm_" + side + "_link4";
            cfg.wrist_link = "openarm_" + side + "_link7";
            cfg.debug_log = false;
            cfg.tau_orientation = .0001;
            cfg.orientation_deadband_deg = 0;
            cfg.q_nominal = {0, 0, 0, M_PI / 2, 0, 0, 0};
            apply_unitree_style_profile(&cfg);
            cfg.reference_yaw_rad = .35;
            HumanMimicController controller(&dynamics, cfg, "unitree-test");
            require(controller.Init(), "controller initialization");
            auto q = cfg.q_nominal;
            QuestPoseSample sample;
            sample.valid = true;
            sample.grip_active = true;
            sample.orientation = Eigen::AngleAxisd(.8, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitX());
            controller.activate_clutch(q, sample);
            const Eigen::Matrix3d anchor = controller.robot_ref_rot_abs_.toRotationMatrix();
            const Eigen::Matrix3d M = controller.vr_to_robot_axes();
            require(std::abs(M.determinant() - 1) < 1e-12, "mapping reflects an axis");
            require((M * Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitZ()).norm() < 1e-12,
                    "world up is not robot up");
            for (int axis = 0; axis < 3; ++axis) {
                require((controller.map_vr_delta_pos(Eigen::Vector3d::Unit(axis)) - M.col(axis)).norm() < 1e-12,
                        "translation mapping inconsistent");
                // Independent of how the controller was tilted when gripping.
                auto rotated = sample;
                const auto delta = Eigen::AngleAxisd(.15, Eigen::Vector3d::Unit(axis));
                rotated.orientation = delta * sample.orientation;
                controller.ori_filt_ = rotated.orientation;
                const Eigen::Matrix3d mapped = controller.compute_hand_target(rotated, .02);
                require((mapped - M * delta.toRotationMatrix() * M.transpose() * anchor).norm() < 1e-6,
                        "rotation and translation use different references");
            }
            std::vector<double> lower, upper;
            require(dynamics.GetJointPositionLimits(&lower, &upper), "joint limits unavailable");
            HumanMimicKinematics kin(cfg.urdf_path, cfg.root_link, cfg.shoulder_link,
                                     cfg.elbow_link, cfg.wrist_link);
            require(kin.Init(), "kinematics initialization");
            HumanMimicIkSolver solver(&kin, cfg);
            solver.SetJointLimits(lower, upper);
            TcpState tcp;
            require(kin.ComputeTcpState(q, cfg.wrist_to_tcp, &tcp), "FK failed");
            // Both a small Cartesian path and an unreachable target must remain bounded.
            for (int step = 0; step < 100; ++step) {
                Eigen::Vector3d goal = tcp.p_tcp;
                goal.x() += step < 90 ? .03 * std::sin(step * .1) : 2.0;
                auto solved = solver.Solve(q, goal, tcp.R_hand, Eigen::Vector3d::Zero(), cfg.q_nominal, 0);
                require(solved.ok, "IK failed");
                for (size_t i = 0; i < q.size(); ++i) {
                    require(std::isfinite(solved.q[i]) && solved.q[i] >= lower[i] - 1e-9 &&
                            solved.q[i] <= upper[i] + 1e-9, "unbounded IK solution");
                }
                q = solved.q;
            }
        }
        std::cout << "unitree_style_test: ok\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
