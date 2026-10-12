// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <vector>
/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface;

    std::string urdf_path;
    std::string start_link;
    std::string end_link;

    KDL::JntSpaceInertiaMatrix inertia_matrix;
    KDL::JntArray q;
    KDL::JntArray q_d;
    KDL::JntArray coriolis_forces;
    KDL::JntArray gravity_forces;

    KDL::JntArray biasangle;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainDynParam> solver;
    Eigen::Matrix<double, 6, 1> ik_weights_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> pos_ik_solver_;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    bool joint_limits_valid_ = false;

public:
    Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
    ~Dynamics();

    bool Init();
    void GetGravity(const double *motor_position, double *gravity);
    void GetCoriolis(const double *motor_position, const double *motor_velocity, double *coriolis);
    void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

    void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);

    void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

    void GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T);

    void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    // FK to a specific chain segment (0-based). Used to locate the robot "shoulder"/"elbow"
    // for human-skeleton redundancy resolution. Returns false if FK fails or index is invalid.
    bool GetSegmentCordinate(const double *motor_position, int segment_index, Eigen::Matrix3d &R,
                             Eigen::Vector3d &p);

    size_t GetNrOfSegments() const { return kdl_chain.getNrOfSegments(); }

    size_t GetDof() const { return kdl_chain.getNrOfJoints(); }

    double ComputeMinSingularValue(const double *motor_position);

    // Weighted DLS: q_dot = W J^T (J W J^T + λ² I)^{-1} v
    // joint_mobility_weights: per-joint mobility (higher => prefer that joint). Empty => identity.
    bool SolveDlsIk(const double *motor_position, const Eigen::VectorXd &v_cart, double lambda,
                    Eigen::VectorXd *dq_out, double *sigma_min_out = nullptr,
                    const std::vector<double> *joint_mobility_weights = nullptr);

    bool SolvePosIk(const double *motor_position, const Eigen::Matrix3d &R_des,
                    const Eigen::Vector3d &p_des, double *q_out);

    bool SolvePosIkPosition(const double *motor_position, const Eigen::Vector3d &p_des,
                            double *q_out);

    bool RefineIkOrientation(const double *motor_position, const Eigen::Quaterniond &q_des,
                             double dt, double *q_out);

    bool GetJointPositionLimits(std::vector<double> *q_min, std::vector<double> *q_max) const;
    void ClampJointPositions(double *q_inout) const;

    // Extra point mass in the end-link frame (e.g. fingers / measured gripper payload).
    void SetEndEffectorPointMass(double mass_kg, double ox = 0.0, double oy = 0.0,
                                 double oz = 0.0);
    // Wrist-top camera in the link7 frame. If ox/oy/oz are finite they override the
    // default lateral mount (left +Y / right −Y, small +Z).
    void SetWristCameraMass(double mass_kg, bool left_arm, double ox = std::numeric_limits<double>::quiet_NaN(),
                            double oy = std::numeric_limits<double>::quiet_NaN(),
                            double oz = std::numeric_limits<double>::quiet_NaN());

    // Per-joint gains for distal EE / camera point-mass gravity (past link7).
    void SetDistalMassGains(const std::array<double, 7>& ee_gain,
                            const std::array<double, 7>& cam_gain);

    // Multiply URDF-chain gravity (not the EE point mass). Use when CAD mass is systematically low.
    void SetUrdfGravityScale(double scale);

    // Gravity vector expressed in the chain root (start_link) frame.
    // OpenArmX starts at link0 and uses ±Y; we start at body_link0 with world -Z.
    void SetGravityVector(double gx, double gy, double gz);

    // Per-moving-link mass scales for link1..link7 (identified inertial parameters).
    // Applied to URDF inertials before building the KDL tree. Call before Init(),
    // or call ReinitWithMassScales() after.
    void SetLinkMassScales(const std::array<double, 7>& scales);
    bool ReinitWithMassScales(const std::array<double, 7>& scales);

    // Per-link CoM deltas [m] in each link frame (link1..link7), from inertial ID.
    // Added to URDF inertial origin before building the KDL tree.
    void SetLinkComDeltas(const std::array<double, 21>& deltas_xyz);

private:
    bool BuildChainFromUrdf();

    double ee_point_mass_ = 0.0;
    Eigen::Vector3d ee_point_offset_ = Eigen::Vector3d::Zero();
    double wrist_camera_mass_ = 0.0;
    Eigen::Vector3d wrist_camera_offset_ = Eigen::Vector3d(0.0, 0.08, 0.04);
    double urdf_gravity_scale_ = 1.0;
    KDL::Vector gravity_vec_{0.0, 0.0, -9.81};
    std::array<double, 7> link_mass_scales_{{1, 1, 1, 1, 1, 1, 1}};
    std::array<double, 21> link_com_deltas_{};  // link1..7 × (dx,dy,dz)
    std::array<double, 7> distal_ee_gain_{{0.24, 0.255, 0.45, 0.15, 0.60, 1.0, 1.1}};
    std::array<double, 7> distal_cam_gain_{{0.30, 0.32, 0.50, 0.30, 1.20, 0.75, 0.45}};
};
