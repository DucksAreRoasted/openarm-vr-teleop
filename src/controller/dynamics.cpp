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

#include <controller/dynamics.hpp>

#include <algorithm>
#include <cmath>

Dynamics::Dynamics(std::string urdf_path, std::string start_link, std::string end_link) {
    this->urdf_path = urdf_path;
    this->start_link = start_link;
    this->end_link = end_link;
}

Dynamics::~Dynamics() {}

bool Dynamics::Init() {
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        fprintf(stderr, "Failed to open URDF file: %s\n", urdf_path.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    urdf_model_interface = urdf::parseURDF(buffer.str());
    if (!urdf_model_interface) {
        fprintf(stderr, "Failed to parse URDF: %s\n", urdf_path.c_str());
        return false;
    }

    const std::string side_token = (end_link.find("left") != std::string::npos) ? "left" : "right";
    for (int i = 0; i < 7; ++i) {
        const std::string link_name = "openarm_" + side_token + "_link" + std::to_string(i + 1);
        auto link = urdf_model_interface->getLink(link_name);
        if (!link || !link->inertial) {
            continue;
        }
        const double s = std::clamp(link_mass_scales_[static_cast<size_t>(i)], 0.2, 3.0);
        link->inertial->mass *= s;
        link->inertial->ixx *= s;
        link->inertial->ixy *= s;
        link->inertial->ixz *= s;
        link->inertial->iyy *= s;
        link->inertial->iyz *= s;
        link->inertial->izz *= s;
        const size_t base = static_cast<size_t>(i) * 3;
        link->inertial->origin.position.x += link_com_deltas_[base + 0];
        link->inertial->origin.position.y += link_com_deltas_[base + 1];
        link->inertial->origin.position.z += link_com_deltas_[base + 2];
    }

    return BuildChainFromUrdf();
}

bool Dynamics::BuildChainFromUrdf() {
    if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface, kdl_tree)) {
        fprintf(stderr, "Failed to extract KDL tree: %s\n", urdf_path.c_str());
        return false;
    }

    if (!kdl_tree.getChain(start_link, end_link, kdl_chain)) {
        fprintf(stderr, "Failed to get KDL chain\n");
        return false;
    }

    std::cout << "[GetGravity] kdl_chain.getNrOfJoints() = " << kdl_chain.getNrOfJoints()
              << std::endl;

    coriolis_forces.resize(kdl_chain.getNrOfJoints());
    gravity_forces.resize(kdl_chain.getNrOfJoints());
    inertia_matrix.resize(kdl_chain.getNrOfJoints());

    coriolis_forces.data.setZero();
    gravity_forces.data.setZero();
    inertia_matrix.data.setZero();

    solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, gravity_vec_);

    Eigen::Matrix<double, 6, 1> ik_weights_;
    ik_weights_ << 1.0, 1.0, 1.0, 0.2, 0.2, 0.2;
    pos_ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain, ik_weights_);

    q_min_.assign(kdl_chain.getNrOfJoints(), -M_PI);
    q_max_.assign(kdl_chain.getNrOfJoints(), M_PI);
    joint_limits_valid_ = false;
    size_t joint_idx = 0;
    for (unsigned int seg = 0; seg < kdl_chain.getNrOfSegments(); ++seg) {
        const KDL::Joint& joint = kdl_chain.getSegment(seg).getJoint();
        if (joint.getType() == KDL::Joint::None) {
            continue;
        }
        if (joint_idx >= q_min_.size()) {
            break;
        }
        const std::string name = joint.getName();
        const auto urdf_joint = urdf_model_interface->getJoint(name);
        if (urdf_joint && urdf_joint->limits) {
            q_min_[joint_idx] = urdf_joint->limits->lower;
            q_max_[joint_idx] = urdf_joint->limits->upper;
            joint_limits_valid_ = true;
        }
        ++joint_idx;
    }

    return true;
}

void Dynamics::SetLinkMassScales(const std::array<double, 7>& scales) {
    link_mass_scales_ = scales;
}

void Dynamics::SetLinkComDeltas(const std::array<double, 21>& deltas_xyz) {
    for (size_t i = 0; i < link_com_deltas_.size(); ++i) {
        link_com_deltas_[i] = std::clamp(deltas_xyz[i], -0.08, 0.08);
    }
}

bool Dynamics::ReinitWithMassScales(const std::array<double, 7>& scales) {
    link_mass_scales_ = scales;
    return Init();
}

bool Dynamics::GetJointPositionLimits(std::vector<double> *q_min, std::vector<double> *q_max) const {
    if (q_min == nullptr || q_max == nullptr || !joint_limits_valid_) {
        return false;
    }
    *q_min = q_min_;
    *q_max = q_max_;
    return true;
}

void Dynamics::ClampJointPositions(double *q_inout) const {
    if (q_inout == nullptr || q_min_.empty()) {
        return;
    }
    for (size_t i = 0; i < q_min_.size(); ++i) {
        q_inout[i] = std::clamp(q_inout[i], q_min_[i], q_max_[i]);
    }
}

void Dynamics::SetEndEffectorPointMass(double mass_kg, double ox, double oy, double oz) {
    ee_point_mass_ = std::max(0.0, mass_kg);
    ee_point_offset_ = Eigen::Vector3d(ox, oy, oz);
}

void Dynamics::SetWristCameraMass(double mass_kg, bool left_arm, double ox, double oy,
                                  double oz) {
    wrist_camera_mass_ = std::max(0.0, mass_kg);
    // Default: camera sits beside/above the wrist in link7. Lateral Y is the
    // dominant off-axis lever (torques j5 roll / j6 yaw); +Z levers j7 pitch.
    // Right arm Y is mirrored. Override via finite ox/oy/oz from yaml.
    if (std::isfinite(ox) && std::isfinite(oy) && std::isfinite(oz)) {
        wrist_camera_offset_ = Eigen::Vector3d(ox, oy, oz);
    } else {
        const double y = left_arm ? 0.08 : -0.08;
        wrist_camera_offset_ = Eigen::Vector3d(0.0, y, 0.04);
    }
}

void Dynamics::SetDistalMassGains(const std::array<double, 7>& ee_gain,
                                  const std::array<double, 7>& cam_gain) {
    distal_ee_gain_ = ee_gain;
    distal_cam_gain_ = cam_gain;
}

void Dynamics::SetUrdfGravityScale(double scale) {
    urdf_gravity_scale_ = std::clamp(scale, 0.2, 3.0);
}

void Dynamics::SetGravityVector(double gx, double gy, double gz) {
    gravity_vec_ = KDL::Vector(gx, gy, gz);
    if (kdl_chain.getNrOfJoints() > 0) {
        solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, gravity_vec_);
    }
}

void Dynamics::GetGravity(const double *motor_position, double *gravity) {
    const auto njoints = kdl_chain.getNrOfJoints();

    KDL::JntArray q_(njoints);

    for (size_t i = 0; i < njoints; i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToGravity(q_, gravity_forces);
    for (size_t i = 0; i < njoints; i++) {
        gravity[i] = urdf_gravity_scale_ * gravity_forces(i);
    }

    // Fingers + gripper motor sit past link7, so KDL G(q) does not include them
    // on joints 4–7. τ = m * axis · (g × r), r from that joint origin to the mass.
    // The wrist camera may remain installed when the gripper is removed.
    if ((ee_point_mass_ > 1e-9 || wrist_camera_mass_ > 1e-9) && gravity != nullptr && njoints >= 7) {
        Eigen::Matrix3d R7;
        Eigen::Vector3d p_link7;
        GetEECordinate(motor_position, R7, p_link7);
        const Eigen::Vector3d p_mass = p_link7 + R7 * ee_point_offset_;
        const Eigen::Vector3d p_cam = p_link7 + R7 * wrist_camera_offset_;
        const Eigen::Vector3d g(0.0, 0.0, -9.81);

        const Eigen::Vector3d axis7_local(0.0, end_link.find("left") != std::string::npos ? -1.0 : 1.0, 0.0);
        const Eigen::Vector3d axis7 = R7 * axis7_local;
        gravity[6] += distal_ee_gain_[6] * ee_point_mass_ * axis7.dot(g.cross(p_mass - p_link7));
        if (wrist_camera_mass_ > 1e-9) {
            gravity[6] += distal_cam_gain_[6] * wrist_camera_mass_ * axis7.dot(g.cross(p_cam - p_link7));
        }

        // Tip segment is link7. link6 / link5 / link4 are 1 / 2 / 3 segments before it.
        const int nseg = static_cast<int>(kdl_chain.getNrOfSegments());
        Eigen::Matrix3d R6;
        Eigen::Vector3d p_link6;
        GetPreEECordinate(motor_position, R6, p_link6);
        const Eigen::Vector3d axis6 = R6.col(0);
        gravity[5] += distal_ee_gain_[5] * ee_point_mass_ * axis6.dot(g.cross(p_mass - p_link6));
        if (wrist_camera_mass_ > 1e-9) {
            gravity[5] += distal_cam_gain_[5] * wrist_camera_mass_ * axis6.dot(g.cross(p_cam - p_link6));
        }

        Eigen::Matrix3d R5;
        Eigen::Vector3d p_link5;
        if (GetSegmentCordinate(motor_position, nseg - 2, R5, p_link5)) {
            // joint5 (forearm roll, +Z): full EE torque felt too floaty.
            gravity[4] += distal_ee_gain_[4] * ee_point_mass_ * R5.col(2).dot(g.cross(p_mass - p_link5));
        }
        Eigen::Matrix3d R4;
        Eigen::Vector3d p_link4;
        if (GetSegmentCordinate(motor_position, nseg - 3, R4, p_link4)) {
            // Elbow (+Y). Full gripper moment held the forearm up, so downward
            // motion felt heavy. Camera is a separate mass above the wrist.
            const Eigen::Vector3d axis4 = R4.col(1);
            gravity[3] += distal_ee_gain_[3] * ee_point_mass_ * axis4.dot(g.cross(p_mass - p_link4));
            if (wrist_camera_mass_ > 1e-9) {
                gravity[3] += distal_cam_gain_[3] * wrist_camera_mass_ * axis4.dot(g.cross(p_cam - p_link4));
            }
        }
        Eigen::Matrix3d R3;
        Eigen::Vector3d p_link3;
        if (GetSegmentCordinate(motor_position, nseg - 4, R3, p_link3)) {
            // joint3 (upper-arm roll, +Z): full distal mass made downward motion heavy.
            const Eigen::Vector3d axis3 = R3.col(2);
            gravity[2] += distal_ee_gain_[2] * ee_point_mass_ * axis3.dot(g.cross(p_mass - p_link3));
            if (wrist_camera_mass_ > 1e-9) {
                gravity[2] += distal_cam_gain_[2] * wrist_camera_mass_ * axis3.dot(g.cross(p_cam - p_link3));
            }
        }
        Eigen::Matrix3d R2;
        Eigen::Vector3d p_link2;
        if (GetSegmentCordinate(motor_position, nseg - 5, R2, p_link2)) {
            // joint2 (shoulder pitch, -X): long lever arm; full distal mass felt floaty.
            const Eigen::Vector3d axis2 = -R2.col(0);
            gravity[1] += distal_ee_gain_[1] * ee_point_mass_ * axis2.dot(g.cross(p_mass - p_link2));
            if (wrist_camera_mass_ > 1e-9) {
                gravity[1] += distal_cam_gain_[1] * wrist_camera_mass_ * axis2.dot(g.cross(p_cam - p_link2));
            }
        }
        Eigen::Matrix3d R1;
        Eigen::Vector3d p_link1;
        if (GetSegmentCordinate(motor_position, nseg - 6, R1, p_link1)) {
            // joint1 (shoulder yaw, +Z): longest lever arm; slightly below joint2.
            const Eigen::Vector3d axis1 = R1.col(2);
            gravity[0] += distal_ee_gain_[0] * ee_point_mass_ * axis1.dot(g.cross(p_mass - p_link1));
            if (wrist_camera_mass_ > 1e-9) {
                gravity[0] += distal_cam_gain_[0] * wrist_camera_mass_ * axis1.dot(g.cross(p_cam - p_link1));
            }
        }

        // Camera sits above the wrist, off the forearm roll axis (joint5 = +Z).
        if (wrist_camera_mass_ > 1e-9) {
            Eigen::Matrix3d R5c;
            Eigen::Vector3d p_link5c;
            if (GetSegmentCordinate(motor_position, nseg - 2, R5c, p_link5c)) {
                gravity[4] += distal_cam_gain_[4] * wrist_camera_mass_ * R5c.col(2).dot(g.cross(p_cam - p_link5c));
            }
        }
    }
}

void Dynamics::GetCoriolis(const double *motor_position, const double *motor_velocity,
                           double *coriolis) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntArray q_dot(kdl_chain.getNrOfJoints());

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
        q_dot(i) = motor_velocity[i];
    }

    solver->JntToCoriolis(q_, q_dot, coriolis_forces);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        coriolis[i] = coriolis_forces(i);
    }
}

void Dynamics::GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntSpaceInertiaMatrix inertia_matrix(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToMass(q_, inertia_matrix);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        inertia_diag[i] = inertia_matrix(i, i);
    }
}

void Dynamics::GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::Jacobian kdl_jac(kdl_chain.getNrOfJoints());
    KDL::ChainJntToJacSolver jac_solver(kdl_chain);
    jac_solver.JntToJac(q_, kdl_jac);

    jacobian = Eigen::MatrixXd(6, kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < kdl_chain.getNrOfJoints(); ++j) {
            jacobian(i, j) = kdl_jac(i, j);
        }
    }
}

void Dynamics::GetNullSpace(const double *motor_position, Eigen::MatrixXd &nullspace) {
    const size_t dof = kdl_chain.getNrOfJoints();

    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    nullspace = I - J_pinv * J;

    //        std::cout << "[INFO] Null space projector computed.\n";
}

void Dynamics::GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T) {
    const size_t dof = kdl_chain.getNrOfJoints();
    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    Eigen::MatrixXd N = I - J_pinv * J;

    nullspace_T = N.transpose();
}

void Dynamics::GetEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                              Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame) < 0) {
        //  std::cerr << "[KDL] FK failed in GetEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}

void Dynamics::GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                                 Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame, kdl_chain.getNrOfSegments() - 1) < 0) {
        //        std::cerr << "[KDL] FK failed in GetPreEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}

bool Dynamics::GetSegmentCordinate(const double *motor_position, int segment_index,
                                   Eigen::Matrix3d &R, Eigen::Vector3d &p) {
    const int nseg = static_cast<int>(kdl_chain.getNrOfSegments());
    if (segment_index < 0 || segment_index > nseg) {
        return false;
    }
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;
    if (fk_solver.JntToCart(q_, kdl_frame, segment_index) < 0) {
        return false;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);
    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
    return true;
}

double Dynamics::ComputeMinSingularValue(const double *motor_position) {
    Eigen::MatrixXd jacobian;
    GetJacobian(motor_position, jacobian);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues().size() == 0) {
        return 0.0;
    }
    return svd.singularValues().tail(1)(0);
}

bool Dynamics::SolveDlsIk(const double *motor_position, const Eigen::VectorXd &v_cart,
                          double lambda, Eigen::VectorXd *dq_out, double *sigma_min_out,
                          const std::vector<double> *joint_mobility_weights) {
    if (dq_out == nullptr || v_cart.size() != 6) {
        return false;
    }

    const size_t dof = kdl_chain.getNrOfJoints();
    Eigen::MatrixXd jacobian;
    GetJacobian(motor_position, jacobian);

    // Mobility diagonal W: higher weight => joint is preferred in the solution.
    // q_dot = W J^T (J W J^T + λ² I)^{-1} v
    Eigen::VectorXd w = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dof));
    if (joint_mobility_weights != nullptr && !joint_mobility_weights->empty()) {
        for (size_t i = 0; i < dof && i < joint_mobility_weights->size(); ++i) {
            const double wi = (*joint_mobility_weights)[i];
            w[static_cast<Eigen::Index>(i)] = (wi > 1e-6 && std::isfinite(wi)) ? wi : 1.0;
        }
    }
    const Eigen::MatrixXd jw = jacobian * w.asDiagonal();  // J * W

    const Eigen::MatrixXd A =
        jw * jacobian.transpose() + (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() != Eigen::Success) {
        dq_out->setZero(static_cast<Eigen::Index>(dof));
        if (sigma_min_out != nullptr) {
            *sigma_min_out = ComputeMinSingularValue(motor_position);
        }
        return false;
    }
    const Eigen::VectorXd y = ldlt.solve(v_cart);
    if (ldlt.info() != Eigen::Success || !y.allFinite()) {
        dq_out->setZero(static_cast<Eigen::Index>(dof));
        if (sigma_min_out != nullptr) {
            *sigma_min_out = ComputeMinSingularValue(motor_position);
        }
        return false;
    }
    *dq_out = w.asDiagonal() * jacobian.transpose() * y;

    if (sigma_min_out != nullptr) {
        *sigma_min_out = ComputeMinSingularValue(motor_position);
    }
    return dq_out->size() == static_cast<Eigen::Index>(dof) && dq_out->allFinite();
}

bool Dynamics::SolvePosIk(const double *motor_position, const Eigen::Matrix3d &R_des,
                          const Eigen::Vector3d &p_des, double *q_out) {
    if (q_out == nullptr) {
        return false;
    }

    KDL::JntArray q_seed(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_seed(i) = motor_position[i];
    }

    KDL::Rotation target_rot;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target_rot(i, j) = R_des(i, j);
        }
    }
    const KDL::Frame target_frame(target_rot, KDL::Vector(p_des.x(), p_des.y(), p_des.z()));

    if (!pos_ik_solver_) {
        return false;
    }
    KDL::JntArray q_result(kdl_chain.getNrOfJoints());
    const int ret = pos_ik_solver_->CartToJnt(q_seed, target_frame, q_result);
    if (ret < 0) {
        return false;
    }

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_out[i] = q_result(i);
    }
    return true;
}

bool Dynamics::SolvePosIkPosition(const double *motor_position, const Eigen::Vector3d &p_des,
                                  double *q_out) {
    if (q_out == nullptr) {
        return false;
    }
    Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
    GetEECordinate(motor_position, r_curr, p_curr);
    return SolvePosIk(motor_position, r_curr, p_des, q_out);
}

bool Dynamics::RefineIkOrientation(const double *motor_position,
                                   const Eigen::Quaterniond &q_des, double dt, double *q_out) {
    if (q_out == nullptr) {
        return false;
    }

    Eigen::Matrix3d r_fk = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_fk = Eigen::Vector3d::Zero();
    GetEECordinate(motor_position, r_fk, p_fk);

    const Eigen::Quaterniond q_fk(r_fk);
    Eigen::Quaterniond q_err =
        (q_des.normalized() * q_fk.normalized().inverse()).normalized();
    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }
    Eigen::AngleAxisd axis_angle(q_err);
    double angle = axis_angle.angle();
    if (!std::isfinite(angle) || angle < 0.03) {
        return true;
    }

    Eigen::VectorXd v_cart(6);
    v_cart.setZero();
    v_cart.tail<3>() = axis_angle.axis() * (angle / std::max(dt, 1e-3));

    Eigen::VectorXd dq_step;
    if (!SolveDlsIk(motor_position, v_cart, 0.08, &dq_step)) {
        return false;
    }

    const size_t dof = kdl_chain.getNrOfJoints();
    for (size_t i = 0; i < dof; ++i) {
        q_out[i] = motor_position[i] + dq_step[static_cast<Eigen::Index>(i)] * dt;
    }
    return true;
}
