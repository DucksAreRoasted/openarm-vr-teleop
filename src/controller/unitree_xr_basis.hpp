#pragma once

#include <Eigen/Dense>

// Rotation of T_ROBOT_OPENXR in televuer/src/televuer/tv_wrapper.py.
// OpenXR Y-up maps to robot Z-up: -Z -> +X, -X -> +Y, +Y -> +Z.
inline Eigen::Matrix3d unitree_robot_from_openxr() {
    Eigen::Matrix3d basis;
    basis << 0.0, 0.0, -1.0,
             -1.0, 0.0, 0.0,
             0.0, 1.0, 0.0;
    return basis;
}
