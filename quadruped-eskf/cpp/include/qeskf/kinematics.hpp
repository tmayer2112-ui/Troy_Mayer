// Go1 leg kinematics, trunk (= IMU) frame. Mirrors python/qeskf/kinematics.py.
#pragma once

#include <Eigen/Dense>

namespace qeskf {

using Joints = Eigen::Matrix<double, 12, 1>;  // FR, FL, RR, RL x (abduction, hip, knee)

Eigen::Vector3d footPosition(int leg, const Eigen::Vector3d& q);
Eigen::Matrix3d footJacobian(int leg, const Eigen::Vector3d& q);

inline Eigen::Vector3d legJoints(const Joints& q, int leg) { return q.segment<3>(3 * leg); }

}  // namespace qeskf
