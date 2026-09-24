#include "qeskf/kinematics.hpp"

#include <cmath>

namespace qeskf {
namespace {
constexpr double kHipX[4] = {0.1881, 0.1881, -0.1881, -0.1881};
constexpr double kHipY[4] = {-0.04675, 0.04675, -0.04675, 0.04675};
constexpr double kSide[4] = {-1.0, 1.0, -1.0, 1.0};
constexpr double kThighY = 0.08;
constexpr double kLThigh = 0.213;
constexpr double kLCalf = 0.213;
}  // namespace

Eigen::Vector3d footPosition(int leg, const Eigen::Vector3d& q) {
  const double s1 = std::sin(q(0)), c1 = std::cos(q(0));
  const double s2 = std::sin(q(1)), c2 = std::cos(q(1));
  const double s23 = std::sin(q(1) + q(2)), c23 = std::cos(q(1) + q(2));
  const double d = kSide[leg] * kThighY;
  const double x = -kLThigh * s2 - kLCalf * s23;
  const double z = -kLThigh * c2 - kLCalf * c23;
  return {kHipX[leg] + x, kHipY[leg] + c1 * d - s1 * z, s1 * d + c1 * z};
}

Eigen::Matrix3d footJacobian(int leg, const Eigen::Vector3d& q) {
  const double s1 = std::sin(q(0)), c1 = std::cos(q(0));
  const double s2 = std::sin(q(1)), c2 = std::cos(q(1));
  const double s23 = std::sin(q(1) + q(2)), c23 = std::cos(q(1) + q(2));
  const double d = kSide[leg] * kThighY;
  const double z = -kLThigh * c2 - kLCalf * c23;
  const double dx2 = -kLThigh * c2 - kLCalf * c23, dx3 = -kLCalf * c23;
  const double dz2 = kLThigh * s2 + kLCalf * s23, dz3 = kLCalf * s23;
  Eigen::Matrix3d J;
  J << 0.0, dx2, dx3,
       -s1 * d - c1 * z, -s1 * dz2, -s1 * dz3,
       c1 * d - s1 * z, c1 * dz2, c1 * dz3;
  return J;
}

}  // namespace qeskf
