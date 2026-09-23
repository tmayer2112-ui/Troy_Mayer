// SO(3) helpers. Conventions match python/qeskf/so3.py exactly:
//   Hamilton quaternions, q_wb maps body vectors to world, local (right) attitude error
//   R_true = R_hat * Exp(dtheta).
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace qeskf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

inline Mat3 skew(const Vec3& v) {
  Mat3 S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return S;
}

inline Mat3 expSO3(const Vec3& phi) {
  const double th = phi.norm();
  const Mat3 K = skew(phi);
  if (th < 1e-8) return Mat3::Identity() + K + 0.5 * K * K;
  return Mat3::Identity() + std::sin(th) / th * K + (1.0 - std::cos(th)) / (th * th) * K * K;
}

inline Vec3 logSO3(const Mat3& R) {
  const double c = std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0);
  const double th = std::acos(c);
  const Vec3 w(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  if (th < 1e-8) return 0.5 * w;
  if (M_PI - th < 1e-6) {
    const Mat3 B = 0.5 * (R + Mat3::Identity());
    Vec3 axis = B.diagonal().cwiseMax(0.0).cwiseSqrt();
    int k;
    axis.maxCoeff(&k);
    axis = B.row(k).transpose() / axis(k);
    return th * axis.normalized();
  }
  return th / (2.0 * std::sin(th)) * w;
}

inline Quat quatExp(const Vec3& phi) {
  const double th = phi.norm();
  if (th < 1e-8) {
    Quat q(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z());
    return q.normalized();
  }
  const double s = std::sin(0.5 * th) / th;
  return Quat(std::cos(0.5 * th), s * phi.x(), s * phi.y(), s * phi.z());
}

// Same closed form as python quat_to_rot (Eigen's toRotationMatrix rounds differently).
inline Mat3 quatToRot(const Quat& q) {
  const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  Mat3 R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
       2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
       2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return R;
}

inline Quat quatMul(const Quat& a, const Quat& b) {
  return Quat(a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
              a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
              a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
              a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w());
}

inline Mat3 rightJacobian(const Vec3& phi) {
  const double th = phi.norm();
  const Mat3 K = skew(phi);
  if (th < 1e-6) return Mat3::Identity() - 0.5 * K + K * K / 6.0;
  return Mat3::Identity() - (1.0 - std::cos(th)) / (th * th) * K + (th - std::sin(th)) / (th * th * th) * K * K;
}

}  // namespace qeskf
