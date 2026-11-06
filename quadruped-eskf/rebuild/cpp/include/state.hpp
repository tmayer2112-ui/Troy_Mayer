// Module 9 (provided): types, rotation helpers and the error-state nudges, in C++/Eigen.
// Same conventions as the Python rebuild and the main project (cpp/include/qeskf/so3.hpp):
//   quaternions are Hamilton, q rotates body-frame vectors into the world, R_true = R_est * Exp(dtheta).
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace rebuild {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat15 = Eigen::Matrix<double, 15, 15>;   // fixed size: lives on the stack, no heap allocation

// Where each piece of the 15-number error state lives (same as IP, IV, ... in eskf.py)
constexpr int IP = 0, IV = 3, ITH = 6, IBA = 9, IBG = 12;
constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;
inline const Vec3 kGravity(0.0, 0.0, -kG);

struct Params {                       // the same numbers as Params in eskf.py
  double acc_noise = 175e-6 * kG;     // m/s^2/sqrt(Hz)
  double gyro_noise = 0.014 * kPi / 180.0;
  double acc_rw = 1.7e-4;
  double gyro_rw = 1.7e-6;
};

struct State {                        // the nominal state and the error covariance (no feet here)
  Vec3 p, v;
  Quat q;
  Vec3 ba, bg;
  Mat15 P;
};

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
  return th / (2.0 * std::sin(th)) * w;
}

inline Mat3 rightJacobian(const Vec3& phi) {
  const double th = phi.norm();
  const Mat3 K = skew(phi);
  if (th < 1e-6) return Mat3::Identity() - 0.5 * K + K * K / 6.0;
  return Mat3::Identity() - (1.0 - std::cos(th)) / (th * th) * K + (th - std::sin(th)) / (th * th * th) * K * K;
}

inline Quat quatExp(const Vec3& phi) {
  const double th = phi.norm();
  if (th < 1e-8) return Quat(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()).normalized();
  const double s = std::sin(0.5 * th) / th;
  return Quat(std::cos(0.5 * th), s * phi.x(), s * phi.y(), s * phi.z());
}

// x [+] dx: apply an error the way inject() does (P untouched). Used by the finite-difference test.
inline State boxplus(const State& x, const Vec15& dx) {
  State y = x;
  y.p = x.p + dx.segment<3>(IP);
  y.v = x.v + dx.segment<3>(IV);
  y.q = x.q * quatExp(dx.segment<3>(ITH));
  y.ba = x.ba + dx.segment<3>(IBA);
  y.bg = x.bg + dx.segment<3>(IBG);
  return y;
}

// a [-] b: the error that takes b to a.
inline Vec15 boxminus(const State& a, const State& b) {
  Vec15 dx;
  dx.segment<3>(IP) = a.p - b.p;
  dx.segment<3>(IV) = a.v - b.v;
  dx.segment<3>(ITH) = logSO3(b.q.toRotationMatrix().transpose() * a.q.toRotationMatrix());
  dx.segment<3>(IBA) = a.ba - b.ba;
  dx.segment<3>(IBG) = a.bg - b.bg;
  return dx;
}

}  // namespace rebuild
