#include "qeskf/gait.hpp"

#include <algorithm>
#include <cmath>

namespace qeskf {
namespace {
// Same geometry as kinematics.cpp / python/qeskf/kinematics.py.
constexpr double kHipX[4] = {0.1881, 0.1881, -0.1881, -0.1881};
constexpr double kHipY[4] = {-0.04675, 0.04675, -0.04675, 0.04675};
constexpr double kSide[4] = {-1.0, 1.0, -1.0, 1.0};
constexpr double kThighY = 0.08;
constexpr double kLThigh = 0.213;
constexpr double kLCalf = 0.213;
constexpr double kPi = 3.14159265358979323846;
// Trot: diagonal pairs FR+RL and FL+RR are half a cycle apart.
constexpr double kPhaseOffset[4] = {0.0, 0.5, 0.5, 0.0};
// models/go1/go1.xml joint ranges: abduction, hip (thigh), knee (calf).
constexpr double kQMin[3] = {-0.863, -0.686, -2.818};
constexpr double kQMax[3] = {0.863, 4.501, -0.888};

// Python's float %: result takes the sign of the divisor.
double pyMod(double a, double b) {
  double r = std::fmod(a, b);
  if (r != 0.0) {
    if ((b < 0) != (r < 0)) r += b;
  } else {
    r = std::copysign(0.0, b);
  }
  return r;
}

double smoothstep(double x) { return x * x * (3.0 - 2.0 * x); }
}  // namespace

Eigen::Vector3d nominalFoot(int leg) { return {kHipX[leg], kHipY[leg] + kSide[leg] * kThighY, 0.0}; }

Eigen::Vector3d legIK(int leg, const Eigen::Vector3d& p, double knee_sign) {
  const Eigen::Vector3d v = p - Eigen::Vector3d(kHipX[leg], kHipY[leg], 0.0);
  const double d = kSide[leg] * kThighY;
  const double r_yz2 = v(1) * v(1) + v(2) * v(2);
  const double z = -std::sqrt(std::max(r_yz2 - d * d, 1e-9));
  double q1 = std::atan2(v(2), v(1)) - std::atan2(z, d);
  q1 = pyMod(q1 + kPi, 2 * kPi) - kPi;
  const double x = v(0);
  const double r2 = x * x + z * z;
  const double c3 = std::clamp((r2 - kLThigh * kLThigh - kLCalf * kLCalf) / (2 * kLThigh * kLCalf), -1.0, 1.0);
  const double q3 = knee_sign * std::acos(c3);
  const double q2 = std::atan2(-x, -z) - std::atan2(kLCalf * std::sin(q3), kLThigh + kLCalf * std::cos(q3));
  return {q1, q2, q3};
}

Joints clampToJointLimits(const Joints& q) {
  Joints out;
  for (int i = 0; i < 12; ++i) out(i) = std::clamp(q(i), kQMin[i % 3], kQMax[i % 3]);
  return out;
}

Eigen::Vector4d TrotController::legPhase(double t) const {
  const double tg = std::max(t - p_.stand_time, 0.0);
  Eigen::Vector4d ph;
  for (int i = 0; i < 4; ++i) ph(i) = pyMod(tg / p_.period + kPhaseOffset[i], 1.0);
  return ph;
}

Stance TrotController::scheduledStance(double t) const {
  if (t < p_.stand_time) return {true, true, true, true};
  const Eigen::Vector4d ph = legPhase(t);
  return {ph(0) < p_.duty, ph(1) < p_.duty, ph(2) < p_.duty, ph(3) < p_.duty};
}

FootTargets TrotController::footTargets(double t, const Eigen::Vector3d& cmd, const Eigen::Vector3d& v_body,
                                        const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega_body,
                                        std::optional<double> height) {
  FootTargets targets;
  const double vx = cmd(0), vy = cmd(1), wz = cmd(2);
  const double t_st = p_.period * p_.duty;
  const double roll = rpy(0), pitch = rpy(1);
  // Servo compliance lets the trunk sag under load; push the stance feet down to compensate.
  const double h_err = height ? p_.height - *height : 0.0;
  h_int_ = h_int_ + p_.k_height_int * h_err;
  const double h_cmd = p_.height + p_.k_height * h_err + h_int_;
  const bool walking = t >= p_.stand_time;
  const Eigen::Vector4d phase = legPhase(t);
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d nom = nominalFoot(i);
    const double dz = p_.k_att * (roll * nom(1) - pitch * nom(0)) +
                      p_.d_att * (omega_body(0) * nom(1) - omega_body(1) * nom(0));
    if (!walking) {
      targets.row(i) << nom(0), nom(1), -h_cmd + dz;
      continue;
    }
    // Foot xy at stance progress s in [0,1]: fixed in the world => moves backwards in the body frame.
    auto stance_xy = [&](double s) -> Eigen::Vector2d {
      const double tau = (s - 0.5) * t_st;
      const double a = -wz * tau, c = std::cos(a), sn = std::sin(a);
      return {c * nom(0) + (-sn) * nom(1) - vx * tau, sn * nom(0) + c * nom(1) - vy * tau};
    };
    const double ph = phase(i);
    Eigen::Vector2d xy;
    double z;
    if (ph < p_.duty) {
      xy = stance_xy(ph / p_.duty);
      z = -h_cmd + dz;
    } else {
      const double s = (ph - p_.duty) / (1.0 - p_.duty);
      const Eigen::Vector2d start = stance_xy(1.0);
      // Ground-speed matching: the touchdown target moves backwards along the extrapolated stance path,
      // so by s = 0.8 the foot already moves at -v in the body frame and lands without skidding.
      const double s_early = -(1.0 - s) * (1.0 - p_.duty) / p_.duty;
      const Eigen::Vector2d end =
          stance_xy(s_early) + p_.raibert_k * Eigen::Vector2d(v_body(0) - vx, v_body(1) - vy);
      const double b = smoothstep(std::min(s / 0.8, 1.0));
      xy = (1 - b) * start + b * end;
      // Raised-cosine lift: zero vertical velocity at liftoff and touchdown.
      z = -h_cmd + dz + p_.swing_height * 0.5 * (1.0 - std::cos(2.0 * kPi * s));
    }
    targets.row(i) << xy(0), xy(1), z;
  }
  return targets;
}

Joints TrotController::jointTargets(double t, const Eigen::Vector3d& cmd, const Eigen::Vector3d& v_body,
                                    const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega_body,
                                    std::optional<double> height) {
  const FootTargets tg = footTargets(t, cmd, v_body, rpy, omega_body, height);
  Joints q;
  for (int i = 0; i < 4; ++i) q.segment<3>(3 * i) = legIK(i, tg.row(i).transpose());
  return q;
}

}  // namespace qeskf
