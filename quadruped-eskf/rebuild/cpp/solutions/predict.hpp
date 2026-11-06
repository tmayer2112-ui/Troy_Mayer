// Module 9: the predict step in C++/Eigen. REFERENCE SOLUTION.
// The same maths as your Python eskf.py (Module 4), without the feet.
// Real project: cpp/src/eskf.cpp, propagateNominal (line 24), coreTransition (34), predict (56).
#pragma once

#include "state.hpp"

namespace rebuild {

inline void propagateNominal(State& x, const Vec3& acc, const Vec3& gyro, double dt) {
  const Mat3 R = x.q.toRotationMatrix();
  const Vec3 a_w = R * (acc - x.ba) + kGravity;
  const Vec3 w = gyro - x.bg;
  x.p += x.v * dt + 0.5 * a_w * dt * dt;
  x.v += a_w * dt;
  x.q = (x.q * quatExp(w * dt)).normalized();
}

inline Mat15 errorTransition(const State& x, const Vec3& acc, const Vec3& gyro, double dt) {
  const Mat3 R = x.q.toRotationMatrix();
  const Vec3 a = acc - x.ba;
  const Vec3 w = gyro - x.bg;
  const Mat3 RA = R * skew(a);
  Mat15 F = Mat15::Identity();
  F.block<3, 3>(IP, IV) = Mat3::Identity() * dt;
  F.block<3, 3>(IP, ITH) = -0.5 * RA * dt * dt;
  F.block<3, 3>(IP, IBA) = -0.5 * R * dt * dt;
  F.block<3, 3>(IV, ITH) = -RA * dt;
  F.block<3, 3>(IV, IBA) = -R * dt;
  F.block<3, 3>(ITH, ITH) = expSO3(-w * dt);
  F.block<3, 3>(ITH, IBG) = -rightJacobian(w * dt) * dt;
  return F;
}

inline Mat15 processNoise(const Params& prm, double dt) {
  Mat15 Q = Mat15::Zero();
  Q.block<3, 3>(IV, IV).diagonal().setConstant(prm.acc_noise * prm.acc_noise * dt);
  Q.block<3, 3>(ITH, ITH).diagonal().setConstant(prm.gyro_noise * prm.gyro_noise * dt);
  Q.block<3, 3>(IBA, IBA).diagonal().setConstant(prm.acc_rw * prm.acc_rw * dt);
  Q.block<3, 3>(IBG, IBG).diagonal().setConstant(prm.gyro_rw * prm.gyro_rw * dt);
  return Q;
}

inline void predict(State& x, const Params& prm, const Vec3& acc, const Vec3& gyro, double dt) {
  const Mat15 F = errorTransition(x, acc, gyro, dt);   // before the nominal state moves
  propagateNominal(x, acc, gyro, dt);
  x.P = F * x.P * F.transpose() + processNoise(prm, dt);
  x.P = (0.5 * (x.P + x.P.transpose())).eval();
}

}  // namespace rebuild
