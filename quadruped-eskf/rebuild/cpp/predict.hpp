// Module 9: the predict step in C++/Eigen.
//   1. Do WORKSHEET.md, Module 9 first (Python -> Eigen translation, fixed-size matrices).
//   2. Fill in the four TODOs (about 25 lines), translating your Python eskf.py (Module 4).
//   3. Build and run:  cmake --build build && ./build/rebuild_tests
// Real project: cpp/src/eskf.cpp, propagateNominal (line 24), coreTransition (34), predict (56).
#pragma once

#include <stdexcept>

#include "state.hpp"

namespace rebuild {

inline void propagateNominal(State& x, const Vec3& acc, const Vec3& gyro, double dt) {
  // TODO (6 lines): R = x.q.toRotationMatrix(); a_w = R (acc - ba) + kGravity; w = gyro - bg;
  // p, then v, then q = (q * quatExp(w dt)).normalized()
  (void)x; (void)acc; (void)gyro; (void)dt;
  throw std::logic_error("TODO: propagateNominal is not written yet");
}

inline Mat15 errorTransition(const State& x, const Vec3& acc, const Vec3& gyro, double dt) {
  // TODO (about 12 lines): start from Mat15::Identity() and fill the same 7 blocks as in Python.
  // A 3x3 block is F.block<3, 3>(row, col), e.g. F.block<3, 3>(IV, ITH) = -R * skew(a) * dt;
  (void)x; (void)acc; (void)gyro; (void)dt;
  throw std::logic_error("TODO: errorTransition is not written yet");
}

inline Mat15 processNoise(const Params& prm, double dt) {
  // TODO (5 lines): Mat15::Zero(), then each 3x3 diagonal block,
  // e.g. Q.block<3, 3>(IV, IV).diagonal().setConstant(prm.acc_noise * prm.acc_noise * dt);
  (void)prm; (void)dt;
  throw std::logic_error("TODO: processNoise is not written yet");
}

inline void predict(State& x, const Params& prm, const Vec3& acc, const Vec3& gyro, double dt) {
  // TODO (4 lines): F first, then move the state, then P = F P F^T + Q, then symmetrize.
  // Careful: x.P = (0.5 * (x.P + x.P.transpose())).eval();  (.eval() because x.P is on both sides)
  (void)x; (void)prm; (void)acc; (void)gyro; (void)dt;
  throw std::logic_error("TODO: predict is not written yet");
}

}  // namespace rebuild
