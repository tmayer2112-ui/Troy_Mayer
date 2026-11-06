// Module 9: the finite-difference Jacobian of one predict step. REFERENCE SOLUTION.
// The C++ twin of fd_jacobian from Module 3, on the manifold (boxplus / boxminus from state.hpp).
// Real project: cpp/tests/test_jacobians.cpp, TransitionMatchesFiniteDifferences.
#pragma once

#include "predict.hpp"

namespace rebuild {

inline Mat15 fdTransition(const State& x, const Vec3& acc, const Vec3& gyro, double dt, double h = 1e-6) {
  State ref = x;
  propagateNominal(ref, acc, gyro, dt);
  Mat15 J;
  for (int i = 0; i < 15; ++i) {
    Vec15 e = Vec15::Zero();
    e(i) = h;
    State up = boxplus(x, e);
    State down = boxplus(x, -e);
    propagateNominal(up, acc, gyro, dt);
    propagateNominal(down, acc, gyro, dt);
    J.col(i) = (boxminus(up, ref) - boxminus(down, ref)) / (2.0 * h);
  }
  return J;
}

}  // namespace rebuild
