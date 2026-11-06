// Module 9: the finite-difference Jacobian of one predict step, the C++ twin of fd_jacobian (Module 3).
// Real project: cpp/tests/test_jacobians.cpp, TransitionMatchesFiniteDifferences.
#pragma once

#include <stdexcept>

#include "predict.hpp"

namespace rebuild {

inline Mat15 fdTransition(const State& x, const Vec3& acc, const Vec3& gyro, double dt, double h = 1e-6) {
  // TODO (about 12 lines):
  //   ref = x moved one step with propagateNominal
  //   for each of the 15 error directions i: e = Vec15::Zero(); e(i) = h;
  //     nudge up = boxplus(x, e) and down = boxplus(x, -e), move both one step,
  //     column i = (boxminus(up, ref) - boxminus(down, ref)) / (2 h)
  (void)x; (void)acc; (void)gyro; (void)dt; (void)h;
  throw std::logic_error("TODO: fdTransition is not written yet");
}

}  // namespace rebuild
