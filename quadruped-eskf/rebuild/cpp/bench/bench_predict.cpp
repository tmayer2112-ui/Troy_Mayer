// Module 9 timing (provided): how long one predict takes, and what fixed-size matrices buy you.
#include <chrono>
#include <cstdio>

#include "predict.hpp"

using namespace rebuild;
using Clock = std::chrono::steady_clock;

int main() {
  State x;
  x.p.setZero();
  x.v = Vec3(0.5, 0.0, 0.0);
  x.q = Quat::Identity();
  x.ba.setZero();
  x.bg.setZero();
  x.P = Mat15::Identity() * 1e-4;
  const Params prm;
  const Vec3 acc(0.1, 0.0, 9.81), gyro(0.01, 0.02, 0.3);
  const int n = 200000;
  volatile double sink = 0;

  auto t0 = Clock::now();
  for (int k = 0; k < n; ++k) predict(x, prm, acc, gyro, 1e-3);
  const double us_predict = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / n;
  sink = sink + x.P.trace();

  // F P F^T with fixed-size (Mat15) vs dynamic-size (MatrixXd) matrices: same maths, same numbers
  const Mat15 F = errorTransition(x, acc, gyro, 1e-3);
  const Mat15 P = x.P;
  const Eigen::MatrixXd Fd = F, Pd = P;
  t0 = Clock::now();
  for (int k = 0; k < n; ++k) {
    const Mat15 r = F * P * F.transpose();
    sink = sink + r(0, 0);
  }
  const double ns_fixed = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / n;
  t0 = Clock::now();
  for (int k = 0; k < n; ++k) {
    const Eigen::MatrixXd r = Fd * Pd * Fd.transpose();
    sink = sink + r(0, 0);
  }
  const double ns_dynamic = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / n;

  std::printf("one predict (15 states):        %.2f us   (budget at 1 kHz: 1000 us)\n", us_predict);
  std::printf("F P F^T, fixed-size Mat15:      %.0f ns\n", ns_fixed);
  std::printf("F P F^T, dynamic-size MatrixXd: %.0f ns\n", ns_dynamic);
  std::printf("At 15x15 the two cost about the same on average. Fixed size still means no heap allocation\n"
              "inside the 1 kHz loop: one less source of an unpredictable delay. (Your project's one 21 ms\n"
              "spike was the host OS, not the filter: DEBUG_LOG #12. That's why it reports median and p99.)\n");
  return sink == 12345.0;  // keeps the compiler from deleting the loops
}
