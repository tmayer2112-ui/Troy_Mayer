// Module 9 tests (provided). The same three ideas as the main project's GoogleTests:
// Jacobians against finite differences, invariants, and C++ against the Python reference.
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include "fd.hpp"
#include "predict.hpp"

using namespace rebuild;

namespace {

State randomState(unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);
  auto r3 = [&](double s) { return Vec3(s * n(rng), s * n(rng), s * n(rng)); };
  State x;
  x.p = r3(1.0);
  x.v = r3(0.5);
  x.q = quatExp(r3(0.3));
  x.ba = r3(0.05);
  x.bg = r3(0.01);
  x.P = Mat15::Identity() * 1e-4;
  return x;
}

struct Fixture {
  int n = 0;
  double dt = 0.0;
  Params prm;
  State x0;
  std::vector<Vec3> acc, gyro;
  std::vector<int> check_step;
  std::vector<State> check_state;
};

State readState(std::istream& in, bool with_P) {
  State x;
  double w, qx, qy, qz;
  in >> x.p.x() >> x.p.y() >> x.p.z() >> x.v.x() >> x.v.y() >> x.v.z() >> w >> qx >> qy >> qz;
  x.q = Quat(w, qx, qy, qz);
  in >> x.ba.x() >> x.ba.y() >> x.ba.z() >> x.bg.x() >> x.bg.y() >> x.bg.z();
  if (with_P)
    for (int i = 0; i < 15; ++i)
      for (int j = 0; j < 15; ++j) in >> x.P(i, j);
  return x;
}

Fixture loadFixture() {
  std::ifstream in(FIXTURE);
  Fixture f;
  if (!in) return f;
  in >> f.n >> f.dt >> f.prm.acc_noise >> f.prm.gyro_noise >> f.prm.acc_rw >> f.prm.gyro_rw;
  f.x0 = readState(in, true);
  f.acc.resize(f.n);
  f.gyro.resize(f.n);
  for (int k = 0; k < f.n; ++k)
    in >> f.acc[k].x() >> f.acc[k].y() >> f.acc[k].z() >> f.gyro[k].x() >> f.gyro[k].y() >> f.gyro[k].z();
  int m = 0;
  in >> m;
  for (int c = 0; c < m; ++c) {
    int step = 0;
    in >> step;
    f.check_step.push_back(step);
    f.check_state.push_back(readState(in, true));
  }
  return f;
}

}  // namespace

// --- 1. Jacobian: your F against your finite differences, 5 random states ---
class Jacobian : public ::testing::TestWithParam<unsigned> {};

TEST_P(Jacobian, TransitionMatchesFiniteDifferences) {
  std::mt19937 rng(GetParam() + 100);
  std::normal_distribution<double> n(0.0, 1.0);
  const State x = randomState(GetParam());
  const Vec3 acc(3 * n(rng), 3 * n(rng), 9.8 + 3 * n(rng));
  const Vec3 gyro(2 * n(rng), 2 * n(rng), 2 * n(rng));
  const Mat15 F = errorTransition(x, acc, gyro, 1e-3);
  const Mat15 Ffd = fdTransition(x, acc, gyro, 1e-3);
  Eigen::Index i, j;
  const double err = (F - Ffd).cwiseAbs().maxCoeff(&i, &j);
  EXPECT_LT(err, 1e-8) << "worst entry F(" << i << ", " << j << "): yours " << F(i, j) << ", nudged " << Ffd(i, j);
}

INSTANTIATE_TEST_SUITE_P(Seeds, Jacobian, ::testing::Range(0u, 5u));

// --- 2. Invariants: P stays symmetric and grows during predict ---
TEST(Invariants, CovarianceStaysSymmetricAndGrows) {
  State x = randomState(7);
  const double trace0 = x.P.trace();
  for (int k = 0; k < 100; ++k) predict(x, Params{}, Vec3(0.1, 0.0, 9.8), Vec3(0.0, 0.0, 0.3), 1e-3);
  EXPECT_EQ((x.P - x.P.transpose()).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_GT(x.P.trace(), trace0);
  EXPECT_NEAR(x.q.norm(), 1.0, 1e-12);
}

// --- 3. Parity: 1000 predicts in C++ against the Python reference on the same IMU samples ---
TEST(Parity, CppMatchesPythonReference) {
  const Fixture f = loadFixture();
  ASSERT_GT(f.n, 0) << "could not read " << FIXTURE;
  State x = f.x0;
  double dp = 0, dv = 0, dth = 0, dP = 0;
  size_t c = 0;
  for (int k = 0; k < f.n; ++k) {
    predict(x, f.prm, f.acc[k], f.gyro[k], f.dt);
    if (c < f.check_step.size() && k + 1 == f.check_step[c]) {
      const State& y = f.check_state[c++];
      dp = std::max(dp, (x.p - y.p).cwiseAbs().maxCoeff());
      dv = std::max(dv, (x.v - y.v).cwiseAbs().maxCoeff());
      dth = std::max(dth, logSO3(y.q.toRotationMatrix().transpose() * x.q.toRotationMatrix()).norm());
      dP = std::max(dP, ((x.P - y.P).cwiseAbs().array() / (y.P.cwiseAbs().array() + 1e-12)).maxCoeff());
    }
  }
  std::cout << "      C++ vs Python after " << f.n << " predicts: max |dp| " << dp << " m, |dv| " << dv
            << " m/s, |dtheta| " << dth << " rad, relative |dP| " << dP << "\n";
  EXPECT_LT(dp, 1e-10);
  EXPECT_LT(dv, 1e-10);
  EXPECT_LT(dth, 1e-10);
  EXPECT_LT(dP, 1e-8);
}
