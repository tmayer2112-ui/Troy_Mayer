// Numerical invariants that must hold over long random runs.
#include <gtest/gtest.h>

#include <random>

#include "qeskf/eskf.hpp"

using namespace qeskf;

namespace {
Eskf makeFilter() {
  return Eskf(EskfParams{}, Vec3::Zero(), Vec3::Zero(), Quat::Identity(), Vec3::Zero(), Vec3::Zero(),
              Eigen::MatrixXd::Identity(kNB, kNB) * 1e-4);
}
Joints wiggle(int k) {
  Joints q;
  for (int i = 0; i < 12; ++i) {
    const double base = (i % 3 == 0) ? 0.05 : (i % 3 == 1 ? 0.8 : -1.6);
    q(i) = base + 0.05 * std::sin(0.01 * k + i);
  }
  return q;
}
}  // namespace

TEST(Invariants, CovarianceStaysSymmetricPositiveDefinite) {
  Eskf f = makeFilter();
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  Contacts c{false, false, false, false};
  for (int k = 0; k < 20000; ++k) {
    if (k % 97 == 0)
      for (auto& ci : c) ci = u(rng) > 0.4;
    const Vec3 acc(0.5 * n(rng), 0.5 * n(rng), 9.8 + 0.5 * n(rng));
    const Vec3 gyr(0.3 * n(rng), 0.3 * n(rng), 0.3 * n(rng));
    f.step(acc, gyr, wiggle(k), c, 1e-3);
    if (k % 500 == 0) {
      const Eigen::MatrixXd& P = f.P();
      ASSERT_EQ((P - P.transpose()).cwiseAbs().maxCoeff(), 0.0) << "at step " << k;
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
      ASSERT_GT(es.eigenvalues().minCoeff(), 0.0) << "at step " << k;
    }
  }
}

TEST(Invariants, QuaternionStaysUnitNorm) {
  Eskf f = makeFilter();
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int k = 0; k < 100000; ++k)
    f.propagateNominal(Vec3(n(rng), n(rng), 9.8 + n(rng)), Vec3(5 * n(rng), 5 * n(rng), 5 * n(rng)), 1e-3);
  EXPECT_NEAR(f.q().norm(), 1.0, 1e-12);
}

TEST(Invariants, MarginalizationIsExactGaussianMarginal) {
  Eskf f = makeFilter();
  const Joints q = wiggle(0);
  for (int leg = 0; leg < 3; ++leg) f.augment(leg, q);
  const Eigen::MatrixXd P = f.P();
  f.marginalize(1);  // middle foot: rows/cols 18..20
  ASSERT_EQ(f.dim(), kNB + 6);
  EXPECT_EQ(f.P().topLeftCorner(kNB + 3, kNB + 3), P.topLeftCorner(kNB + 3, kNB + 3));
  EXPECT_EQ(f.P().bottomRightCorner(3, 3), P.bottomRightCorner(3, 3));
  EXPECT_EQ(f.P().bottomLeftCorner(3, kNB + 3), P.block(kNB + 6, 0, 3, kNB + 3));
  EXPECT_EQ(f.order(), (std::vector<int>{0, 2}));
}

TEST(Invariants, StructuredPredictEqualsDenseFPFt) {
  // predict() skips the identity part of F; with Q = 0 it must equal the dense F P F^T.
  EskfParams zeroQ;
  zeroQ.acc_noise = zeroQ.gyro_noise = zeroQ.acc_rw = zeroQ.gyro_rw = zeroQ.foot_noise = 0.0;
  Eskf f(zeroQ, Vec3(0.1, 0.2, 0.3), Vec3(0.4, -0.1, 0.0), quatExp(Vec3(0.1, -0.2, 0.3)), Vec3::Zero(),
         Vec3::Zero(), Eigen::MatrixXd::Identity(kNB, kNB) * 1e-4);
  const Joints q = wiggle(0);
  f.augment(0, q);
  f.augment(2, q);
  f.mutableP() += Eigen::MatrixXd::Constant(f.dim(), f.dim(), 1e-6);  // non-trivial cross terms
  const Vec3 acc(0.3, -0.2, 9.9), gyr(0.1, 0.4, -0.3);
  const Eigen::MatrixXd F = f.errorTransition(acc, gyr, 1e-3);
  const Eigen::MatrixXd dense = F * f.P() * F.transpose();
  f.predict(acc, gyr, 1e-3);
  EXPECT_LT((f.P() - dense).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(Invariants, So3Roundtrip) {
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 3.1);
  for (int i = 0; i < 100; ++i) {
    Vec3 phi(n(rng), n(rng), n(rng));
    phi *= u(rng) / phi.norm();
    EXPECT_LT((logSO3(expSO3(phi)) - phi).norm(), 1e-9);
    EXPECT_LT((quatToRot(quatExp(phi)) - expSO3(phi)).cwiseAbs().maxCoeff(), 1e-12);
  }
}
