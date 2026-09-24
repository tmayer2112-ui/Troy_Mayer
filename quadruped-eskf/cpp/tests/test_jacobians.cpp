// Every analytic Jacobian in the filter against central finite differences on the manifold.
#include <gtest/gtest.h>

#include <random>

#include "qeskf/eskf.hpp"

using namespace qeskf;

namespace {

Joints standJoints(std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 0.1);
  Joints q;
  for (int leg = 0; leg < 4; ++leg) q.segment<3>(3 * leg) << 0.05 + n(rng), 0.8 + n(rng), -1.6 + n(rng);
  return q;
}

Vec3 randn3(std::mt19937& rng, double s) {
  std::normal_distribution<double> n(0.0, s);
  return Vec3(n(rng), n(rng), n(rng));
}

Eskf randomFilter(int n_feet, unsigned seed, Joints* q_out) {
  std::mt19937 rng(seed);
  Eskf f(EskfParams{}, randn3(rng, 1.0), randn3(rng, 0.5), quatExp(randn3(rng, 0.3)), randn3(rng, 0.05),
         randn3(rng, 0.01), Eigen::MatrixXd::Identity(kNB, kNB) * 1e-4);
  const Joints q = standJoints(rng);
  for (int leg = 0; leg < n_feet; ++leg) f.augment(leg, q);
  if (q_out) *q_out = q;
  return f;
}

// x [+] dx, same convention as Eskf::inject but without touching P.
Eskf boxplus(const Eskf& f, const Eigen::VectorXd& dx) {
  Eskf g = f;
  g.setState(f.p() + dx.segment<3>(kIP), f.v() + dx.segment<3>(kIV),
             quatMul(f.q(), quatExp(dx.segment<3>(kITH))), f.ba() + dx.segment<3>(kIBA),
             f.bg() + dx.segment<3>(kIBG));
  for (size_t j = 0; j < f.order().size(); ++j) {
    const int leg = f.order()[j];
    g.setFoot(leg, f.foot(leg) + dx.segment<3>(kNB + 3 * j));
  }
  return g;
}

// a [-] b
Eigen::VectorXd boxminus(const Eskf& a, const Eskf& b) {
  Eigen::VectorXd dx(a.dim());
  dx.segment<3>(kIP) = a.p() - b.p();
  dx.segment<3>(kIV) = a.v() - b.v();
  dx.segment<3>(kITH) = logSO3(quatToRot(b.q()).transpose() * quatToRot(a.q()));
  dx.segment<3>(kIBA) = a.ba() - b.ba();
  dx.segment<3>(kIBG) = a.bg() - b.bg();
  for (size_t j = 0; j < a.order().size(); ++j) {
    const int leg = a.order()[j];
    dx.segment<3>(kNB + 3 * j) = a.foot(leg) - b.foot(leg);
  }
  return dx;
}

constexpr double kStep = 1e-6;
constexpr double kTol = 1e-8;

}  // namespace

class JacobianTest : public ::testing::TestWithParam<unsigned> {};

TEST_P(JacobianTest, TransitionMatchesFiniteDifferences) {
  std::mt19937 rng(GetParam() + 100);
  const Eskf f = randomFilter(2, GetParam(), nullptr);
  const Vec3 acc = randn3(rng, 3.0) + Vec3(0, 0, 9.8);
  const Vec3 gyr = randn3(rng, 2.0);
  const double dt = 1e-3;
  const Eigen::MatrixXd F = f.errorTransition(acc, gyr, dt);
  Eskf ref = f;
  ref.propagateNominal(acc, gyr, dt);
  Eigen::MatrixXd Ffd(f.dim(), f.dim());
  for (int i = 0; i < f.dim(); ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(f.dim());
    e(i) = kStep;
    Eskf gp = boxplus(f, e), gm = boxplus(f, -e);
    gp.propagateNominal(acc, gyr, dt);
    gm.propagateNominal(acc, gyr, dt);
    Ffd.col(i) = (boxminus(gp, ref) - boxminus(gm, ref)) / (2 * kStep);
  }
  EXPECT_LT((F - Ffd).cwiseAbs().maxCoeff(), kTol);
}

TEST_P(JacobianTest, MeasurementMatchesFiniteDifferences) {
  Joints q;
  const Eskf f = randomFilter(3, GetParam(), &q);
  const std::vector<int> legs = f.order();
  Eigen::VectorXd r, rp, rm;
  Eigen::MatrixXd H, Hd, Rn;
  f.measurement(legs, q, r, H, Rn);
  Eigen::MatrixXd Hfd(H.rows(), H.cols());
  for (int i = 0; i < f.dim(); ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(f.dim());
    e(i) = kStep;
    boxplus(f, e).measurement(legs, q, rp, Hd, Rn);
    boxplus(f, -e).measurement(legs, q, rm, Hd, Rn);
    Hfd.col(i) = -(rp - rm) / (2 * kStep);  // r = z - h(x)
  }
  EXPECT_LT((H - Hfd).cwiseAbs().maxCoeff(), kTol);
}

TEST_P(JacobianTest, AugmentationMatchesFiniteDifferences) {
  Joints q;
  const Eskf f = randomFilter(1, GetParam(), &q);
  const int leg = 3;
  auto newFoot = [&](const Eskf& g) { return Vec3(g.p() + quatToRot(g.q()) * footPosition(leg, legJoints(q, leg))); };
  Eigen::MatrixXd Jfd(3, f.dim());
  for (int i = 0; i < f.dim(); ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(f.dim());
    e(i) = kStep;
    Jfd.col(i) = (newFoot(boxplus(f, e)) - newFoot(boxplus(f, -e))) / (2 * kStep);
  }
  Eskf g = f;
  const Eigen::MatrixXd P_before = f.P();
  g.augment(leg, q);
  const int n = f.dim();
  // The cross-covariance the filter writes must be exactly J * P.
  EXPECT_LT((g.P().bottomLeftCorner(3, n) - Jfd * P_before).cwiseAbs().maxCoeff(), 1e-10);
}

TEST_P(JacobianTest, FootKinematicsJacobian) {
  std::mt19937 rng(GetParam());
  const Joints q = standJoints(rng);
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector3d ql = legJoints(q, leg);
    Eigen::Matrix3d Jfd;
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e(i) = kStep;
      Jfd.col(i) = (footPosition(leg, ql + e) - footPosition(leg, ql - e)) / (2 * kStep);
    }
    EXPECT_LT((footJacobian(leg, ql) - Jfd).cwiseAbs().maxCoeff(), kTol);
  }
}

INSTANTIATE_TEST_SUITE_P(Seeds, JacobianTest, ::testing::Range(0u, 5u));
