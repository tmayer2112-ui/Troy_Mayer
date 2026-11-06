// Trot controller: C++ vs the Python reference, IK vs forward kinematics, no heap allocation per step.
// Fixture: python/scripts/make_gait_fixture.py -> cpp/tests/data/gait_python.csv (inputs and Python outputs).
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "qeskf/gait.hpp"
#include "qeskf/io.hpp"
#include "qeskf/kinematics.hpp"

using namespace qeskf;

// Count every heap allocation in this test binary, so a test can check a code path makes none.
static std::atomic<long> g_allocs{0};
void* operator new(std::size_t n) {
  ++g_allocs;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

#ifndef QESKF_TEST_DATA
#error "QESKF_TEST_DATA must point at cpp/tests/data"
#endif

namespace {
struct Row {
  double t;
  Eigen::Vector3d cmd, v, rpy, w;
  std::optional<double> h;
};
Row parse(const std::vector<double>& x) {
  Row r;
  r.t = x[0];
  r.cmd << x[1], x[2], x[3];
  r.v << x[4], x[5], x[6];
  r.rpy << x[7], x[8], x[9];
  r.w << x[10], x[11], x[12];
  if (!std::isnan(x[13])) r.h = x[13];
  return r;
}
}  // namespace

TEST(Gait, CppMatchesPythonReference) {
  const auto rows = readCsv(std::string(QESKF_TEST_DATA) + "/gait_python.csv");
  ASSERT_GT(rows.size(), 2000u);
  TrotController ctrl;
  double max_dq = 0;
  int stance_mismatch = 0, walking = 0, no_height = 0, swing = 0;
  for (const auto& x : rows) {
    const Row r = parse(x);
    const Joints q = ctrl.jointTargets(r.t, r.cmd, r.v, r.rpy, r.w, r.h);
    for (int i = 0; i < 12; ++i) max_dq = std::max(max_dq, std::abs(q(i) - x[14 + i]));
    const Stance st = ctrl.scheduledStance(r.t);
    for (int i = 0; i < 4; ++i) {
      stance_mismatch += st[i] != (x[26 + i] != 0.0);
      swing += !st[i];
    }
    walking += r.t >= ctrl.params().stand_time;
    no_height += !r.h.has_value();
  }
  std::printf("trot controller, %zu steps: max |q_cpp - q_python| = %.3e rad\n", rows.size(), max_dq);
  EXPECT_EQ(stance_mismatch, 0);
  EXPECT_LT(max_dq, 1e-12);
  // The fixture must exercise every branch: standing, stance, swing, and no height measurement.
  EXPECT_GT(walking, 0);
  EXPECT_LT(walking, static_cast<int>(rows.size()));
  EXPECT_GT(swing, 0);
  EXPECT_GT(no_height, 0);
}

TEST(Gait, IkInvertsForwardKinematics) {
  for (int leg = 0; leg < 4; ++leg) {
    for (double dx : {-0.08, 0.0, 0.1}) {
      for (double dy : {-0.04, 0.0, 0.05}) {
        for (double h : {0.2, 0.27, 0.32}) {
          const Eigen::Vector3d p = nominalFoot(leg) + Eigen::Vector3d(dx, dy, -h);
          const Eigen::Vector3d q = legIK(leg, p);
          EXPECT_LT((footPosition(leg, q) - p).norm(), 1e-12) << "leg " << leg;
          EXPECT_LT(q(2), 0.0);  // knee bent backwards, as on the robot
        }
      }
    }
  }
}

TEST(Gait, StandingIsSymmetricAndWithinJointLimits) {
  TrotController ctrl;
  const Eigen::Vector3d z = Eigen::Vector3d::Zero();
  const Joints q = ctrl.jointTargets(0.0, z, z, z, z, ctrl.params().height);
  EXPECT_NEAR(q(0), -q(3), 1e-12);  // FR / FL abduction mirror each other
  EXPECT_NEAR(q(1), q(4), 1e-12);   // and share hip and knee
  EXPECT_NEAR(q(2), q(5), 1e-12);
  EXPECT_NEAR(q(1), q(7), 1e-12);   // front and rear legs are the same at the neutral point
  EXPECT_LT((clampToJointLimits(q) - q).norm(), 1e-15);
  for (int leg = 0; leg < 4; ++leg)
    EXPECT_NEAR(footPosition(leg, q.segment<3>(3 * leg))(2), -ctrl.params().height, 1e-12);
  Joints wild = Joints::Constant(10.0);
  const Joints c = clampToJointLimits(wild);
  EXPECT_DOUBLE_EQ(c(0), 0.863);
  EXPECT_DOUBLE_EQ(c(1), 4.501);
  EXPECT_DOUBLE_EQ(c(2), -0.888);
}

TEST(Gait, StepDoesNotAllocate) {
  const auto rows = readCsv(std::string(QESKF_TEST_DATA) + "/gait_python.csv");
  std::vector<Row> in;
  for (const auto& x : rows) in.push_back(parse(x));
  TrotController ctrl;
  Joints sink = Joints::Zero();
  std::vector<double> us;
  us.reserve(in.size());
  const long before = g_allocs.load();
  for (const Row& r : in) {
    const auto t0 = std::chrono::steady_clock::now();
    const Joints q = ctrl.jointTargets(r.t, r.cmd, r.v, r.rpy, r.w, r.h);
    const auto t1 = std::chrono::steady_clock::now();
    sink += clampToJointLimits(q);
    us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  EXPECT_EQ(g_allocs.load() - before, 0) << "jointTargets() allocated on the heap";
  std::sort(us.begin(), us.end());
  std::printf("jointTargets: median %.2f us, p99 %.2f us over %zu calls (this machine)\n", us[us.size() / 2],
              us[us.size() * 99 / 100], us.size());
  EXPECT_TRUE(sink.allFinite());
}
