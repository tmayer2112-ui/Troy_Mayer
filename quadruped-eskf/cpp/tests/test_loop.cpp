// Estimator-in-the-loop controller: C++ EstimatorController vs the Python closed loop.
// Fixture: python/scripts/make_loop_fixture.py -> cpp/tests/data/loop_python.csv + loop_config.txt
// (a 4.5 s estimate-fed Python run: 2 s standing calibration, then trotting).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "qeskf/io.hpp"
#include "qeskf/loop.hpp"

using namespace qeskf;

#ifndef QESKF_TEST_DATA
#error "QESKF_TEST_DATA must point at cpp/tests/data"
#endif

namespace {
LoopConfig configFromFile(const std::string& path) {
  const Config c = readConfig(path);
  LoopConfig cfg;
  cfg.eskf = paramsFromConfig(c);
  cfg.dt = scalar(c, "dt");
  cfg.imu.rate_hz = scalar(c, "rate_hz");
  cfg.imu.acc_noise_density = scalar(c, "acc_noise_density");
  cfg.imu.gyro_noise_density = scalar(c, "gyro_noise_density");
  cfg.imu.acc_turn_on_bias = scalar(c, "acc_turn_on_bias");
  cfg.imu.acc_rrw = scalar(c, "acc_rrw");
  cfg.cal_start = scalar(c, "cal_start");
  cfg.t_start = scalar(c, "t_start");
  cfg.liftoff_advance = scalar(c, "liftoff_advance");
  cfg.use_schedule = scalar(c, "use_schedule") != 0.0;
  cfg.yaw0 = scalar(c, "yaw0");
  const auto p0 = need(c, "p0", 3);
  cfg.p0 = Vec3(p0[0], p0[1], p0[2]);
  GaitParams& g = cfg.gait;
  g.period = scalar(c, "gait_period");
  g.duty = scalar(c, "gait_duty");
  g.height = scalar(c, "gait_height");
  g.swing_height = scalar(c, "gait_swing_height");
  g.raibert_k = scalar(c, "gait_raibert_k");
  g.k_att = scalar(c, "gait_k_att");
  g.d_att = scalar(c, "gait_d_att");
  g.stand_time = scalar(c, "gait_stand_time");
  g.k_height = scalar(c, "gait_k_height");
  return cfg;
}

struct Tick {
  Vec3 acc, gyro, cmd, est_v;
  Joints qenc, q_ref;
  Contacts c;
};

std::vector<Tick> loadTicks() {
  std::vector<Tick> out;
  for (const auto& x : readCsv(std::string(QESKF_TEST_DATA) + "/loop_python.csv")) {
    // columns: t, acc(3), gyro(3), qenc(12), contact(4), cmd(3), q_cmd(12), est_v(3)
    Tick k;
    k.acc << x[1], x[2], x[3];
    k.gyro << x[4], x[5], x[6];
    for (int i = 0; i < 12; ++i) k.qenc(i) = x[7 + i];
    for (int i = 0; i < 4; ++i) k.c[i] = x[19 + i] > 0.5;
    k.cmd << x[23], x[24], x[25];
    for (int i = 0; i < 12; ++i) k.q_ref(i) = x[26 + i];
    k.est_v << x[38], x[39], x[40];
    out.push_back(k);
  }
  return out;
}
}  // namespace

TEST(Loop, CppMatchesPythonClosedLoop) {
  const LoopConfig cfg = configFromFile(std::string(QESKF_TEST_DATA) + "/loop_config.txt");
  const auto ticks = loadTicks();
  ASSERT_GT(ticks.size(), 4000u);
  EstimatorController loop(cfg);
  double max_dq_stand = 0, max_dq_walk = 0, max_dv = 0;
  int max_feet = 0, min_feet = 4;
  std::vector<double> us;
  for (size_t k = 0; k < ticks.size(); ++k) {
    const Tick& x = ticks[k];
    const auto t0 = std::chrono::steady_clock::now();
    const Joints& q = loop.step(x.acc, x.gyro, x.qenc, x.c, x.cmd);
    const auto t1 = std::chrono::steady_clock::now();
    const double dq = (q - x.q_ref).cwiseAbs().maxCoeff();
    if (!loop.running()) {
      max_dq_stand = std::max(max_dq_stand, dq);
      continue;
    }
    us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    max_dq_walk = std::max(max_dq_walk, dq);
    max_dv = std::max(max_dv, (loop.filter()->v() - x.est_v).cwiseAbs().maxCoeff());
    const int nf = static_cast<int>(loop.filter()->order().size());
    max_feet = std::max(max_feet, nf);
    if (k > 2500) min_feet = std::min(min_feet, nf);
  }
  std::sort(us.begin(), us.end());
  std::printf("estimator + trot controller, %zu ticks: max |q_cpp - q_python| standing %.3e rad, with the filter "
              "%.3e rad; max |v_cpp - v_python| %.3e m/s\n", ticks.size(), max_dq_stand, max_dq_walk, max_dv);
  std::printf("step(): median %.1f us, p99 %.1f us, max %.1f us over %zu filter ticks (this machine)\n",
              us[us.size() / 2], us[us.size() * 99 / 100], us.back(), us.size());
  EXPECT_TRUE(loop.running());
  EXPECT_LT(max_dq_stand, 1e-12);
  EXPECT_LT(max_dq_walk, 1e-9);
  EXPECT_LT(max_dv, 1e-9);
  EXPECT_EQ(max_feet, 4);   // the run exercises touchdown/liftoff bookkeeping while trotting
  EXPECT_LE(min_feet, 2);
}

TEST(Loop, StaticAlignmentRecoversTilt) {
  const double roll = 0.05, pitch = -0.03, yaw = 0.4;
  const Mat3 R = (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                  Eigen::AngleAxisd(roll, Vec3::UnitX())).toRotationMatrix();
  const Vec3 f = R.transpose() * Vec3(0, 0, 9.80665) + Vec3(0, 0, 0.1);   // at rest + a z bias
  Mat3 R0;
  Vec3 ba, bg;
  staticAlignment(f, Vec3(0.001, -0.002, 0.003), yaw, R0, ba, bg);
  const Vec3 rpy = rpyFromRot(R0);
  EXPECT_NEAR(rpy(0), roll, 1e-3);   // first order in the z bias
  EXPECT_NEAR(rpy(1), pitch, 1e-3);
  EXPECT_NEAR(rpy(2), yaw, 1e-12);
  EXPECT_NEAR(ba(2), 0.1, 1e-3);
  EXPECT_EQ(bg, Vec3(0.001, -0.002, 0.003));
  EXPECT_LT((quatToRot(rotToQuat(R0)) - R0).norm(), 1e-12);
}

TEST(Loop, InitialCovarianceIsSymmetricPositiveSemidefinite) {
  const CoreMat P = initialCovariance(ImuNoise(), 1500);
  EXPECT_LT((P - P.transpose()).norm(), 1e-18);
  const Eigen::SelfAdjointEigenSolver<CoreMat> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-15);
  // tilt and horizontal accel bias are correlated (leveling absorbs one into the other)
  EXPECT_NE(P(kITH + 1, kIBA + 0), 0.0);
}
