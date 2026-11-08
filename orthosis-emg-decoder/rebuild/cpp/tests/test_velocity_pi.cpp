// Module 9 tests (provided). The same three kinds as the firmware's GoogleTests (firmware/sim/):
// numbers worked out by hand, behaviour on a plant, and C++ against a reference.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "velocity_pi.hpp"

#ifdef FIRMWARE_CORE
#include FIRMWARE_CORE
#endif

using namespace rebuild;

namespace {

// A first-order plant, tau d(omega)/dt = K u - omega, integrated exactly over each 1 ms tick.
struct FirstOrder {
  double K, tau, omega = 0.0;
  void step(double u, double dt) {
    const double a = std::exp(-dt / tau);
    omega = a * omega + (1.0 - a) * K * u;
  }
};

struct Tick {
  int scenario, antiwindup;
  double ref, meas;
  int ramping;
  double duty, integ;
};

std::vector<Tick> loadFixture(double& dt) {
  std::ifstream in(FIXTURE);
  std::vector<Tick> ticks;
  int n = 0;
  if (!(in >> n >> dt)) return ticks;
  ticks.resize(n);
  for (Tick& t : ticks) in >> t.scenario >> t.antiwindup >> t.ref >> t.meas >> t.ramping >> t.duty >> t.integ;
  return ticks;
}

}  // namespace

// --- 1. By hand (worksheet 6.2-6.3, 9.1) ----------------------------------------------------------------
TEST(ByHand, ImcGainsMatchMetricsJson) {
  const Gains g = imcGains(187.0, 0.020, 0.050);
  EXPECT_NEAR(g.kp, 0.0021390374, 1e-10);  // firmware/results/metrics.json: kp_duty_per_dps 0.00214
  EXPECT_NEAR(g.ki, 0.1069518717, 1e-10);  //                                ki_duty_per_deg 0.1070
}

TEST(ByHand, FeedForward) {
  EXPECT_NEAR(feedforward(15.0, 0.0, 187.0, 0.020, 0.05), 15.0 / 187.0 + 0.05, 1e-15);
  EXPECT_NEAR(feedforward(0.3, 0.0, 187.0, 0.020, 0.05), 0.3 / 187.0, 1e-15);  // no friction under 0.5 deg/s
  EXPECT_NEAR(feedforward(-10.0, -150.0, 187.0, 0.020, 0.05), -(13.0 / 187.0 + 0.05), 1e-15);
}

TEST(ByHand, OnePiStep) {
  VelocityPI c;
  c.prev_ref = 10.0;
  EXPECT_NEAR(c.step(10.0, 8.0, 0.001), 10.0 / 187.0 + 0.05 + 2.0 * 0.020 / (187.0 * 0.050), 1e-15);
  EXPECT_NEAR(c.integ, 2.0 * 0.001 / (187.0 * 0.050), 1e-15);
}

// --- 2. Behaviour --------------------------------------------------------------------------------------------
TEST(AntiWindup, SaturatedAndPushingFurtherFreezesTheIntegral) {
  VelocityPI c;
  c.prev_ref = 20.0;
  c.integ = 0.5;
  EXPECT_EQ(c.step(20.0, 0.0, 0.001), 0.6);
  EXPECT_EQ(c.integ, 0.5);
  c.step(20.0, 30.0, 0.001);  // the joint is ahead: e < 0 may still discharge it
  EXPECT_LT(c.integ, 0.5);
}

TEST(AntiWindup, ARampingSetpointCannotChargeTheIntegral) {
  VelocityPI c;
  c.prev_ref = 10.0;
  c.step(10.0, 8.0, 0.001, /*ramping=*/true);
  EXPECT_EQ(c.integ, 0.0);
  c.prev_ref = 10.0;
  c.step(10.0, 12.0, 0.001, /*ramping=*/true);
  EXPECT_NEAR(c.integ, -2.0 * 0.001 / (187.0 * 0.050), 1e-15);
}

TEST(Behaviour, ImcPromiseOnAnExactModel) {
  // PI alone (no feed-forward, no friction) on a plant equal to the model: 63 % of a step at t = lambda.
  VelocityPI c;
  c.friction = 0.0;
  c.K = std::numeric_limits<double>::infinity();  // (ref + tau dref) / inf = 0: feed-forward off
  FirstOrder plant{187.0, 0.020};
  int t63 = -1;
  for (int k = 0; k < 400 && t63 < 0; ++k) {
    plant.step(c.step(20.0, plant.omega, 0.001), 0.001);
    if (plant.omega >= 0.632 * 20.0) t63 = k + 1;
  }
  EXPECT_NEAR(t63, 50, 2) << "ms to 63 %";
}

// --- 3. Parity: C++ against the Python reference, tick by tick ----------------------------------------------
TEST(Parity, CppMatchesPythonReference) {
  double dt = 0.0;
  const std::vector<Tick> ticks = loadFixture(dt);
  ASSERT_FALSE(ticks.empty()) << "could not read " << FIXTURE;
  VelocityPI c;
  int scenario = -1;
  double d_duty = 0.0, d_integ = 0.0;
  for (const Tick& t : ticks) {
    if (t.scenario != scenario) {  // a fresh controller for each scenario
      c = VelocityPI();
      c.antiwindup = t.antiwindup != 0;
      scenario = t.scenario;
    }
    const double duty = c.step(t.ref, t.meas, dt, t.ramping != 0);
    d_duty = std::max(d_duty, std::fabs(duty - t.duty));
    d_integ = std::max(d_integ, std::fabs(c.integ - t.integ));
  }
  std::cout << "      C++ vs Python over " << ticks.size() << " ticks: max |d duty| " << d_duty << ", max |d integ| "
            << d_integ << "\n";
  EXPECT_LT(d_duty, 1e-12);
  EXPECT_LT(d_integ, 1e-12);
}

#ifdef FIRMWARE_CORE
// --- 4. Your port against the firmware's own VelocityPI (float), on the same ticks ---------------------------
TEST(Parity, CppMatchesTheFirmwareWithinFloatPrecision) {
  double dt = 0.0;
  const std::vector<Tick> ticks = loadFixture(dt);
  ASSERT_FALSE(ticks.empty());
  orth::Config cfg;  // the firmware's defaults: the same model and the conservative tier
  orth::VelocityPI fw;
  VelocityPI mine;
  double d_duty = 0.0;
  int scenario = -1;
  for (const Tick& t : ticks) {
    if (t.scenario != scenario) {
      fw.reset();
      mine = VelocityPI();
      mine.antiwindup = t.antiwindup != 0;
      cfg.ramp_antiwindup = t.antiwindup != 0;
      scenario = t.scenario;
    }
    if (!t.antiwindup) continue;  // the firmware has no switch for anti-windup part 1
    const float out = fw.step(static_cast<float>(t.ref), static_cast<float>(t.meas), static_cast<float>(dt), cfg,
                              t.ramping != 0);
    d_duty = std::max(d_duty, std::fabs(mine.step(t.ref, t.meas, dt, t.ramping != 0) - out));
  }
  std::cout << "      yours (double) vs firmware (float): max |d duty| " << d_duty << "\n";
  EXPECT_LT(d_duty, 1e-4);
}
#endif
