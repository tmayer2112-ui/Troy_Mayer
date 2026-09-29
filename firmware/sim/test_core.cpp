// Unit tests for the building blocks in orthosis_core.h.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "../emg_orthosis/orthosis_core.h"
#include "plant.h"

using namespace orth;

TEST(Quadrature, DecodesBothDirectionsAndCountsIllegalJumps) {
  EdgeTracker e;
  e.reset(0);
  const uint8_t fwd[] = {2, 3, 1, 0, 2, 3, 1, 0};
  uint32_t t = 0;
  for (uint8_t ab : fwd) e.onChange(ab, t += 100);
  EXPECT_EQ(e.count, 8);
  EXPECT_EQ(e.dir, 1);
  const uint8_t back[] = {1, 3, 2, 0};
  for (uint8_t ab : back) e.onChange(ab, t += 100);
  EXPECT_EQ(e.count, 4);
  EXPECT_EQ(e.dir, -1);
  e.onChange(3, t += 100);  // 00 -> 11: both channels changed at once
  EXPECT_EQ(e.errors, 1u);
  EXPECT_EQ(e.count, 4);
}

// Drive a perfect-speed shaft through an imperfect encoder and compare the
// edge-timing estimate with a count-per-tick estimate at 100 Hz (the old firmware).
struct EstimatorRun { double edge_timing_err, count_diff_err; };

static EstimatorRun runEstimator(double joint_dps) {
  sim::PlantParams pp;
  const Config cfg;
  const double motor_counts_per_s = joint_dps / cfg.degPerCount();
  EdgeTracker e;
  e.reset(0);
  VelocityEstimator est;
  const uint8_t seq[4] = {0, 2, 3, 1};
  long c = 0;
  double se_new = 0, se_old = 0;
  int n = 0;
  int32_t prev_count_100hz = 0;
  double old_est = 0;
  for (int tick = 1; tick <= 3000; ++tick) {
    const double t = tick * 1e-3;
    const double x = motor_counts_per_s * t;
    auto edge_at = [&](long k) { return k + pp.edge_err[((k % 4) + 4) % 4]; };
    while (edge_at(c + 1) <= x) {
      ++c;
      const double te = edge_at(c) / motor_counts_per_s;
      e.onChange(seq[((c % 4) + 4) % 4], static_cast<uint32_t>(te * 1e6));
    }
    const double v = est.update(e, static_cast<uint32_t>(t * 1e6)) * cfg.degPerCount();
    if (tick % 10 == 0) {
      old_est = (e.count - prev_count_100hz) * cfg.degPerCount() / 0.01;
      prev_count_100hz = e.count;
    }
    if (tick > 500) {
      se_new += (v - joint_dps) * (v - joint_dps);
      se_old += (old_est - joint_dps) * (old_est - joint_dps);
      ++n;
    }
  }
  return {std::sqrt(se_new / n) / joint_dps, std::sqrt(se_old / n) / joint_dps};
}

TEST(VelocityEstimator, EdgeTimingBeatsCountDifference) {
  for (double dps : {3.0, 10.0, 25.0, 100.0}) {
    const EstimatorRun r = runEstimator(dps);
    // RMS error relative to true speed
    EXPECT_LT(r.edge_timing_err, 0.02) << dps << " deg/s";
    EXPECT_GT(r.count_diff_err, 3.0 * r.edge_timing_err) << dps << " deg/s";
    printf("  %6.1f deg/s: edge-timing RMS error %5.2f %%, 100 Hz count-difference %6.1f %%\n", dps,
           100 * r.edge_timing_err, 100 * r.count_diff_err);
  }
}

TEST(VelocityEstimator, DecaysToZeroWhenTheShaftStops) {
  EdgeTracker e;
  e.reset(0);
  VelocityEstimator est;
  const uint8_t seq[4] = {0, 2, 3, 1};
  uint32_t t = 0;
  for (int k = 1; k <= 20; ++k) e.onChange(seq[k % 4], t += 2000);  // 500 counts/s
  EXPECT_NEAR(est.update(e, t), 500.0f, 1.0f);
  EXPECT_LE(std::fabs(est.update(e, t + 50000)), 20.0f + 1e-3f);   // <= 1 count / 50 ms
  EXPECT_EQ(est.update(e, t + 300000), 0.0f);
}

TEST(Envelope, TwoPoleDelayMatchesTheDesign) {
  // Two equal one-pole stages at fc: the step response reaches 50 % at 1.678 tau.
  // A 150 Hz tone switched on at t = 0 is a step in rectified amplitude.
  Config c;
  c.envelope = Config::Envelope::TwoPole;
  auto run = [&](int n_samples, int* t50, float final_env) {
    EmgChannel ch;
    ch.design(c);
    ch.prime(2048);
    for (int k = 0; k < n_samples; ++k) {
      const float x = 2048.0f + 300.0f * std::sin(2.0f * kPi * 150.0f * k / 1000.0f);
      ch.sample(static_cast<uint16_t>(x), c);
      if (t50 && *t50 < 0 && ch.envelope() >= 0.5f * final_env) *t50 = k;
    }
    return ch.envelope();
  };
  const float final_env = run(3000, nullptr, 0.0f);
  int t50 = -1;
  run(3000, &t50, final_env);
  const float tau_ms = 1000.0f / (2.0f * kPi * c.envelope_hz);
  EXPECT_NEAR(t50, 1.678f * tau_ms, 4.0f);
  printf("  two-pole envelope: 50 %% of a step after %d ms (design %.0f ms), mean delay %.0f ms\n", t50,
         1.678f * tau_ms, 2.0f * tau_ms);
}

// Gaussian-noise "EMG" at a given RMS, as ADC counts about mid-scale.
static uint16_t noiseSample(std::mt19937& g, double rms) {
  std::normal_distribution<double> n(0.0, rms);
  return static_cast<uint16_t>(std::lround(std::clamp(2048.0 + n(g), 0.0, 4095.0)));
}

TEST(Envelope, BayesTracksAStepFastAndHoldsSteady) {
  Config c;   // Bayes is the default
  EmgChannel ch;
  ch.design(c);
  ch.prime(2048);
  std::mt19937 g(1);
  for (int k = 0; k < 2000; ++k) ch.sample(noiseSample(g, 4.0), c);
  const float rest = ch.envelope();
  int t50 = -1;
  double sum = 0, sum2 = 0;
  int n = 0;
  for (int k = 0; k < 3000; ++k) {
    ch.sample(noiseSample(g, 200.0), c);
    if (t50 < 0 && ch.envelope() >= 100.0f) t50 = k;
    if (k >= 1000) { sum += ch.envelope(); sum2 += ch.envelope() * ch.envelope(); ++n; }
  }
  const double mean = sum / n, cv = std::sqrt(sum2 / n - mean * mean) / mean;
  printf("  Bayes envelope: rest %.1f, 50 %% of a 4 -> 200 RMS step after %d ms, steady %.0f (CV %.3f)\n",
         rest, t50, mean, cv);
  EXPECT_LT(rest, 10.0f);
  EXPECT_LE(t50, 10);                     // the two-pole filter at 4 Hz takes 67 ms
  EXPECT_NEAR(mean, 200.0, 30.0);         // the MAP level is the RMS, to within a bin or two
  EXPECT_LT(cv, 0.10);
}

TEST(Activation, ConfirmWindowRejectsASpike) {
  Config c;
  EmgChannel ch;
  ch.design(c);
  ch.prime(2048);
  std::mt19937 g(2);
  for (int k = 0; k < 3000; ++k) ch.sample(noiseSample(g, 4.0), c);
  ch.rest_mean = ch.envelope();
  ch.rest_sd = 1.0f;
  ch.mvc = 400.0f;
  ch.setThresholds(c);
  bool fired = false;
  // a 3 ms, 400-count biphasic electrode pop
  const int pop[3] = {400, -400, 200};
  for (int k = 0; k < 200; ++k) {
    const uint16_t raw = k < 3 ? static_cast<uint16_t>(2048 + pop[k]) : noiseSample(g, 4.0);
    ch.sample(raw, c);
    ch.updateActivation(c);
    fired |= ch.active;
  }
  EXPECT_FALSE(fired) << "a single artefact must not start the motor";
  // a real contraction does get through, one confirm window later
  int on_at = -1;
  for (int k = 0; k < 200 && on_at < 0; ++k) {
    ch.sample(noiseSample(g, 150.0), c);
    ch.updateActivation(c);
    if (ch.active) on_at = k;
  }
  EXPECT_GT(on_at, static_cast<int>(c.confirm_ms) - 1);
  EXPECT_LT(on_at, static_cast<int>(c.confirm_ms) + 15);
}

TEST(Activation, HysteresisAndScaling) {
  Config c;
  c.confirm_ms = 0;
  EmgChannel ch;
  ch.use_bayes = false;
  ch.rest_mean = 10.0f;
  ch.rest_sd = 2.0f;
  ch.mvc = 400.0f;
  ch.setThresholds(c);
  EXPECT_FLOAT_EQ(ch.onset, std::max(10.0f + 12.0f, 10.0f + 0.08f * 390.0f));
  auto at = [&](float env) { ch.e2.y = env; ch.updateActivation(c); return ch.activation; };
  EXPECT_EQ(at(ch.onset - 1), 0.0f);                 // below onset
  EXPECT_GT(at(ch.onset + 50), 0.0f);                 // on
  EXPECT_TRUE(ch.active);
  EXPECT_EQ(at(ch.onset - 1), 0.0f);                 // still active but at 0 ...
  EXPECT_TRUE(ch.active);                            // ... because release is lower
  at(ch.release - 1);
  EXPECT_FALSE(ch.active);
  EXPECT_FLOAT_EQ(at(0.6f * ch.mvc + 100), 1.0f);    // saturates at 60 % MVC
}

TEST(Intent, DeadzoneCocontractionAndDirection) {
  Config c;
  EXPECT_EQ(intentToSpeed(0.05f, 0.0f, c), 0.0f);
  EXPECT_GT(intentToSpeed(0.6f, 0.0f, c), 0.0f);
  EXPECT_LT(intentToSpeed(0.0f, 0.6f, c), 0.0f);
  EXPECT_EQ(intentToSpeed(0.9f, 0.5f, c), 0.0f);            // both above 0.35: stop
  EXPECT_FLOAT_EQ(intentToSpeed(1.0f, 0.0f, c), c.joint_max_dps);
}

TEST(SetpointShaper, BrakingCurveNeverOvershootsTheLimit) {
  Config c;
  SetpointShaper sh;
  // Kinematic check: follow the shaped setpoint exactly from mid-range at full command.
  float pos = 10.0f, max_pos = 0.0f;
  for (int k = 0; k < 5000; ++k) {
    const float v = sh.step(c.joint_max_dps, pos, 1e-3f, c);
    pos += v * 1e-3f;
    max_pos = std::max(max_pos, pos);
  }
  EXPECT_LE(max_pos, c.joint_max_deg - c.limit_margin_deg + 0.01f);
  EXPECT_GE(max_pos, c.joint_max_deg - c.limit_margin_deg - 0.05f);
}

TEST(VelocityPI, AntiWindupStopsIntegratingIntoSaturation) {
  Config c;
  VelocityPI pi;
  for (int k = 0; k < 5000; ++k) pi.step(25.0f, 0.0f, 1e-3f, c);   // stalled, saturated
  EXPECT_FLOAT_EQ(pi.last(), c.duty_max);
  EXPECT_LE(pi.integrator(), c.duty_max);
  // The moment the error reverses, the output leaves saturation immediately.
  const float d = pi.step(0.0f, 25.0f, 1e-3f, c);
  EXPECT_LT(d, c.duty_max);
}

TEST(Config, TransmissionArithmetic) {
  Config c;
  // 4 mm per screw rev over a 47 mm pulley = 9.75 joint deg per output rev,
  // over 28 * 5.2 counts = 0.0670 deg per count.
  EXPECT_NEAR(c.degPerCount(), 0.06698, 1e-4);
  EXPECT_NEAR(c.kp(), 0.020 / (187.0 * 0.050), 1e-6);
  EXPECT_NEAR(c.ki(), 1.0 / (187.0 * 0.050), 1e-6);
}
