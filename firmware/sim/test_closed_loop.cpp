// Closed-loop tests: the real Controller driving the motor + transmission + limb model.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "harness.h"

using namespace sim;
using orth::Config;
using orth::Request;
using orth::State;

namespace {

struct StepStats { double rise_s, overshoot, ss_err, ss_ripple; };

// Command `dps` from standstill and measure the joint's true speed.
StepStats stepResponse(Harness& h, double dps, double seconds = 0.6) {
  h.log.clear();
  h.record = true;
  h.ctl.setSpeedOverride(true, static_cast<float>(dps));
  h.run(seconds);
  h.record = false;
  double t10 = -1, t90 = -1, peak = 0, sum = 0, sum2 = 0;
  int n = 0;
  const double t0 = h.log.front().t;
  for (const Sample& s : h.log) {
    const double v = s.vel * (dps > 0 ? 1 : -1);
    const double target = std::fabs(dps);
    if (t10 < 0 && v >= 0.1 * target) t10 = s.t;
    if (t90 < 0 && v >= 0.9 * target) t90 = s.t;
    peak = std::max(peak, v);
    if (s.t - t0 > seconds - 0.2) { sum += v; sum2 += v * v; ++n; }
  }
  const double mean = sum / n;
  return {t90 - t10, (peak - std::fabs(dps)) / std::fabs(dps), std::fabs(dps) - mean,
          std::sqrt(std::max(0.0, sum2 / n - mean * mean))};
}

Harness armedAt(double start_deg, Config cfg = Config{}, PlantParams pp = PlantParams{}) {
  cfg.joint_max_deg = 60.0f;   // room to run in these tests
  Harness h(cfg, pp);
  calibrateZeroArm(h, start_deg);
  return h;
}

}  // namespace

TEST(ClosedLoop, ArmsAfterTheBenchProcedure) {
  Harness h = armedAt(15.0);
  EXPECT_EQ(h.ctl.state(), State::Armed) << h.ctl.lastMessage();
}

TEST(ClosedLoop, StepResponseUnderLimbWeight) {
  Harness h = armedAt(10.0);
  h.run(0.5);  // settle the hold
  const StepStats s = stepResponse(h, 20.0);
  printf("  step 0 -> 20 deg/s with the limb: rise %.0f ms, overshoot %.1f %%, "
         "steady error %.2f deg/s, ripple %.2f deg/s RMS\n",
         1e3 * s.rise_s, 100 * s.overshoot, s.ss_err, s.ss_ripple);
  EXPECT_EQ(h.ctl.state(), State::Armed) << h.ctl.faultReason();
  EXPECT_LT(s.rise_s, 0.20);
  EXPECT_LT(s.overshoot, 0.10);
  EXPECT_LT(std::fabs(s.ss_err), 0.5);
  EXPECT_LT(s.ss_ripple, 1.0);
}

TEST(ClosedLoop, FeedbackRemovesTheLoadDroopThatOpenLoopHas) {
  // Same speed command, lifting and lowering the limb, with and without feedback.
  for (bool closed : {false, true}) {
    Config cfg;
    cfg.closed_loop = closed;
    for (double dps : {15.0, -15.0}) {
      Harness h = armedAt(dps > 0 ? 5.0 : 50.0, cfg);
      h.run(0.3);
      const StepStats s = stepResponse(h, dps, 1.5);
      printf("  %-11s %+5.0f deg/s: steady speed error %+6.2f deg/s (%5.1f %%)\n",
             closed ? "closed-loop" : "feed-fwd", dps, -s.ss_err * (dps > 0 ? 1 : -1),
             100 * std::fabs(s.ss_err) / std::fabs(dps));
      if (closed) EXPECT_LT(std::fabs(s.ss_err), 0.5);
      else EXPECT_GT(std::fabs(s.ss_err), 1.5);   // the droop feedback exists to remove
    }
  }
}

TEST(ClosedLoop, HoldsPositionAgainstGravityAtZeroCommand) {
  Harness h = armedAt(40.0);
  h.run(0.5);
  const double p0 = h.plant.jointDeg();
  h.run(5.0);
  const double drift = h.plant.jointDeg() - p0;
  printf("  hold at zero command, 5 s against the limb: drift %.3f deg\n", drift);
  EXPECT_LT(std::fabs(drift), 0.5);
  EXPECT_EQ(h.ctl.state(), State::Armed);
}

TEST(ClosedLoop, RejectsAnExternalTorqueStep) {
  Harness h = armedAt(20.0);
  h.ctl.setSpeedOverride(true, 10.0f);
  h.run(0.8);
  h.log.clear();
  h.record = true;
  h.tau_ext_joint = -2.0;   // someone pushes the forearm down with 2 N m
  h.run(1.0);
  double worst = 0;
  for (const Sample& s : h.log) worst = std::max(worst, std::fabs(s.vel - 10.0));
  const double final_err = std::fabs(h.log.back().vel - 10.0);
  printf("  2 N m disturbance at 10 deg/s: worst dip %.1f deg/s, recovered to %.2f deg/s\n", worst, final_err);
  EXPECT_LT(final_err, 0.5);
  EXPECT_EQ(h.ctl.state(), State::Armed) << h.ctl.faultReason();
}

TEST(ClosedLoop, RobustToModelError) {
  // The gains assume nominal numbers. Vary the plant, keep the gains.
  int cases = 0;
  double worst_os = 0;
  for (double jmul : {0.3, 1.0, 3.0})
    for (double vbus : {10.5, 12.0, 13.5})
      for (double fmul : {0.5, 2.0})
        for (double mmul : {0.5, 1.5}) {
          PlantParams pp;
          pp.J_rotor *= jmul;
          pp.vbus = vbus;
          pp.tau_coulomb *= fmul;
          pp.limb_mgr *= mmul;
          pp.limb_I *= mmul;
          Harness h = armedAt(10.0, Config{}, pp);
          h.run(0.3);
          const StepStats s = stepResponse(h, 20.0, 1.0);
          worst_os = std::max(worst_os, s.overshoot);
          EXPECT_EQ(h.ctl.state(), State::Armed) << "J x" << jmul << " V " << vbus << " f x" << fmul;
          EXPECT_LT(s.overshoot, 0.25) << "J x" << jmul << " V " << vbus << " f x" << fmul;
          EXPECT_LT(std::fabs(s.ss_err), 0.5);
          EXPECT_LT(s.ss_ripple, 1.5);
          ++cases;
        }
  printf("  %d plant variants (inertia x0.3-3, 10.5-13.5 V, friction x0.5-2, limb x0.5-1.5): "
         "worst overshoot %.1f %%\n", cases, 100 * worst_os);
}

TEST(ClosedLoop, FastTierIsRobustOnceTheMotorIsIdentified) {
  // Same 36 variants, but each one is identified with 'j' first (limb out),
  // which unlocks the fast tier: 600 deg/s^2 ramp, lambda 30 ms.
  int cases = 0;
  double worst_os = 0, worst_rise = 0;
  for (double jmul : {0.3, 1.0, 3.0})
    for (double vbus : {10.5, 12.0, 13.5})
      for (double fmul : {0.5, 2.0})
        for (double mmul : {0.5, 1.5}) {
          PlantParams pp;
          pp.J_rotor *= jmul;
          pp.vbus = vbus;
          pp.tau_coulomb *= fmul;
          pp.limb_mgr *= mmul;
          pp.limb_I *= mmul;
          const Config cfg = identifiedConfig(Config{}, pp);
          ASSERT_TRUE(cfg.model_identified) << "J x" << jmul << " V " << vbus << " f x" << fmul;
          Harness h = armedAt(10.0, cfg, pp);
          h.run(0.3);
          const StepStats s = stepResponse(h, 20.0, 1.0);
          worst_os = std::max(worst_os, s.overshoot);
          worst_rise = std::max(worst_rise, s.rise_s);
          EXPECT_EQ(h.ctl.state(), State::Armed) << "J x" << jmul << " V " << vbus << " f x" << fmul;
          EXPECT_LT(s.overshoot, 0.25) << "J x" << jmul << " V " << vbus << " f x" << fmul << " m x" << mmul;
          EXPECT_LT(std::fabs(s.ss_err), 0.5);
          EXPECT_LT(s.ss_ripple, 1.5);
          ++cases;
        }
  printf("  %d identified variants, fast tier: worst overshoot %.1f %%, worst rise %.0f ms\n", cases,
         100 * worst_os, 1e3 * worst_rise);
}

TEST(ClosedLoop, BrakesSmoothlyIntoTheSoftLimit) {
  Config cfg;
  cfg.joint_max_deg = 30.0f;
  Harness h(cfg, PlantParams{});
  calibrateZeroArm(h, 15.0);
  h.ctl.setSpeedOverride(true, 25.0f);
  double peak = 0;
  for (int k = 0; k < 3000; ++k) { h.run(0.001); peak = std::max(peak, h.ctl.jointDeg() + 0.0); }
  printf("  full speed into the 30 deg limit: stopped at %.2f deg (stop point %.1f)\n", peak,
         cfg.joint_max_deg - cfg.limit_margin_deg);
  EXPECT_LE(peak, cfg.joint_max_deg);
  EXPECT_GE(peak, cfg.joint_max_deg - 2.0);
  EXPECT_EQ(h.ctl.state(), State::Armed) << h.ctl.faultReason();
}

TEST(Safety, StallFaultsAndReleases) {
  PlantParams pp;
  pp.hard_stop = true;
  pp.hard_stop_deg = 25.0;   // an obstruction inside the soft limits (joint reads 15 at arm)
  Config cfg;
  cfg.joint_max_deg = 60.0f;
  Harness h(cfg, pp);
  calibrateZeroArm(h, 15.0);
  h.ctl.setSpeedOverride(true, 20.0f);
  double t_contact = -1, t_fault = -1;
  for (int k = 0; k < 3000 && t_fault < 0; ++k) {
    h.run(0.001);
    if (t_contact < 0 && h.plant.jointDeg() >= pp.hard_stop_deg - 1e-6) t_contact = h.t();
    if (h.ctl.state() == State::Fault) t_fault = h.t();
  }
  ASSERT_GT(t_fault, 0);
  printf("  obstruction: fault '%s' %.0f ms after contact\n", h.ctl.faultReason(), 1e3 * (t_fault - t_contact));
  EXPECT_EQ(std::string(h.ctl.faultReason()).rfind("stall", 0), 0u);
  EXPECT_LT(t_fault - t_contact, 0.7);
  EXPECT_FALSE(h.out().enable);   // coasting
}

TEST(Safety, ReversedEncoderIsCaughtBeforeItRunsAway) {
  PlantParams pp;
  pp.encoder_reversed = true;
  Config cfg;
  cfg.joint_max_deg = 60.0f;
  Harness h(cfg, pp);
  calibrateZeroArm(h, 20.0);   // hand-move goes "down" as far as the encoder can tell
  // The joint reads -20 deg, outside the limits, so arming is refused ...
  EXPECT_NE(h.ctl.state(), State::Armed);
  // ... so do what a confused user would: zero here and arm.
  h.run(0.01, Request::Zero);
  handMove(h, h.plant.jointDeg() - 15.0);   // reads +15
  h.run(0.2);
  h.run(0.01, Request::Arm);
  ASSERT_EQ(h.ctl.state(), State::Armed) << h.ctl.lastMessage();
  const double p0 = h.plant.jointDeg();
  h.ctl.setSpeedOverride(true, 10.0f);
  double t_fault = -1;
  const double t0 = h.t();
  for (int k = 0; k < 2000 && t_fault < 0; ++k) {
    h.run(0.001);
    if (h.ctl.state() == State::Fault) t_fault = h.t();
  }
  ASSERT_GT(t_fault, 0);
  const double travel = std::fabs(h.plant.jointDeg() - p0);
  printf("  reversed encoder: fault '%s' after %.0f ms, %.1f deg of travel\n", h.ctl.faultReason(),
         1e3 * (t_fault - t0), travel);
  EXPECT_LT(t_fault - t0, 0.4);
  EXPECT_LT(travel, 6.0);
}

TEST(Safety, KillSwitchDriverFaultAndLeadOff) {
  {
    Harness h = armedAt(15.0);
    h.ctl.setSpeedOverride(true, 10.0f);
    h.run(0.3);
    h.kill_closed = false;
    h.run(0.001);
    EXPECT_FALSE(h.out().enable);
    EXPECT_EQ(h.ctl.state(), State::Idle);
  }
  {
    Harness h = armedAt(15.0);
    h.mot_ok = false;
    h.run(0.001);
    EXPECT_EQ(h.ctl.state(), State::Fault);
    EXPECT_FALSE(h.out().enable);
  }
  {
    Harness h = armedAt(15.0);
    h.railed[0] = true;   // flexor electrode falls off, channel rails
    h.run(0.25);
    EXPECT_EQ(h.ctl.state(), State::Fault) << h.ctl.lastMessage();
    EXPECT_EQ(std::string(h.ctl.faultReason()).rfind("EMG lead off", 0), 0u);
  }
}

TEST(Safety, ArmIsRefusedUntilTheBenchProcedureIsDone) {
  Harness h(Config{}, PlantParams{});
  h.run(0.01, Request::Arm);
  EXPECT_NE(h.ctl.state(), State::Armed);   // still calibrating
  h.calibrate();
  h.run(0.01, Request::Arm);
  EXPECT_EQ(h.ctl.state(), State::Idle);
  EXPECT_STREQ(h.ctl.lastMessage(), "joint not zeroed ('z' at the lower limit)");
  h.run(0.01, Request::Zero);
  h.kill_closed = false;
  h.run(0.01, Request::Arm);
  EXPECT_STREQ(h.ctl.lastMessage(), "kill switch is open");
}

TEST(EndToEnd, EmgDrivesTheJoint) {
  Config cfg;
  cfg.joint_max_deg = 60.0f;
  Harness h(cfg, PlantParams{});
  h.calibrate();
  h.run(0.01, Request::Zero);
  handMove(h, 20.0);
  h.run(0.3);
  h.run(0.01, Request::Arm);
  ASSERT_EQ(h.ctl.state(), State::Armed) << h.ctl.lastMessage();

  const double t0 = h.t();
  const double rest = h.rest_amp, mvc = h.mvc_amp;
  // 0-1 s rest, 1-3 s flexor at ~45 % MVC, 3-4 s rest, 4-5 s co-contraction,
  // 5-6 s rest, 6-8 s extensor, 8-10 s rest.
  h.emg = [=](double t, double a[4]) {
    const double s = t - t0;
    for (int i = 0; i < 4; ++i) a[i] = rest;
    if (s > 1 && s < 3) a[0] = 0.45 * mvc;
    if (s > 4 && s < 5) { a[0] = 0.6 * mvc; a[1] = 0.6 * mvc; }
    if (s > 6 && s < 8) a[1] = 0.45 * mvc;
  };
  auto posAt = [&](double s) { h.run(s - (h.t() - t0)); return h.plant.jointDeg(); };
  const double p1 = posAt(1.0), p3 = posAt(3.0), p4 = posAt(4.0), p5 = posAt(5.0);
  const double p6 = posAt(6.0), p8 = posAt(8.0), p10 = posAt(10.0);
  printf("  EMG end to end: flex %+.1f deg, co-contraction %+.2f deg, extend %+.1f deg, relaxed hold %+.2f deg\n",
         p3 - p1, p5 - p4, p8 - p6, p10 - p8);
  EXPECT_EQ(h.ctl.state(), State::Armed) << h.ctl.faultReason();
  EXPECT_GT(p3 - p1, 5.0);            // flexor moves it up
  EXPECT_LT(std::fabs(p5 - p4), 1.0); // co-contraction holds still
  EXPECT_LT(p8 - p6, -5.0);           // extensor moves it down
  EXPECT_LT(std::fabs(p10 - p8), 1.0);
}

TEST(Identify, RecoversTheMotorModelAndTheEncoderSign) {
  for (bool reversed : {false, true}) {
    PlantParams pp;
    pp.encoder_reversed = reversed;
    pp.limb_mgr = 0.0;   // identify runs with the limb out of the orthosis
    Config cfg;
    cfg.joint_max_deg = 60.0f;
    Harness h(cfg, pp);
    h.calibrate();
    h.run(0.01, Request::Zero);
    handMove(h, reversed ? -30.0 : 30.0);   // reads +30 on the controller either way
    h.run(0.2);
    h.run(0.01, Request::Identify);
    h.run(3.0);
    const orth::IdentResult& r = h.ctl.ident();
    ASSERT_TRUE(r.done) << h.ctl.lastMessage();
    if (reversed) {
      EXPECT_FALSE(r.sign_ok);
      EXPECT_STREQ(r.note, "encoder sign is reversed: set encoder_sign = -1");
      continue;
    }
    // Truth from the plant: 12 V / Ke, through the transmission, in joint deg/s per unit duty.
    const double true_k = pp.vbus / pp.Ke / pp.N() * 180.0 / kPi;
    const double true_tau = pp.J() * pp.R / (pp.Ke * pp.Ke);
    printf("  identify: K %.0f deg/s/duty (true %.0f), friction duty %.3f, tau %.1f ms (true %.1f)\n",
           r.dps_per_duty, true_k, r.friction_duty, 1e3 * r.tau_s, 1e3 * true_tau);
    EXPECT_TRUE(r.ok) << r.note;
    EXPECT_NEAR(r.dps_per_duty, true_k, 0.15 * true_k);
    EXPECT_NEAR(r.tau_s, true_tau, 0.6 * true_tau);
  }
}
