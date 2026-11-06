// `make latency`: the numbers behind the README's latency and tuning choices.
//   latency_sweep <path to real_contractions.bin>
// 1. Ramp x loop time constant, on the real contractions.
// 2. Fast-tier robustness: worst step overshoot over 36 perturbed motors.
// 3. Ablation: the acceleration feed-forward term and the ramp anti-windup.
// 4. Holding the forearm against a push.
// 5. How far the latency medians move with small changes to the plant.
#include <algorithm>
#include <cstdio>
#include <functional>
#include <vector>

#include "latency.h"

using orth::Config;
using sim::PlantParams;

namespace {

// The 36 variants of ClosedLoop.RobustToModelError: rotor inertia x0.3-3,
// 10.5-13.5 V, friction x0.5-2, limb mass and inertia x0.5-1.5.
std::vector<PlantParams> variants() {
  std::vector<PlantParams> v;
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
          v.push_back(pp);
        }
  return v;
}

sim::Harness armed(Config cfg, const PlantParams& pp, double start_deg) {
  cfg.joint_max_deg = 60.0f;
  sim::Harness h(cfg, pp);
  sim::calibrateZeroArm(h, start_deg);
  return h;
}

// 0 -> 20 deg/s from 10 deg with the forearm on, as in the robustness tests.
sim::StepStats step20(const Config& cfg, const PlantParams& pp) {
  sim::Harness h = armed(cfg, pp, 10.0);
  h.run(0.3);
  return sim::stepResponse(h, 20.0, 1.0);
}

// `tweak` edits the config; with `identify`, 'j' runs on each motor first.
double worstOvershoot(bool identify, const std::function<void(Config&)>& tweak) {
  double worst = 0;
  for (const PlantParams& pp : variants()) {
    Config base;
    tweak(base);
    const Config cfg = identify ? sim::identifiedConfig(base, pp) : base;
    worst = std::max(worst, step20(cfg, pp).overshoot);
  }
  return worst;
}

}  // namespace

int main(int argc, char** argv) {
  const auto trials = sim::loadContractions(argc > 1 ? argv[1] : "sim/data/real_contractions.bin");
  Config before;
  before.envelope = Config::Envelope::TwoPole;
  before.confirm_ms = 0;

  std::printf("1. Latency on %zu real contractions (ms; the shipped fast tier is accel 500, lambda 30)\n",
              trials.size());
  auto show = [&](const char* name, const Config& c) {
    const sim::LatencySummary s = sim::summarize(c, trials);
    std::printf("   %-44s move %4.0f (p90 %4.0f)  half-speed %4.0f (p90 %4.0f)  stop %4.0f (p90 %4.0f)  "
                "jitter %.2f deg/s  n=%d\n", name, s.move_med, s.move_p90, s.half_med, s.half_p90, s.stop_med,
                s.stop_p90, s.jitter_med, s.n);
  };
  show("before: two-pole 4 Hz, accel 150, lambda 50", before);
  const Config ident = sim::identifiedConfig(Config{}, PlantParams{});
  const sim::LatencySummary shipped = sim::summarize(ident, trials);
  std::printf("   shipped, median split: speed command after %.1f ms (%.0f of them the onset confirm), "
              "joint past 2 deg/s %.1f ms later\n", shipped.cmd_med, static_cast<double>(ident.confirm_ms),
              shipped.spinup_med);
  for (float accel : {150.0f, 300.0f, 500.0f, 1000.0f})
    for (float lambda : {0.05f, 0.03f, 0.02f}) {
      Config c = ident;
      c.joint_accel_fast_dps2 = accel;
      c.lambda_fast_s = lambda;
      char name[64];
      std::snprintf(name, sizeof name, "bayes, accel %.0f, lambda %.0f ms", accel, 1e3 * lambda);
      show(name, c);
    }

  std::printf("\n2. Worst overshoot of a 0 -> 20 deg/s step across the 36 perturbed motors\n");
  auto fast = [](float accel, float lambda) {
    return [=](Config& c) { c.joint_accel_fast_dps2 = accel; c.lambda_fast_s = lambda; };
  };
  std::printf("   %-52s %5.1f %%\n", "conservative tier, datasheet model",
              100 * worstOvershoot(false, [](Config&) {}));
  std::printf("   %-52s %5.1f %%\n", "fast tier forced on with the datasheet model",
              100 * worstOvershoot(false, [](Config& c) { c.model_identified = true; }));
  std::printf("   %-52s %5.1f %%\n", "fast tier after 'j': accel 500, lambda 30 (shipped)",
              100 * worstOvershoot(true, fast(500.0f, 0.03f)));
  std::printf("   %-52s %5.1f %%\n", "fast tier after 'j': accel 500, lambda 20",
              100 * worstOvershoot(true, fast(500.0f, 0.02f)));
  std::printf("   %-52s %5.1f %%\n", "fast tier after 'j': accel 1000, lambda 30",
              100 * worstOvershoot(true, fast(1000.0f, 0.03f)));

  std::printf("\n3. Ablation: 0 -> 20 deg/s step; nominal motor, then the worst of the 36\n");
  for (bool ff : {true, false})
    for (bool aw : {true, false}) {
      auto tweak = [=](Config& c) { c.ff_accel = ff; c.ramp_antiwindup = aw; };
      Config cons;
      tweak(cons);
      const sim::StepStats sc = step20(cons, PlantParams{});
      const sim::StepStats sf = step20(sim::identifiedConfig(cons, PlantParams{}), PlantParams{});
      std::printf("   accel feed-forward %-3s ramp anti-windup %-3s  nominal: conservative %5.1f %% (rise %3.0f ms), "
                  "fast %5.1f %% (rise %3.0f ms)  worst: conservative %5.1f %%, fast %5.1f %%\n",
                  ff ? "on" : "off", aw ? "on" : "off", 100 * sc.overshoot, 1e3 * sc.rise_s, 100 * sf.overshoot,
                  1e3 * sf.rise_s, 100 * worstOvershoot(false, tweak), 100 * worstOvershoot(true, tweak));
    }

  std::printf("\n4. Holding at 20 deg, then pushing the forearm down for 3 s\n");
  for (bool identified : {false, true})
    for (double push : {2.0, 4.0}) {
      sim::Harness h = armed(identified ? ident : Config{}, PlantParams{}, 20.0);
      h.run(1.0);
      const double p0 = h.plant.jointDeg();
      h.tau_ext_joint = -push;
      h.run(3.0);
      std::printf("   %-12s tier, %.0f N m: sags %.2f deg\n", identified ? "fast" : "conservative", push,
                  p0 - h.plant.jointDeg());
    }

  std::printf("\n5. Latency medians on slightly different plants (the firmware's config is unchanged)\n");
  struct Case { const char* name; std::function<void(PlantParams&)> edit; };
  const Case cases[] = {
      {"nominal", [](PlantParams&) {}},
      {"gearbox 5.18:1 (goBILDA's exact 1 + 46/11)", [](PlantParams& p) { p.gear = 57.0 / 11.0; }},
      {"pulley diameter +0.5 %", [](PlantParams& p) { p.pulley_r_m *= 1.005; }},
      {"rotor inertia +5 %", [](PlantParams& p) { p.J_rotor *= 1.05; }},
      {"supply 12.1 V", [](PlantParams& p) { p.vbus = 12.1; }},
  };
  for (const Case& c : cases) {
    PlantParams pp;
    c.edit(pp);
    const sim::LatencySummary b = sim::summarize(before, trials, pp);
    const sim::LatencySummary n = sim::summarize(sim::identifiedConfig(Config{}, pp), trials, pp);
    std::printf("   %-44s before %4.0f (p90 %4.0f)  now %4.0f (p90 %4.0f)\n", c.name, b.move_med, b.move_p90,
                n.move_med, n.move_p90);
  }
  return 0;
}
