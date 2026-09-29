// Writes the traces behind the README figures and results/metrics.json.
//   sim_demo <out_dir>
#include <cmath>
#include <cstdio>
#include <string>

#include "harness.h"

using namespace sim;
using orth::Config;
using orth::Request;

static Harness armed(double start_deg, Config cfg) {
  cfg.joint_max_deg = 60.0f;
  Harness h(cfg, PlantParams{});
  calibrateZeroArm(h, start_deg);
  h.run(0.3);
  return h;
}

int main(int argc, char** argv) {
  const std::string out = argc > 1 ? argv[1] : "results";
  FILE* m = std::fopen((out + "/metrics.json").c_str(), "w");
  std::fprintf(m, "{\n");

  // 1. Lifting the limb at 15 deg/s: feed-forward only vs closed loop.
  {
    FILE* f = std::fopen((out + "/step_load.csv").c_str(), "w");
    std::fprintf(f, "t,ref,vel_ff_only,vel_closed,duty_ff_only,duty_closed\n");
    Config ff;
    ff.closed_loop = false;
    Harness a = armed(5.0, ff), b = armed(5.0, Config{});
    a.record = b.record = true;
    a.ctl.setSpeedOverride(true, 15.0f);
    b.ctl.setSpeedOverride(true, 15.0f);
    a.run(1.5);
    b.run(1.5);
    double ea = 0, eb = 0;
    int n = 0;
    for (size_t k = 0; k < a.log.size(); ++k) {
      const Sample &x = a.log[k], &y = b.log[k];
      std::fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.4f,%.4f\n", x.t - a.log[0].t, y.ref, x.vel, y.vel, x.duty, y.duty);
      if (k > a.log.size() - 500) { ea += 15.0 - x.vel; eb += 15.0 - y.vel; ++n; }
    }
    std::fclose(f);
    std::fprintf(m, "  \"lift_15dps_steady_error_ff_only_dps\": %.3f,\n", ea / n);
    std::fprintf(m, "  \"lift_15dps_steady_error_closed_dps\": %.3f,\n", eb / n);
  }

  // 2. Step 0 -> 20 deg/s, closed loop, with the limb.
  {
    Harness h = armed(10.0, Config{});
    h.record = true;
    h.ctl.setSpeedOverride(true, 20.0f);
    h.run(0.6);
    double t10 = -1, t90 = -1, peak = 0;
    for (const Sample& s : h.log) {
      if (t10 < 0 && s.vel >= 2.0) t10 = s.t;
      if (t90 < 0 && s.vel >= 18.0) t90 = s.t;
      peak = std::fmax(peak, s.vel);
    }
    std::fprintf(m, "  \"step20_rise_10_90_ms\": %.1f,\n", 1e3 * (t90 - t10));
    std::fprintf(m, "  \"step20_overshoot_pct\": %.2f,\n", 100.0 * (peak - 20.0) / 20.0);
  }

  // 3. Hold at zero command against the limb for 5 s.
  {
    Harness h = armed(40.0, Config{});
    const double p0 = h.plant.jointDeg();
    h.run(5.0);
    std::fprintf(m, "  \"hold_drift_5s_deg\": %.4f,\n", h.plant.jointDeg() - p0);
  }

  // 4. EMG end to end: flex, co-contract, extend, relax.
  {
    Config cfg;
    cfg.joint_max_deg = 60.0f;
    Harness h(cfg, PlantParams{});
    h.calibrate();
    h.run(0.01, Request::Zero);
    handMove(h, 20.0);
    h.run(0.3);
    h.run(0.01, Request::Arm);
    const double t0 = h.t(), rest = h.rest_amp, mvc = h.mvc_amp;
    h.emg = [=](double t, double a[4]) {
      const double s = t - t0;
      for (int i = 0; i < 4; ++i) a[i] = rest;
      if (s > 1 && s < 3) a[0] = 0.25 * mvc + 0.25 * mvc * std::fmin(1.0, s - 1);   // ramping effort
      if (s > 4 && s < 5) { a[0] = 0.6 * mvc; a[1] = 0.6 * mvc; }
      if (s > 6 && s < 8) a[1] = 0.45 * mvc;
    };
    FILE* f = std::fopen((out + "/emg_end_to_end.csv").c_str(), "w");
    std::fprintf(f, "t,env_flex,env_ext,act_flex,act_ext,cmd,ref,vel,pos\n");
    for (int k = 0; k < 10000; ++k) {
      h.run(0.001);
      if (k % 5 == 0)
        std::fprintf(f, "%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", h.t() - t0,
                     h.ctl.channel(0).envelope(), h.ctl.channel(1).envelope(), h.ctl.channel(0).activation,
                     h.ctl.channel(1).activation, h.ctl.speedCommand(), h.ctl.speedRef(), h.plant.jointDps(),
                     h.plant.jointDeg());
    }
    std::fclose(f);
    std::fprintf(m, "  \"emg_demo_final_state\": \"%s\",\n", orth::stateName(h.ctl.state()));
  }

  // 5. Speed estimate on a slow ramp: edge timing vs 100 Hz count difference.
  {
    FILE* f = std::fopen((out + "/estimator.csv").c_str(), "w");
    std::fprintf(f, "t,true,edge_timing,count_diff_100hz\n");
    Harness h = armed(5.0, Config{});
    int32_t prev = 0;
    double cd = 0;
    for (int k = 0; k < 1500; ++k) {
      h.ctl.setSpeedOverride(true, static_cast<float>(std::fmin(30.0, k * 0.03)));
      h.run(0.001);
      const double pos_counts = h.ctl.jointDeg() / h.ctl.config().degPerCount();
      if (k % 10 == 0) {
        const int32_t c = static_cast<int32_t>(std::lround(pos_counts));
        cd = (c - prev) * h.ctl.config().degPerCount() / 0.01;
        prev = c;
      }
      std::fprintf(f, "%.3f,%.3f,%.3f,%.3f\n", k * 1e-3, h.plant.jointDps(), h.ctl.jointDps(), k < 10 ? 0.0 : cd);
    }
    std::fclose(f);
  }

  const Config c;
  std::fprintf(m, "  \"envelope\": \"%s\",\n  \"onset_confirm_ms\": %u,\n",
               c.envelope == Config::Envelope::Bayes ? "bayes" : "two-pole", static_cast<unsigned>(c.confirm_ms));
  std::fprintf(m, "  \"kp_duty_per_dps\": %.5f,\n  \"ki_duty_per_deg\": %.4f,\n", c.kp(), c.ki());
  std::fprintf(m, "  \"deg_per_count\": %.5f\n}\n", c.degPerCount());
  std::fclose(m);
  std::printf("wrote %s/{step_load,emg_end_to_end,estimator}.csv and metrics.json\n", out.c_str());
  return 0;
}
