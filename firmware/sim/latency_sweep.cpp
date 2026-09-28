// Sweeps the latency-relevant settings with real EMG and prints the table.
//   latency_sweep <path to real_contractions.bin>
#include <cstdio>

#include "latency.h"

int main(int argc, char** argv) {
  const auto trials = sim::loadContractions(argc > 1 ? argv[1] : "sim/data/real_contractions.bin");
  std::printf("%zu real contractions\n", trials.size());
  auto show = [&](const char* name, const orth::Config& c) {
    const sim::LatencySummary s = sim::summarize(c, trials);
    std::printf("%-44s move %4.0f (p90 %4.0f)  half-speed %4.0f (p90 %4.0f)  stop %4.0f (p90 %4.0f) ms  "
                "jitter %.2f dps  n=%d\n", name, s.move_med, s.move_p90, s.half_med, s.half_p90, s.stop_med,
                s.stop_p90, s.jitter_med, s.n);
  };
  orth::Config before;
  before.envelope = orth::Config::Envelope::TwoPole;
  before.confirm_ms = 0;
  show("before: two-pole 4 Hz, accel 150, lambda 50", before);
  const orth::Config ident = sim::identifiedConfig(orth::Config{}, sim::PlantParams{});
  for (float accel : {150.0f, 300.0f, 600.0f, 1000.0f})
    for (float lambda : {0.05f, 0.03f, 0.02f}) {
      orth::Config c = ident;
      c.joint_accel_fast_dps2 = accel;
      c.lambda_fast_s = lambda;
      char name[64];
      std::snprintf(name, sizeof name, "bayes, accel %.0f, lambda %.0f ms", accel, 1e3 * lambda);
      show(name, c);
    }
  return 0;
}
