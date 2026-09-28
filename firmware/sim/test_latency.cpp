// End-to-end latency with real EMG (see latency.h): pins the numbers the README
// and the resume quote, so a change that slows the arm down fails CI.
#include <gtest/gtest.h>

#include "latency.h"

#ifndef ORTH_DATA_DIR
#define ORTH_DATA_DIR "sim/data"
#endif

namespace {
const std::vector<std::vector<int16_t>>& trials() {
  static const auto t = sim::loadContractions(ORTH_DATA_DIR "/real_contractions.bin");
  return t;
}
}  // namespace

TEST(Latency, RealEmgOnsetToMotion) {
  ASSERT_GE(trials().size(), 20u) << "missing sim/data/real_contractions.bin";
  // "before": this morning's firmware - two-pole 4 Hz envelope, no confirm,
  // 150 deg/s^2 ramp, lambda 50 ms. "now": Bayesian envelope with the fast
  // tier unlocked by identifying the (simulated) motor with 'j'.
  orth::Config before;
  before.envelope = orth::Config::Envelope::TwoPole;
  before.confirm_ms = 0;
  const orth::Config now = sim::identifiedConfig(orth::Config{}, sim::PlantParams{});
  ASSERT_TRUE(now.model_identified);
  const sim::LatencySummary b = sim::summarize(before, trials());
  const sim::LatencySummary n = sim::summarize(now, trials());
  printf("  %d real contractions, EMG onset -> joint:\n", n.n);
  printf("    before: moving %3.0f ms (p90 %3.0f), half speed %3.0f ms (p90 %3.0f), stops %3.0f ms after "
         "(p90 %3.0f), jitter %.2f deg/s\n", b.move_med, b.move_p90, b.half_med, b.half_p90, b.stop_med,
         b.stop_p90, b.jitter_med);
  printf("    now:    moving %3.0f ms (p90 %3.0f), half speed %3.0f ms (p90 %3.0f), stops %3.0f ms after "
         "(p90 %3.0f), jitter %.2f deg/s\n", n.move_med, n.move_p90, n.half_med, n.half_p90, n.stop_med,
         n.stop_p90, n.jitter_med);
  EXPECT_EQ(n.n, static_cast<int>(trials().size()));   // every trial armed and ran without a fault
  EXPECT_LT(n.move_med, 60.0);
  EXPECT_LT(n.half_med, 85.0);
  EXPECT_LT(n.stop_p90, 50.0);
  EXPECT_LT(n.move_med, b.move_med);
  EXPECT_LT(n.stop_med, b.stop_med);
  EXPECT_LE(n.jitter_med, b.jitter_med);
}
