// Module 9 timing (provided): how much of the 1 ms tick does the control code use?
// It times your PI step, and, when the firmware header is found, the firmware's own float PI and its
// Bayesian envelope (4 channels, as every tick runs). This runs on YOUR computer, not the ESP32-S3.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

#include "velocity_pi.hpp"

#ifdef FIRMWARE_CORE
#include FIRMWARE_CORE
#endif

using Clock = std::chrono::steady_clock;

namespace {

// Mean and 99th-percentile ns per call, from batches of `batch` calls.
template <class F>
void timeIt(const char* name, F&& f, int batches = 2000, int batch = 100) {
  std::vector<double> per_call(batches);
  for (int b = 0; b < batches; ++b) {
    const auto t0 = Clock::now();
    for (int k = 0; k < batch; ++k) f(k);
    per_call[b] = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / batch;
  }
  std::vector<double> sorted = per_call;
  std::sort(sorted.begin(), sorted.end());
  double mean = 0;
  for (double v : per_call) mean += v / batches;
  std::printf("  %-44s mean %8.1f ns   p99 of batches %8.1f ns   (%.4f %% of the 1 ms tick)\n", name, mean,
              sorted[static_cast<size_t>(0.99 * (batches - 1))], 100 * mean / 1e6);
}

}  // namespace

int main() {
  volatile double sink = 0;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 1.0);
  std::vector<double> meas(256);
  for (double& m : meas) m = 15.0 + noise(rng);

  std::printf("Per call, on this computer:\n");
  rebuild::VelocityPI pi;
  timeIt("your PI step (double)", [&](int k) { sink = sink + pi.step(15.0, meas[k & 255], 0.001); });
#ifdef FIRMWARE_CORE
  orth::Config cfg;
  orth::VelocityPI fw;
  timeIt("firmware PI step (float)",
         [&](int k) { sink = sink + fw.step(15.0f, static_cast<float>(meas[k & 255]), 0.001f, cfg); });
  orth::BayesEnvelope env[4];
  for (auto& e : env) e.design(cfg.bayes_alpha, cfg.bayes_beta);
  std::vector<float> emg(256);
  for (float& v : emg) v = static_cast<float>(200.0 * noise(rng));
  timeIt("firmware Bayesian envelope, 4 channels", [&](int k) {
    for (auto& e : env) sink = sink + e.step(emg[k & 255]);
  }, 500, 20);
  std::printf("The envelope's 64 exp() per channel per sample are the expensive part, not the PI. The firmware\n"
              "comment guesses ~50 us per channel on the ESP32-S3 and says it hasn't been timed on the chip.\n");
#else
  std::printf("(firmware/emg_orthosis/orthosis_core.h not found: only your PI was timed)\n");
#endif
  std::printf("This laptop is not the ESP32-S3 (240 MHz, single-precision FPU): the chip is slower, and double\n"
              "maths there runs in software, which is why the firmware uses float.\n");
  return sink == 12345.0;  // keeps the compiler from deleting the loops
}
