// Real-time loop benchmark: run the filter at 1 kHz on wall-clock deadlines.
//
//   qeskf_bench_loop <config.txt> <sensors.csv> <timing_out.csv> [--rate 1000] [--cpu N]
//
// Each iteration sleeps to an absolute deadline (clock_nanosleep TIMER_ABSTIME), then runs one
// filter step on the next recorded sample. Logged per iteration:
//   wake_late_ns  how late the thread woke relative to its deadline (scheduler jitter)
//   compute_ns    time spent in Eskf::step
//   period_ns     wall time between consecutive wake-ups
// A deadline miss = wake lateness + compute exceeded one period.
// Tries SCHED_FIFO + mlockall; if the process lacks permission it says so and runs SCHED_OTHER.
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "qeskf/eskf.hpp"
#include "qeskf/io.hpp"

using namespace qeskf;

static inline long long nsNow() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static double pct(std::vector<long long> v, double p) {
  std::sort(v.begin(), v.end());
  const size_t i = std::min(v.size() - 1, static_cast<size_t>(std::llround(p / 100.0 * (v.size() - 1))));
  return static_cast<double>(v[i]);
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0] << " config.txt sensors.csv timing.csv [--rate HZ] [--cpu N]\n";
    return 2;
  }
  double rate = 1000.0;
  int cpu = -1;
  for (int i = 4; i + 1 < argc; ++i) {
    if (!std::strcmp(argv[i], "--rate")) rate = std::stod(argv[i + 1]);
    if (!std::strcmp(argv[i], "--cpu")) cpu = std::stoi(argv[i + 1]);
  }
  const Config cfg = readConfig(argv[1]);
  std::vector<SensorRow> sensors;
  for (const auto& r : readCsv(argv[2])) sensors.push_back(parseRow(r));
  Eskf f = filterFromConfig(cfg);
  const double dt = scalar(cfg, "dt");

  // --- real-time setup (best effort) ---
  std::string sched = "SCHED_OTHER";
  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) std::perror("sched_setaffinity");
  }
  const bool locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
  sched_param sp{};
  sp.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) sched = "SCHED_FIFO(80)";

  const long long period = static_cast<long long>(std::llround(1e9 / rate));
  const size_t n = sensors.size();
  std::vector<long long> late(n), compute(n), per(n);
  // Warm up caches/allocator on a copy so the first timed iterations are representative.
  {
    Eskf w = f;
    for (size_t k = 0; k < std::min<size_t>(200, n); ++k)
      w.step(sensors[k].acc, sensors[k].gyro, sensors[k].q, sensors[k].c, dt);
  }
  long long deadline = nsNow() + 10 * period;
  long long prev_wake = deadline - period;
  long misses = 0;
  for (size_t k = 0; k < n; ++k) {
    timespec ts{static_cast<time_t>(deadline / 1000000000LL), static_cast<long>(deadline % 1000000000LL)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
    const long long wake = nsNow();
    f.step(sensors[k].acc, sensors[k].gyro, sensors[k].q, sensors[k].c, dt);
    const long long done = nsNow();
    late[k] = wake - deadline;
    compute[k] = done - wake;
    per[k] = wake - prev_wake;
    prev_wake = wake;
    if (done - deadline > period) ++misses;
    deadline += period;
  }

  std::FILE* out = std::fopen(argv[3], "w");
  std::fprintf(out, "wake_late_ns,compute_ns,period_ns\n");
  for (size_t k = 0; k < n; ++k) std::fprintf(out, "%lld,%lld,%lld\n", late[k], compute[k], per[k]);
  std::fclose(out);

  double mean = 0, var = 0;
  for (auto p : per) mean += p;
  mean /= n;
  for (auto p : per) var += (p - mean) * (p - mean);
  std::printf("iterations        %zu at %.0f Hz\n", n, rate);
  std::printf("scheduler         %s, mlockall %s, cpu %s\n", sched.c_str(), locked ? "yes" : "no",
              cpu >= 0 ? std::to_string(cpu).c_str() : "any");
  std::printf("compute  us       p50 %.1f  p99 %.1f  p99.9 %.1f  max %.1f\n", pct(compute, 50) / 1e3,
              pct(compute, 99) / 1e3, pct(compute, 99.9) / 1e3, pct(compute, 100) / 1e3);
  std::printf("wake late us      p50 %.1f  p99 %.1f  p99.9 %.1f  max %.1f\n", pct(late, 50) / 1e3,
              pct(late, 99) / 1e3, pct(late, 99.9) / 1e3, pct(late, 100) / 1e3);
  std::printf("period   us       mean %.2f  std %.2f\n", mean / 1e3, std::sqrt(var / n) / 1e3);
  std::printf("deadline misses   %ld (%.4f%%)\n", misses, 100.0 * misses / n);
  return 0;
}
