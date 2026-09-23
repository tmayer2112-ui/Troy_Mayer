// Offline replay: run the C++ filter over a recorded sensor stream as fast as possible.
//
//   qeskf_replay <config.txt> <sensors.csv> <estimates_out.csv> [--imu-only]
//
// Writes one row per input sample: t, p(3), v(3), q(w,x,y,z), ba(3), bg(3), Pdiag(15), nfeet, step_ns
// step_ns is the wall time of that step (contacts + update + predict), for the timing histogram.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "qeskf/eskf.hpp"
#include "qeskf/io.hpp"

using namespace qeskf;

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0] << " config.txt sensors.csv out.csv [--imu-only]\n";
    return 2;
  }
  const bool imu_only = argc > 4 && std::strcmp(argv[4], "--imu-only") == 0;
  const Config cfg = readConfig(argv[1]);
  const auto rows = readCsv(argv[2]);
  Eskf f = filterFromConfig(cfg);
  const double dt = scalar(cfg, "dt");

  std::vector<SensorRow> sensors;
  sensors.reserve(rows.size());
  for (const auto& r : rows) sensors.push_back(parseRow(r));

  std::FILE* out = std::fopen(argv[3], "w");
  if (!out) {
    std::cerr << "cannot write " << argv[3] << "\n";
    return 1;
  }
  std::fprintf(out, "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz");
  for (int i = 0; i < kNB; ++i) std::fprintf(out, ",P%d", i);
  std::fprintf(out, ",nfeet,step_ns\n");

  using clk = std::chrono::steady_clock;
  for (const auto& s : sensors) {
    const auto t0 = clk::now();
    const Snapshot snap = f.step(s.acc, s.gyro, s.q, s.c, dt, !imu_only);
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(clk::now() - t0).count();
    std::fprintf(out, "%.9f", s.t);
    for (int i = 0; i < 3; ++i) std::fprintf(out, ",%.17g", snap.p(i));
    for (int i = 0; i < 3; ++i) std::fprintf(out, ",%.17g", snap.v(i));
    std::fprintf(out, ",%.17g,%.17g,%.17g,%.17g", snap.q.w(), snap.q.x(), snap.q.y(), snap.q.z());
    for (int i = 0; i < 3; ++i) std::fprintf(out, ",%.17g", snap.ba(i));
    for (int i = 0; i < 3; ++i) std::fprintf(out, ",%.17g", snap.bg(i));
    for (int i = 0; i < kNB; ++i) std::fprintf(out, ",%.17g", snap.Pdiag(i));
    std::fprintf(out, ",%d,%lld\n", snap.nfeet, static_cast<long long>(ns));
  }
  std::fclose(out);
  std::cerr << "replayed " << sensors.size() << " samples; updates=" << f.stats.updates
            << " augment=" << f.stats.augment << " marginalize=" << f.stats.marginalize
            << " gated=" << f.stats.gated << "\n";
  return 0;
}
