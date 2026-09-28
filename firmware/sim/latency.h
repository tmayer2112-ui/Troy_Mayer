/* End-to-end latency with real EMG: muscle onset -> joint motion.
 *
 * Each trial is one real forearm contraction (sim/data/real_contractions.bin,
 * exported from 115 days of LibEMG MultiDay recordings run through the v2
 * board model at 1 kHz). It drives the flexor channel of the full controller,
 * after the full bench procedure (calibrate, zero, arm), with the limb's
 * weight on the joint:
 *
 *   0.5 s rest | 1.5 s contraction (50 ms ramp-up) | 1.0 s rest
 *
 * Rest is resting tone (3 % of the contraction) plus 2.3 counts of board noise.
 * Calibration puts the contraction at about half of maximum.
 */
#pragma once

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "harness.h"

namespace sim {

inline std::vector<std::vector<int16_t>> loadContractions(const std::string& path) {
  std::vector<std::vector<int16_t>> out;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return out;
  char magic[4];
  uint32_t n = 0;
  if (std::fread(magic, 1, 4, f) != 4 || std::fread(&n, 4, 1, f) != 1) { std::fclose(f); return out; }
  for (uint32_t k = 0; k < n; ++k) {
    uint32_t len = 0;
    if (std::fread(&len, 4, 1, f) != 1) break;
    std::vector<int16_t> x(len);
    if (std::fread(x.data(), 2, len, f) != len) break;
    out.push_back(std::move(x));
  }
  std::fclose(f);
  return out;
}

struct Latency { double t_move, t_50, t_stop, jitter; bool ok; };

inline Latency runLatencyTrial(const orth::Config& base, const std::vector<int16_t>& raw, unsigned seed) {
  orth::Config cfg = base;
  cfg.joint_max_deg = 60.0f;
  Harness h(cfg, PlantParams{});
  // contraction RMS, about its mean
  double mean = 0, rms = 0;
  for (int16_t v : raw) mean += v;
  mean /= raw.size();
  for (int16_t v : raw) rms += (v - mean) * (v - mean);
  rms = std::sqrt(rms / raw.size());
  h.rest_amp = 4.0;
  h.mvc_amp = 2.0 * rms;        // the test contraction sits at about half of maximum
  h.calibrate();
  h.run(0.01, orth::Request::Zero);
  handMove(h, 10.0);
  h.run(0.3);
  h.run(0.01, orth::Request::Arm);
  if (h.ctl.state() != orth::State::Armed) return {0, 0, 0, 0, false};

  const double t0 = h.t(), on = 0.5, dur = 1.5, ramp = 0.05;
  std::mt19937 g(seed);
  std::normal_distribution<double> noise(0.0, 2.3);
  h.adc_override = [&](double t, int ch, uint16_t& v) {
    if (ch != 0) return false;
    const double s = t - t0;
    const int k = static_cast<int>((s < on || s >= on + dur ? std::fmod(std::fabs(s), dur) : s - on) * 1000.0);
    const double x = raw[std::min<size_t>(k, raw.size() - 1)] - mean;
    double gain = 0.03;
    if (s >= on && s < on + dur) gain = std::min(1.0, (s - on) / ramp);
    v = static_cast<uint16_t>(std::clamp(std::lround(2048.0 + gain * x + noise(g)), 0L, 4095L));
    return true;
  };
  h.log.clear();
  h.record = true;
  h.run(on + dur + 1.0);
  if (h.ctl.state() != orth::State::Armed) return {0, 0, 0, 0, false};

  double steady = 0, steady2 = 0;
  int n = 0;
  for (const Sample& s : h.log)
    if (s.t - t0 >= on + 0.8 && s.t - t0 < on + dur) { steady += s.vel; steady2 += s.vel * s.vel; ++n; }
  steady /= n;
  const double jitter = std::sqrt(std::max(0.0, steady2 / n - steady * steady));
  double t_move = -1, t_50 = -1, t_stop = 0;
  for (const Sample& s : h.log) {
    const double r = s.t - t0 - on;
    if (r < 0) continue;
    if (t_move < 0 && s.vel > 2.0) t_move = r;
    if (t_50 < 0 && s.vel >= 0.5 * steady) t_50 = r;
    if (r > dur && std::fabs(s.vel) > 1.0) t_stop = r - dur;   // last moment still moving
  }
  return {t_move, t_50, t_stop, jitter, t_move >= 0 && t_50 >= 0};
}

struct LatencySummary { double move_med, move_p90, half_med, half_p90, stop_med, stop_p90, jitter_med; int n; };

inline double pct(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  const double idx = p * (v.size() - 1);
  const size_t i = static_cast<size_t>(idx);
  return i + 1 < v.size() ? v[i] + (idx - i) * (v[i + 1] - v[i]) : v.back();
}

inline LatencySummary summarize(const orth::Config& cfg, const std::vector<std::vector<int16_t>>& trials) {
  std::vector<double> mv, hf, st, jt;
  unsigned seed = 1;
  for (const auto& tr : trials) {
    const Latency l = runLatencyTrial(cfg, tr, seed++);
    if (!l.ok) continue;
    mv.push_back(1e3 * l.t_move);
    hf.push_back(1e3 * l.t_50);
    st.push_back(1e3 * l.t_stop);
    jt.push_back(l.jitter);
  }
  if (mv.empty()) return {0, 0, 0, 0, 0, 0, 0, 0};
  return {pct(mv, 0.5), pct(mv, 0.9), pct(hf, 0.5), pct(hf, 0.9), pct(st, 0.5), pct(st, 0.9), pct(jt, 0.5),
          static_cast<int>(mv.size())};
}

}  // namespace sim
