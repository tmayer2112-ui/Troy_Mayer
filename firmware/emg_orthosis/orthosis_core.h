/* =============================================================================
 * orthosis_core.h - everything that decides what the motor does.
 *
 * No Arduino, no ESP-IDF, no hardware. The same file is compiled into the
 * firmware (emg_orthosis.ino) and into the host simulator and tests
 * (firmware/sim), so every behaviour claimed in firmware/README.md is exercised
 * against a motor + transmission + limb model before it touches a person.
 *
 * Signal flow, once per 1 ms tick:
 *
 *   ADC x4 --> EmgChannel (DC track, rectify, 2-pole envelope, lead-off check)
 *          --> activation 0..1 (rest/MVC calibration, onset hysteresis)
 *          --> intent: flexor - extensor, dead zone, co-contraction stop
 *          --> joint speed command (deg/s)
 *          --> setpoint shaping (accel limit, braking curve into soft limits)
 *          --> PI velocity loop + back-EMF/friction feed-forward --> duty
 *   encoder edges --> VelocityEstimator (period over one quadrature cycle)
 *   Supervisor checks every tick: kill switch, driver OK, lead-off, EMG
 *   artefact, overspeed, over-travel, stall, runaway (encoder sign), tracking.
 * ============================================================================= */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#if defined(__GNUC__)
#define ORTH_FORCE_INLINE inline __attribute__((always_inline))
#else
#define ORTH_FORCE_INLINE inline
#endif

namespace orth {

constexpr float kPi = 3.14159265358979f;

// =============================================================================
// Configuration. Defaults match the v2 board, a goBILDA 5203 5.2:1 motor, a
// 4 mm screw and a 47 mm pulley. Items marked VERIFY need a bench measurement;
// the 'j' identify command measures the motor ones for you.
// =============================================================================
struct Config {
  // --- transmission: motor shaft -> joint ----------------------------------
  float gear_ratio = 5.2f;              // VERIFY with 'm'
  float screw_pitch_mm = 4.0f;          // cable travel per screw revolution
  float pulley_dia_mm = 47.0f;          // VERIFY: notebook uses 47 early, 65 late
  float counts_per_motor_rev = 28.0f;   // 7 PPR x 4 (quadrature)
  int encoder_sign = +1;                // 'j' tells you if this must be -1

  // --- joint envelope -------------------------------------------------------
  float joint_min_deg = 0.0f;           // measured from the 'z' zero
  float joint_max_deg = 30.0f;          // real travel is 120; widen once proven
  float joint_max_dps = 25.0f;          // top speed a user can command
  // Setpoint ramp. It also bounds how fast the joint stops once the EMG does.
  // Two tiers - see "Two tiers" under the velocity loop below.
  float joint_accel_dps2 = 150.0f;      // model from the datasheet: 0 -> 25 deg/s in 167 ms
  float joint_accel_fast_dps2 = 500.0f; // model identified on this motor ('j'): 50 ms
  float limit_decel_dps2 = 120.0f;      // braking curve into the soft limits
  float limit_margin_deg = 0.5f;        // stop this far inside each limit

  // --- motor model, used for feed-forward and gain design -------------------
  float dps_per_duty = 187.0f;          // joint deg/s per unit duty, unloaded
  float friction_duty = 0.05f;          // duty needed to break away (Coulomb)
  float plant_tau_s = 0.020f;           // mechanical time constant J*R/(Kt*Ke)
  float duty_max = 0.60f;               // hard ceiling on |duty|

  // --- velocity loop design --------------------------------------------------
  // IMC tuning for a first-order plant K/(tau s + 1): cancel the plant pole
  // (Ti = tau) and place the closed loop at 1/lambda.
  //   Kp = tau / (K lambda) [duty per deg/s],  Ki = 1 / (K lambda) [duty per deg]
  // Ki is also the holding stiffness at zero command: the integral of a speed
  // error is a position error, so the loop holds the joint where it stopped.
  //
  // Two tiers. Fast response leans on the feed-forward, and feed-forward is only
  // as good as the motor model. With datasheet numbers, a fast ramp overshoots
  // up to ~45 % when friction or supply voltage differ from the model; no loop
  // tuning fixes a wrong model. So the loop starts conservative and the fast
  // tier (ramp 500 deg/s^2, lambda 30 ms) unlocks only once 'j' has measured
  // this motor, or model_identified is set with measured values pasted in.
  // Across 36 perturbed motors, each identified first, the fast tier's worst
  // step overshoot is ~20 % (600 deg/s^2 reached 27 %). 30 ms is the floor for
  // lambda: at 20 ms the stop starts to ring (latency_sweep).
  float lambda_s = 0.050f;
  float lambda_fast_s = 0.030f;
  bool model_identified = false;
  bool closed_loop = true;              // false = feed-forward only (used for comparison)

  // --- EMG ------------------------------------------------------------------
  float fs_hz = 1000.0f;                // sample and control rate
  float dc_track_hz = 0.5f;             // follows VREF/electrode drift
  // Envelope estimator. Bayes (default) is a recursive posterior over EMG
  // amplitude (Sanger 2007): it can jump within a few samples at a contraction
  // onset and still averages hard during a steady one. On 200 real forearm
  // contractions through the v2 board model (firmware/analysis/latency_study.py)
  // it halved onset-to-half-command time against the 4 Hz two-pole filter
  // (97 -> 49 ms median), cut the stop lag from 73 to 15 ms (p90), and had
  // slightly LESS jitter. A jump model reacts to single spikes, so an onset
  // must hold for confirm_ms: with 20 ms, zero false starts at one motion
  // artefact per second; with none, 62 a minute.
  enum class Envelope : uint8_t { Bayes, TwoPole };
  Envelope envelope = Envelope::Bayes;
  float bayes_alpha = 1e-4f;            // per-sample diffusion between neighbouring amplitude bins
  float bayes_beta = 1e-12f;            // per-sample probability of a jump to any amplitude
  uint32_t confirm_ms = 20;             // envelope must stay above onset this long
  // TwoPole: two equal one-pole low passes, mean delay 2/(2 pi fc) = 80 ms at 4 Hz.
  float envelope_hz = 4.0f;
  float mvc_fraction = 0.60f;           // full command at 60 % of calibrated max
  float onset_k_sigma = 6.0f;           // onset = rest mean + k * rest SD ...
  float onset_min_mvc = 0.08f;          // ... but never below 8 % of MVC
  float release_ratio = 0.70f;          // release at 70 % of the onset level
  float deadzone = 0.08f;               // |flex - ext| below this = no motion
  float cocontract_level = 0.35f;       // both above this = commanded stop
  float min_mvc_over_rest = 4.0f;       // calibration rejected below this ratio
  float artefact_mvc_ratio = 2.0f;      // envelope > 2x MVC = artefact, not intent
  uint16_t adc_rail_margin = 40;        // counts from 0 or 4095 = railed
  float rail_fraction_fault = 0.20f;    // >20 % railed samples in 100 ms = lead off

  // --- supervisor -------------------------------------------------------------
  bool require_mot_ok = true;           // J4 pin 8. See README before disabling.
  float stall_duty = 0.30f;
  float stall_speed_dps = 1.5f;
  uint32_t stall_ms = 400;
  float overspeed_ratio = 1.5f;         // x joint_max_dps
  uint32_t overspeed_ms = 50;
  float overtravel_deg = 3.0f;          // past a soft limit = fault
  float runaway_duty = 0.25f;           // duty and motion disagree in sign ...
  float runaway_speed_dps = 5.0f;       // ... at this speed ...
  uint32_t runaway_ms = 60;             // ... for this long = wrong encoder sign
  float track_err_dps = 20.0f;
  uint32_t track_err_ms = 500;
  uint32_t max_motion_ms = 15000;       // longest uninterrupted commanded motion
  uint32_t encoder_error_limit = 20;    // illegal quadrature transitions

  // --- calibration --------------------------------------------------------------
  uint32_t cal_rest_ms = 4000;
  uint32_t cal_mvc_ms = 4000;

  // derived ------------------------------------------------------------------
  float degPerCount() const {
    const float mm_per_joint_deg = kPi * pulley_dia_mm / 360.0f;
    const float joint_deg_per_output_rev = screw_pitch_mm / mm_per_joint_deg;
    return joint_deg_per_output_rev / (counts_per_motor_rev * gear_ratio);
  }
  float lambda() const { return model_identified ? lambda_fast_s : lambda_s; }
  float accel() const { return model_identified ? joint_accel_fast_dps2 : joint_accel_dps2; }
  float kp() const { return closed_loop ? plant_tau_s / (dps_per_duty * lambda()) : 0.0f; }
  float ki() const { return closed_loop ? 1.0f / (dps_per_duty * lambda()) : 0.0f; }
};

// =============================================================================
// Small filters
// =============================================================================
struct OnePole {
  float a = 1.0f, y = 0.0f;
  void design(float fc_hz, float fs_hz) { a = 1.0f - std::exp(-2.0f * kPi * fc_hz / fs_hz); }
  float step(float x) { y += a * (x - y); return y; }
};

// =============================================================================
// Encoder: the ISR feeds EdgeTracker, the control tick reads a snapshot.
//
// Why not count-difference per tick: at 25 deg/s the motor produces ~370
// counts/s, i.e. 0.37 counts per 1 ms tick, so counts/tick is mostly 0 or 1 and
// the speed estimate is quantised in 67 deg/s steps. Timing the edges instead
// (the "M/T" method) gives a real number every edge. Timing a full quadrature
// cycle (4 edges) cancels the encoder's channel duty/phase errors, which would
// otherwise show up as a 4-periodic ripple.
// =============================================================================
// Quadrature decode without a lookup table (a table would sit in flash, which
// an IRAM interrupt must not read). AB = (A << 1) | B. Map the Gray sequence
// 00 -> 10 -> 11 -> 01 to positions 0..3; the position difference mod 4 is the
// step: 1 = +1, 3 = -1, 2 = both channels changed at once (illegal), 0 = none.
ORTH_FORCE_INLINE uint8_t quadPos(uint8_t ab) {
  const uint8_t a = (ab >> 1) & 1u, b = ab & 1u;
  return static_cast<uint8_t>((a ^ b) | (b << 1));
}

struct EdgeTracker {
  int32_t count = 0;          // raw counts, before encoder_sign
  uint32_t t[5] = {0};        // edge times (us), t[0] newest, same direction only
  uint8_t run = 0;            // how many of t[] are valid (saturates at 5)
  int8_t dir = 0;             // direction of the newest edge
  uint8_t state = 0;          // last AB state
  uint32_t errors = 0;        // illegal transitions (both channels changed)

  ORTH_FORCE_INLINE void reset(uint8_t ab) { state = ab; run = 0; }

  // Called from the ISR with the current AB bits and a microsecond timestamp.
  ORTH_FORCE_INLINE void onChange(uint8_t ab, uint32_t now_us) {
    const uint8_t step = static_cast<uint8_t>((quadPos(ab) - quadPos(state)) & 3u);
    state = ab;
    if (step == 0) return;
    if (step == 2) { ++errors; return; }
    const int8_t d = step == 1 ? 1 : -1;
    count += d;
    if (d != dir) { run = 0; dir = d; }
    t[4] = t[3]; t[3] = t[2]; t[2] = t[1]; t[1] = t[0]; t[0] = now_us;
    if (run < 5) ++run;
  }
};

class VelocityEstimator {
 public:
  // Returns counts/s (raw sign). Call once per tick with a copy of the tracker.
  float update(const EdgeTracker& e, uint32_t now_us) {
    if (e.count != last_count_) {
      last_count_ = e.count;
      const uint8_t k = e.run >= 5 ? 4 : static_cast<uint8_t>(e.run > 0 ? e.run - 1 : 0);
      const uint32_t span_us = k ? e.t[0] - e.t[k] : 0;
      if (k && span_us) {
        est_ = e.dir * static_cast<float>(k) / usToS(span_us);
        interval_s_ = usToS(span_us) / k;
      } else {
        est_ = 0.0f;  // first edge after a reversal: no interval yet
      }
      last_edge_us_ = e.t[0];
      have_edge_ = true;
    }
    if (!have_edge_) return 0.0f;
    // No new edge for longer than the recent edge spacing allows (1.5x, to
    // tolerate uneven edge placement): the shaft has slowed, and its speed
    // cannot exceed one count per elapsed time.
    const float elapsed = usToS(now_us - last_edge_us_);
    if (elapsed > 0.25f) { est_ = 0.0f; return 0.0f; }
    if (elapsed > 1.5f * interval_s_) {
      const float bound = 1.0f / elapsed;
      if (std::fabs(est_) > bound) est_ = std::copysign(bound, est_);
    }
    return est_;
  }
  void reset() { est_ = 0.0f; have_edge_ = false; interval_s_ = 0.0f; }

 private:
  static float usToS(uint32_t us) { return static_cast<float>(us) * 1e-6f; }
  int32_t last_count_ = 0;
  uint32_t last_edge_us_ = 0;
  bool have_edge_ = false;
  float est_ = 0.0f;
  float interval_s_ = 0.0f;
};

// =============================================================================
// EMG channel: raw ADC counts -> envelope -> activation
// =============================================================================
// Bayesian amplitude estimator (Sanger, "Bayesian filtering of myoelectric
// signals", J Neurophysiol 2007). The state is the EMG's local RMS amplitude,
// kept as a probability over kBins log-spaced levels. Each sample:
//   predict: diffuse to neighbouring levels (alpha) and allow a jump anywhere (beta)
//   update:  multiply by the likelihood of this sample, Laplacian with that RMS
//            (EMG is heavier-tailed than Gaussian; a Gaussian model is thrown
//            around more by single large samples)
//   output:  the posterior mean of log-amplitude (continuous; the most probable
//            bin would step in 12 % increments and gave slightly more jitter)
// Cost: 2 x kBins exp() per sample, ~50 us per channel on an ESP32-S3.
class BayesEnvelope {
 public:
  static constexpr int kBins = 64;
  void design(float alpha, float beta, float lo = 1.0f, float hi = 2000.0f) {
    alpha_ = alpha;
    beta_ = beta;
    for (int i = 0; i < kBins; ++i) {
      log_level_[i] = std::log(lo) + std::log(hi / lo) * i / (kBins - 1);
      level_[i] = std::exp(log_level_[i]);
      inv_[i] = 1.41421356f / level_[i];
      p_[i] = 1.0f / kBins;
    }
  }
  float step(float e) {
    const float ae = std::fabs(e);
    float q[kBins];
    float s = 0.0f;
    for (int i = 0; i < kBins; ++i) {
      const float l = i > 0 ? p_[i - 1] : p_[i];
      const float r = i < kBins - 1 ? p_[i + 1] : p_[i];
      float v = (1.0f - 2.0f * alpha_) * p_[i] + alpha_ * (l + r);
      v = (1.0f - beta_) * v + beta_ / kBins;
      v *= std::exp(-ae * inv_[i]) * inv_[i];
      q[i] = v;
      s += v;
    }
    const float inv_s = s > 0.0f ? 1.0f / s : 0.0f;
    float log_mean = 0.0f;
    for (int i = 0; i < kBins; ++i) {
      p_[i] = s > 0.0f ? q[i] * inv_s : 1.0f / kBins;
      log_mean += p_[i] * log_level_[i];
    }
    y_ = std::exp(log_mean);
    return y_;
  }
  float y() const { return y_; }

 private:
  float alpha_ = 1e-4f, beta_ = 1e-12f;
  float level_[kBins] = {}, log_level_[kBins] = {}, inv_[kBins] = {}, p_[kBins] = {};
  float y_ = 0.0f;
};

struct EmgChannel {
  OnePole dc, e1, e2, slow;
  BayesEnvelope bayes;
  bool use_bayes = true;
  uint32_t above_n = 0;       // consecutive samples above onset (for confirm_ms)
  float rest_mean = 0.0f, rest_sd = 1.0f, mvc = 1.0f;
  float onset = 1e9f, release = 1e9f;
  bool active = false;
  float activation = 0.0f;
  // lead-off: a detached electrode drives the in-amp to a rail and holds it
  // there. EMG itself is zero-mean about VREF, so even a clipping contraction
  // never sits on a rail for long. 20 consecutive railed samples, or 20 % of a
  // 100-sample window, is a lead off.
  uint16_t rail_run = 0, rail_hits = 0, window_n = 0;
  float rail_fraction = 0.0f;
  bool leadOff(const Config& c) const { return rail_run >= 20 || rail_fraction > c.rail_fraction_fault; }

  void design(const Config& c) {
    dc.design(c.dc_track_hz, c.fs_hz);
    e1.design(c.envelope_hz, c.fs_hz);
    e2.design(c.envelope_hz, c.fs_hz);
    slow.design(2.0f, c.fs_hz);
    use_bayes = c.envelope == Config::Envelope::Bayes;
    bayes.design(c.bayes_alpha, c.bayes_beta);
  }
  void prime(float x) { dc.y = x; }
  float envelope() const { return use_bayes ? bayes.y() : e2.y; }
  // A 2 Hz smoothing of the envelope: used for the MVC peak, so one lucky
  // sample during calibration can't set the scale.
  float slowEnvelope() const { return slow.y; }

  void sample(uint16_t raw, const Config& c) {
    const float x = static_cast<float>(raw);
    const float ac = x - dc.step(x);
    if (use_bayes) bayes.step(ac);
    else e2.step(e1.step(std::fabs(ac)));
    slow.step(envelope());
    const bool railed = raw <= c.adc_rail_margin || raw >= 4095 - c.adc_rail_margin;
    rail_run = railed ? static_cast<uint16_t>(std::min(rail_run + 1, 60000)) : 0;
    if (railed) ++rail_hits;
    if (++window_n >= 100) {
      rail_fraction = rail_hits / 100.0f;
      rail_hits = 0;
      window_n = 0;
    }
  }

  void setThresholds(const Config& c) {
    const float t = rest_mean + c.onset_k_sigma * rest_sd;
    const float floor_t = rest_mean + c.onset_min_mvc * (mvc - rest_mean);
    onset = std::max(t, floor_t);
    release = rest_mean + c.release_ratio * (onset - rest_mean);
    active = false;
  }

  void updateActivation(const Config& c) {
    const float e = envelope();
    above_n = e > onset ? above_n + 1 : 0;
    const uint32_t confirm_n = static_cast<uint32_t>(c.confirm_ms * c.fs_hz / 1000.0f);
    if (active) { if (e < release) active = false; }
    else if (above_n > confirm_n) { active = true; }
    if (!active) { activation = 0.0f; return; }
    const float span = std::max(c.mvc_fraction * mvc - onset, 1.0f);
    activation = std::clamp((e - onset) / span, 0.0f, 1.0f);
  }
};

// Intent: two-site proportional control. Returns a joint speed command in deg/s.
inline float intentToSpeed(float a_flex, float a_ext, const Config& c) {
  if (a_flex > c.cocontract_level && a_ext > c.cocontract_level) return 0.0f;  // stop
  const float net = a_flex - a_ext;
  if (std::fabs(net) < c.deadzone) return 0.0f;
  const float mag = (std::fabs(net) - c.deadzone) / (1.0f - c.deadzone);
  return std::copysign(mag * c.joint_max_dps, net);
}

// =============================================================================
// Setpoint shaping: acceleration limit, then a braking curve that makes the
// joint decelerate smoothly to a stop inside each soft limit. The speed allowed
// at distance x from the stop point is sqrt(2 a x): a constant-deceleration
// profile, so the joint can never arrive at a limit faster than it can stop.
// =============================================================================
class SetpointShaper {
 public:
  float step(float target_dps, float pos_deg, float dt, const Config& c) {
    const float max_step = c.accel() * dt;
    ramping_ = std::fabs(target_dps - ref_) > max_step;
    ref_ += std::clamp(target_dps - ref_, -max_step, max_step);
    const float to_max = (c.joint_max_deg - c.limit_margin_deg) - pos_deg;
    const float to_min = pos_deg - (c.joint_min_deg + c.limit_margin_deg);
    const float up = to_max > 0.0f ? std::sqrt(2.0f * c.limit_decel_dps2 * to_max) : 0.0f;
    const float down = to_min > 0.0f ? std::sqrt(2.0f * c.limit_decel_dps2 * to_min) : 0.0f;
    ref_ = std::clamp(ref_, -down, up);   // store the clamp so the ramp can't wind up
    return ref_;
  }
  void reset() { ref_ = 0.0f; ramping_ = false; }
  float ref() const { return ref_; }
  // True while the acceleration limit is shaping the setpoint (a transient).
  bool ramping() const { return ramping_; }

 private:
  float ref_ = 0.0f;
  bool ramping_ = false;
};

// =============================================================================
// PI velocity loop with plant-inversion feed-forward and anti-windup.
//
//   duty = ff + Kp e + I,      I += Ki e dt  (only when not saturating further
//                                              in the same sign)
//   ff   = (ref + tau * d(ref)/dt) / K + friction_duty * sign(ref)
//
// ff is the inverse of the first-order motor model K/(tau s + 1): the ref/K
// term supplies the back-EMF voltage for the commanded speed, the tau*d(ref)/dt
// term supplies the torque to accelerate the inertia. With the model right, the
// joint follows the shaped setpoint with no lag, so the integrator does not
// charge up during a ramp and then overshoot. The PI is left to reject what the
// model misses: limb weight, friction error, supply sag.
// =============================================================================
class VelocityPI {
 public:
  // ramping: the setpoint is being shaped by the acceleration limit this tick.
  // Tracking a ramp is the feed-forward's job. An integrator that charges on
  // the joint lagging a ramp (large when the model's inertia is off) comes back
  // out as overshoot, so while ramping it may only discharge - pull back a joint
  // that is AHEAD of the setpoint (feed-forward too strong: friction or supply
  // voltage above the model) - never push harder. Its real job, the
  // quasi-static load (limb weight, friction), is untouched.
  float step(float ref_dps, float meas_dps, float dt, const Config& c, bool ramping = false) {
    const float e = ref_dps - meas_dps;
    const float dref = (ref_dps - prev_ref_) / dt;
    prev_ref_ = ref_dps;
    float ff = (ref_dps + c.plant_tau_s * dref) / c.dps_per_duty;
    if (std::fabs(ref_dps) > 0.5f) ff += std::copysign(c.friction_duty, ref_dps);
    const float unsat = ff + c.kp() * e + integ_;
    const float out = std::clamp(unsat, -c.duty_max, c.duty_max);
    const bool pushing_further = (unsat != out) && ((e > 0.0f) == (unsat > 0.0f));
    const bool charging = ramping && (e > 0.0f) == (ref_dps > 0.0f);
    if (!pushing_further && !charging) integ_ += c.ki() * e * dt;
    integ_ = std::clamp(integ_, -c.duty_max, c.duty_max);
    last_ = out;
    return out;
  }
  void reset() { integ_ = 0.0f; last_ = 0.0f; prev_ref_ = 0.0f; }
  float integrator() const { return integ_; }
  float last() const { return last_; }

 private:
  float integ_ = 0.0f;
  float last_ = 0.0f;
  float prev_ref_ = 0.0f;
};

// =============================================================================
// The controller: state machine + supervisor. One call per tick.
// =============================================================================
enum class State : uint8_t { CalRest, CalFlex, CalExt, Idle, Armed, Identify, Fault };
enum class Request : uint8_t { None, Calibrate, Arm, Disarm, Zero, Identify };

inline const char* stateName(State s) {
  switch (s) {
    case State::CalRest:  return "CAL_REST";
    case State::CalFlex:  return "CAL_FLEX";
    case State::CalExt:   return "CAL_EXT";
    case State::Idle:     return "IDLE";
    case State::Armed:    return "ARMED";
    case State::Identify: return "IDENTIFY";
    case State::Fault:    return "FAULT";
  }
  return "?";
}

struct Inputs {
  uint16_t adc[4] = {2048, 2048, 2048, 2048};
  EdgeTracker enc;            // snapshot copied out of the ISR
  uint32_t now_us = 0;
  bool kill_closed = false;   // DPDT pole B reads LOW
  bool mot_ok = false;        // OUTF, motor-side status
};

struct Outputs {
  float duty = 0.0f;          // -1..1, + drives toward joint_max
  bool enable = false;        // false = coast (EN low). true with duty 0 = hold/brake
};

// Result of the 'j' identification run.
struct IdentResult {
  bool done = false, ok = false, sign_ok = false;
  float dps_per_duty = 0.0f, friction_duty = 0.0f, tau_s = 0.0f;
  const char* note = "";
};

class Controller {
 public:
  explicit Controller(const Config& cfg) : cfg_(cfg) {
    for (auto& ch : ch_) ch.design(cfg_);
    beginCal(State::CalRest);
  }

  Config& config() { return cfg_; }
  const Config& config() const { return cfg_; }
  State state() const { return state_; }
  const char* faultReason() const { return fault_; }
  const EmgChannel& channel(int i) const { return ch_[i]; }
  float jointDeg() const { return pos_deg_; }
  float jointDps() const { return vel_dps_; }
  float speedCommand() const { return cmd_dps_; }
  float speedRef() const { return shaper_.ref(); }
  float duty() const { return out_.duty; }
  bool zeroed() const { return zeroed_; }
  const IdentResult& ident() const { return ident_; }
  const char* lastMessage() const { return msg_; }
  uint32_t messageSeq() const { return msg_seq_; }

  void prime(const uint16_t adc[4]) { for (int i = 0; i < 4; ++i) ch_[i].prime(adc[i]); }

  // For tests: skip the EMG path and command a joint speed directly.
  void setSpeedOverride(bool on, float dps = 0.0f) { override_ = on; override_dps_ = dps; }

  Outputs step(const Inputs& in, Request req) {
    const float dt = 1.0f / cfg_.fs_hz;
    ++ticks_;

    // --- sense ----------------------------------------------------------------
    for (int i = 0; i < 4; ++i) {
      ch_[i].sample(in.adc[i], cfg_);
      ch_[i].updateActivation(cfg_);
    }
    const float cps = vel_est_.update(in.enc, in.now_us);
    vel_dps_ = cfg_.encoder_sign * cps * cfg_.degPerCount();
    pos_deg_ = cfg_.encoder_sign * static_cast<float>(in.enc.count - zero_count_) * cfg_.degPerCount();
    raw_count_ = in.enc.count;

    handleRequest(req, in);

    // --- act ----------------------------------------------------------------
    switch (state_) {
      case State::CalRest:
      case State::CalFlex:
      case State::CalExt:  calibrationTick(); return coast();
      case State::Idle:    return coast();
      case State::Fault:   return coast();
      case State::Identify: return identifyTick(in, dt);
      case State::Armed:   return armedTick(in, dt);
    }
    return coast();
  }

 private:
  // ---------------------------------------------------------------- requests
  void handleRequest(Request req, const Inputs& in) {
    switch (req) {
      case Request::None: break;
      case Request::Calibrate:
        if (state_ == State::Armed || state_ == State::Identify) say("disarm before calibrating");
        else beginCal(State::CalRest);
        break;
      case Request::Zero:
        if (state_ == State::Armed || state_ == State::Identify) { say("disarm before zeroing"); break; }
        zero_count_ = in.enc.count;
        pos_deg_ = 0.0f;
        zeroed_ = true;
        say("joint zeroed here");
        break;
      case Request::Disarm:
        if (state_ == State::Armed || state_ == State::Identify || state_ == State::Fault) {
          const bool was_fault = state_ == State::Fault;
          state_ = State::Idle;
          say(was_fault ? "fault cleared" : "disarmed");
        }
        break;
      case Request::Arm:
        if (const char* why = armBlocker(in)) { say(why); break; }
        motionReset();
        enc_err_base_ = in.enc.errors;
        state_ = State::Armed;
        say("ARMED");
        break;
      case Request::Identify:
        if (const char* why = armBlocker(in, /*need_cal=*/false)) { say(why); break; }
        motionReset();
        enc_err_base_ = in.enc.errors;
        ident_ = IdentResult{};
        ident_t_ = 0;
        ident_phase_ = 0;
        state_ = State::Identify;
        say("IDENTIFY: open-loop duty steps, joint must be free and unloaded");
        break;
    }
  }

  const char* armBlocker(const Inputs& in, bool need_cal = true) const {
    if (state_ != State::Idle) return "can only arm from IDLE";
    if (need_cal && !calibrated_) return "not calibrated ('c')";
    if (!zeroed_) return "joint not zeroed ('z' at the lower limit)";
    if (!in.kill_closed) return "kill switch is open";
    if (cfg_.require_mot_ok && !in.mot_ok) return "driver not reporting OK (MOT_OK low)";
    if (need_cal && (ch_[0].active || ch_[1].active)) return "muscle active - relax first";
    if (pos_deg_ < cfg_.joint_min_deg - 1.0f || pos_deg_ > cfg_.joint_max_deg + 1.0f)
      return "joint outside the soft limits";
    return nullptr;
  }

  // ---------------------------------------------------------------- calibration
  void beginCal(State s) {
    state_ = s;
    cal_ticks_ = 0;
    for (auto& a : acc_) a = Accum{};
    say(s == State::CalRest ? "CALIBRATION: relax completely for 4 s"
        : s == State::CalFlex ? "contract the FLEXOR as hard as you can for 4 s"
                              : "contract the EXTENSOR as hard as you can for 4 s");
  }

  void calibrationTick() {
    for (int i = 0; i < 4; ++i) {
      const float e = ch_[i].envelope();
      acc_[i].sum += e;
      acc_[i].sum_sq += static_cast<double>(e) * e;
      acc_[i].peak = std::max(acc_[i].peak, ch_[i].slowEnvelope());
      ++acc_[i].n;
    }
    const uint32_t ms = ++cal_ticks_ * 1000u / static_cast<uint32_t>(cfg_.fs_hz);
    if (state_ == State::CalRest && ms >= cfg_.cal_rest_ms) {
      for (int i = 0; i < 4; ++i) {
        const double m = acc_[i].sum / acc_[i].n;
        const double v = acc_[i].sum_sq / acc_[i].n - m * m;
        ch_[i].rest_mean = static_cast<float>(m);
        ch_[i].rest_sd = v > 0 ? static_cast<float>(std::sqrt(v)) : 1.0f;
      }
      beginCal(State::CalFlex);
    } else if (state_ == State::CalFlex && ms >= cfg_.cal_mvc_ms) {
      ch_[0].mvc = acc_[0].peak;
      beginCal(State::CalExt);
    } else if (state_ == State::CalExt && ms >= cfg_.cal_mvc_ms) {
      ch_[1].mvc = acc_[1].peak;
      for (auto& ch : ch_) ch.setThresholds(cfg_);
      const bool ok = ch_[0].mvc > cfg_.min_mvc_over_rest * ch_[0].rest_mean &&
                      ch_[1].mvc > cfg_.min_mvc_over_rest * ch_[1].rest_mean;
      calibrated_ = ok;
      state_ = State::Idle;
      say(ok ? "calibrated - 'z' to zero, then 'a' to arm"
             : "calibration REJECTED: contraction barely above rest - check electrodes, then 'c'");
    }
  }

  // ---------------------------------------------------------------- armed
  Outputs armedTick(const Inputs& in, float dt) {
    if (!in.kill_closed) {           // the switch already cut EN in hardware
      state_ = State::Idle;
      say("kill switch opened - disarmed");
      return coast();
    }
    if (const char* why = supervise(in)) return fault(why);

    const float a_flex = ch_[0].activation, a_ext = ch_[1].activation;
    cmd_dps_ = override_ ? override_dps_ : intentToSpeed(a_flex, a_ext, cfg_);
    const float ref = shaper_.step(cmd_dps_, pos_deg_, dt, cfg_);
    const float duty = pi_.step(ref, vel_dps_, dt, cfg_, shaper_.ramping());

    if (const char* why = superviseMotion(ref, duty)) return fault(why);
    out_ = Outputs{duty, true};
    return out_;
  }

  // Checks that don't depend on this tick's command.
  const char* supervise(const Inputs& in) {
    if (cfg_.require_mot_ok && !in.mot_ok) return "driver fault (MOT_OK low)";
    if (in.enc.errors - enc_err_base_ > cfg_.encoder_error_limit) return "encoder errors (noise or missed edges)";
    for (int i = 0; i < 2; ++i)
      if (ch_[i].leadOff(cfg_)) return "EMG lead off / channel railed";
    const bool artefact = ch_[0].envelope() > cfg_.artefact_mvc_ratio * ch_[0].mvc ||
                          ch_[1].envelope() > cfg_.artefact_mvc_ratio * ch_[1].mvc;
    if (timer(artefact_n_, artefact, cfg_.confirm_ms)) return "EMG artefact (envelope > 2x MVC)";
    if (pos_deg_ > cfg_.joint_max_deg + cfg_.overtravel_deg ||
        pos_deg_ < cfg_.joint_min_deg - cfg_.overtravel_deg) return "over-travel past soft limit";
    if (timer(overspeed_n_, std::fabs(vel_dps_) > cfg_.overspeed_ratio * cfg_.joint_max_dps, cfg_.overspeed_ms))
      return "overspeed";
    return nullptr;
  }

  // Checks on the loop's own behaviour.
  const char* superviseMotion(float ref, float duty) {
    const bool moving_cmd = std::fabs(ref) > 2.0f;
    if (timer(stall_n_, moving_cmd && std::fabs(duty) > cfg_.stall_duty &&
                             std::fabs(vel_dps_) < cfg_.stall_speed_dps, cfg_.stall_ms))
      return "stall: commanded motion, joint not moving";
    // Wrong encoder sign turns a velocity loop into positive feedback: duty and
    // measured motion disagree and both grow. Catch it within tens of ms.
    if (timer(runaway_n_, std::fabs(duty) > cfg_.runaway_duty && duty * vel_dps_ < 0.0f &&
                               std::fabs(vel_dps_) > cfg_.runaway_speed_dps, cfg_.runaway_ms))
      return "runaway: motion opposes duty (encoder sign?)";
    if (timer(track_n_, std::fabs(ref - vel_dps_) > cfg_.track_err_dps, cfg_.track_err_ms))
      return "cannot track speed command";
    if (timer(motion_n_, std::fabs(cmd_dps_) > 0.5f, cfg_.max_motion_ms))
      return "continuous motion limit";
    return nullptr;
  }

  // ---------------------------------------------------------------- identify
  // Open-loop duty steps, both directions, two levels. Each step's settled
  // speed gives omega = K (d - d_fric); two levels give K and d_fric per
  // direction (gravity shows up as a direction-dependent d_fric and cancels
  // in the average). Time to 63 % of settled speed gives tau.
  Outputs identifyTick(const Inputs& in, float dt) {
    if (!in.kill_closed) { state_ = State::Idle; say("kill switch opened - identify aborted"); return coast(); }
    if (cfg_.require_mot_ok && !in.mot_ok) return fault("driver fault (MOT_OK low)");
    if (pos_deg_ > cfg_.joint_max_deg + cfg_.overtravel_deg ||
        pos_deg_ < cfg_.joint_min_deg - cfg_.overtravel_deg) return fault("over-travel during identify");

    static constexpr float kDuty[4] = {0.20f, -0.20f, 0.32f, -0.32f};
    static constexpr uint32_t kStepMs = 250, kRestMs = 300;
    const uint32_t ms = ++ident_t_ * 1000u / static_cast<uint32_t>(cfg_.fs_hz);
    if (ident_phase_ >= 4) { finishIdentify(); return coast(); }

    const float d = kDuty[ident_phase_];
    if (ms <= kStepMs) {
      // stop early if the next sample would cross a limit
      const float margin = 1.0f;
      if ((d > 0 && pos_deg_ > cfg_.joint_max_deg - margin) || (d < 0 && pos_deg_ < cfg_.joint_min_deg + margin)) {
        return fault("identify ran into a soft limit - start mid-range");
      }
      auto& s = ident_steps_[ident_phase_];
      if (ms == 1) s = IdentStep{};
      const float v = vel_dps_;
      s.trace[std::min<uint32_t>(ms - 1, kTraceLen - 1)] = v;
      if (ms > kStepMs * 6 / 10) { s.settled += v; ++s.n; }
      if (d * v < 0 && std::fabs(v) > cfg_.runaway_speed_dps) s.sign_bad = true;
      out_ = Outputs{d, true};
      return out_;
    }
    if (ms <= kStepMs + kRestMs) { out_ = Outputs{0.0f, true}; return out_; }  // brake between steps
    ++ident_phase_;
    ident_t_ = 0;
    return Outputs{0.0f, true};
  }

  void finishIdentify() {
    IdentResult r;
    r.done = true;
    float w[4];
    bool sign_bad = false;
    for (int i = 0; i < 4; ++i) {
      w[i] = ident_steps_[i].n ? ident_steps_[i].settled / ident_steps_[i].n : 0.0f;
      sign_bad |= ident_steps_[i].sign_bad;
    }
    r.sign_ok = !sign_bad && w[0] > 0 && w[2] > 0 && w[1] < 0 && w[3] < 0;
    if (!r.sign_ok) {
      r.note = (w[0] < 0 && w[2] < 0 && w[1] > 0 && w[3] > 0)
                   ? "encoder sign is reversed: set encoder_sign = -1"
                   : "no consistent motion - check motor power, EN and encoder wiring";
      ident_ = r;
      state_ = State::Idle;
      say(r.note);
      return;
    }
    const float kpos = (w[2] - w[0]) / (0.32f - 0.20f);
    const float kneg = (w[3] - w[1]) / (-0.32f + 0.20f);
    r.dps_per_duty = 0.5f * (kpos + kneg);
    const float dpos = 0.20f - w[0] / kpos;
    const float dneg = -(-0.20f - w[1] / kneg);
    r.friction_duty = std::max(0.0f, 0.5f * (dpos + dneg));
    // tau from the larger positive step: first sample above 63 % of settled
    const IdentStep& s = ident_steps_[2];
    const float target = 0.632f * w[2];
    float tau = 0.0f;
    for (uint32_t k = 0; k < kTraceLen; ++k) {
      if (s.trace[k] >= target) { tau = (k + 1) / cfg_.fs_hz; break; }
    }
    r.tau_s = tau;
    r.ok = r.dps_per_duty > 20.0f && r.dps_per_duty < 1000.0f && tau > 0.002f && tau < 0.2f;
    r.note = r.ok ? "identified - gains updated in RAM; copy the values into Config"
                  : "implausible result - gains NOT updated";
    if (r.ok) {
      cfg_.dps_per_duty = r.dps_per_duty;
      cfg_.friction_duty = std::min(r.friction_duty, 0.2f);
      cfg_.plant_tau_s = r.tau_s;
      cfg_.model_identified = true;   // unlocks the fast tier
    }
    ident_ = r;
    state_ = State::Idle;
    say(r.note);
  }

  // ---------------------------------------------------------------- helpers
  Outputs coast() {
    out_ = Outputs{0.0f, false};
    cmd_dps_ = 0.0f;
    return out_;
  }
  Outputs fault(const char* why) {
    fault_ = why;
    state_ = State::Fault;
    say(why);
    motionReset();
    return coast();
  }
  void motionReset() {
    pi_.reset();
    shaper_.reset();
    stall_n_ = runaway_n_ = track_n_ = motion_n_ = overspeed_n_ = artefact_n_ = 0;
  }
  // Counts consecutive ticks a condition holds; true once it has held `ms`.
  bool timer(uint32_t& ticks, bool cond, uint32_t ms) const {
    ticks = cond ? ticks + 1 : 0;
    return ticks * 1000u / static_cast<uint32_t>(cfg_.fs_hz) >= ms && cond;
  }
  void say(const char* m) { msg_ = m; ++msg_seq_; }

  struct Accum { double sum = 0, sum_sq = 0; float peak = 0; uint32_t n = 0; };
  static constexpr uint32_t kTraceLen = 250;
  struct IdentStep { float settled = 0; uint32_t n = 0; bool sign_bad = false; float trace[kTraceLen] = {0}; };

  Config cfg_;
  State state_ = State::CalRest;
  EmgChannel ch_[4];
  VelocityEstimator vel_est_;
  SetpointShaper shaper_;
  VelocityPI pi_;
  Outputs out_;
  Accum acc_[4];
  IdentStep ident_steps_[4];
  IdentResult ident_;
  uint32_t ident_t_ = 0;
  int ident_phase_ = 0;
  uint32_t cal_ticks_ = 0, ticks_ = 0;
  // consecutive-tick counters for the supervisor timers
  uint32_t stall_n_ = 0, runaway_n_ = 0, track_n_ = 0, motion_n_ = 0, overspeed_n_ = 0, artefact_n_ = 0;
  uint32_t enc_err_base_ = 0;
  int32_t zero_count_ = 0, raw_count_ = 0;
  float pos_deg_ = 0.0f, vel_dps_ = 0.0f, cmd_dps_ = 0.0f;
  bool calibrated_ = false, zeroed_ = false;
  bool override_ = false;
  float override_dps_ = 0.0f;
  const char* fault_ = "";
  const char* msg_ = "";
  uint32_t msg_seq_ = 0;
};

}  // namespace orth
