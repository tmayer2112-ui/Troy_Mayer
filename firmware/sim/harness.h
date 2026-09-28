/* Closed-loop harness: the real orthosis_core Controller driving the Plant.
 *
 * The plant is integrated at 50 kHz; the controller ticks at 1 kHz exactly as
 * on the ESP32 (read ADC + encoder snapshot, step, write the driver). Encoder
 * edges are timestamped inside the plant step and fed through the same
 * EdgeTracker::onChange the ISR calls.
 */
#pragma once

#include <functional>
#include <random>

#include "../emg_orthosis/orthosis_core.h"
#include "plant.h"

namespace sim {

// EMG amplitude (ADC counts RMS) per channel as a function of time.
using EmgProfile = std::function<void(double t, double amp[4])>;

struct Sample {
  double t, pos, vel, vel_est, ref, cmd, duty, current;
  orth::State state;
};

class Harness {
 public:
  Harness(const orth::Config& cfg, const PlantParams& pp, double joint_deg0 = 0.0)
      : ctl(cfg), plant(pp, joint_deg0), rng_(12345) {
    enc_.reset(plant.ab());
    uint16_t adc[4] = {2048, 2048, 2048, 2048};
    ctl.prime(adc);
  }

  // Rest EMG amplitude and full-contraction amplitude used by calibrate().
  double rest_amp = 6.0, mvc_amp = 350.0;
  bool kill_closed = true, mot_ok = true;
  bool railed[4] = {false, false, false, false};
  double tau_ext_joint = 0.0;
  // While the motor is not powered the user holds their own arm; once the
  // driver is enabled the limb's full weight is on the orthosis.
  bool user_supports_when_disabled = true;
  EmgProfile emg;   // empty = every channel at rest_amp (no captured `this`, so Harness copies safely)
  // Optional: replace channel values with recorded ADC samples (return false = use the synthetic value).
  std::function<bool(double t, int ch, uint16_t& adc)> adc_override;
  std::vector<Sample> log;
  bool record = false;

  // Run for `seconds`, issuing `req` on the first tick.
  void run(double seconds, orth::Request req = orth::Request::None) {
    const int ticks = static_cast<int>(seconds * 1000.0 + 0.5);
    for (int k = 0; k < ticks; ++k) { tick(req); req = orth::Request::None; }
  }

  // Full rest / flexor / extensor calibration with synthetic EMG.
  void calibrate() {
    auto saved = emg;
    const double t0 = t_;
    emg = [this, t0](double t, double a[4]) {
      const double s = t - t0;
      for (int i = 0; i < 4; ++i) a[i] = rest_amp;
      if (s > 4.2 && s < 8.0) a[0] = mvc_amp;
      if (s > 8.2 && s < 12.0) a[1] = mvc_amp;
    };
    run(0.001, orth::Request::Calibrate);
    run(12.5);
    emg = saved;
  }

  void tick(orth::Request req) {
    // edges from a hand move (Plant::handSet), then 50 plant steps of 20 us
    for (const Edge& e : plant.takeEdges()) enc_.onChange(e.ab, us(e.t));
    const double dt = 20e-6;
    plant.setGravityScale(user_supports_when_disabled && !out_.enable ? 0.0 : 1.0);
    for (int s = 0; s < 50; ++s) {
      plant.step(out_.duty, out_.enable, dt, t_, tau_ext_joint);
      for (const Edge& e : plant.takeEdges()) enc_.onChange(e.ab, us(e.t));
      t_ += dt;
    }
    // sensors
    orth::Inputs in;
    double amp[4] = {rest_amp, rest_amp, rest_amp, rest_amp};
    if (emg) emg(t_, amp);
    std::normal_distribution<double> n(0.0, 1.0);
    for (int i = 0; i < 4; ++i) {
      double x = 2048.0 + amp[i] * n(rng_);
      if (railed[i]) x = 4095.0;
      in.adc[i] = static_cast<uint16_t>(std::lround(std::fmin(std::fmax(x, 0.0), 4095.0)));
      if (adc_override) {
        uint16_t v;
        if (adc_override(t_, i, v)) in.adc[i] = v;
      }
    }
    in.enc = enc_;
    in.now_us = us(t_);
    in.kill_closed = kill_closed;
    in.mot_ok = mot_ok;
    out_ = ctl.step(in, req);
    if (record)
      log.push_back({t_, plant.jointDeg(), plant.jointDps(), ctl.jointDps(), ctl.speedRef(),
                     ctl.speedCommand(), out_.duty, plant.current(), ctl.state()});
  }

  double t() const { return t_; }
  const orth::Outputs& out() const { return out_; }

  orth::Controller ctl;
  Plant plant;

 private:
  static uint32_t us(double t) { return static_cast<uint32_t>(static_cast<uint64_t>(t * 1e6)); }
  orth::EdgeTracker enc_;
  orth::Outputs out_;
  double t_ = 0.0;
  std::mt19937 rng_;
};

// Move the disarmed joint by hand at `dps`, the way you would back-drive it on the bench.
inline void handMove(Harness& h, double target_deg, double dps = 30.0) {
  while (std::fabs(h.plant.jointDeg() - target_deg) > 1e-6) {
    const double step = std::copysign(std::fmin(dps * 1e-3, std::fabs(target_deg - h.plant.jointDeg())),
                                      target_deg - h.plant.jointDeg());
    h.plant.handSet(h.plant.jointDeg() + step, h.t());
    h.run(0.001);
  }
}

// The bench procedure: calibrate, zero at the lower limit, hand-move to
// `start_deg`, arm, and hand the speed command to the test.
inline void calibrateZeroArm(Harness& h, double start_deg = 15.0) {
  h.calibrate();
  h.run(0.01, orth::Request::Zero);
  handMove(h, h.plant.jointDeg() + start_deg);
  h.run(0.3);
  h.ctl.setSpeedOverride(true, 0.0);
  h.run(0.01, orth::Request::Arm);
}

// The bench identification ('j') on this plant with the limb out, exactly as
// the README's bring-up does it. Returns the config with the measured model
// loaded and the fast tier unlocked (or `base` unchanged if it failed).
inline orth::Config identifiedConfig(orth::Config base, PlantParams pp) {
  pp.limb_mgr = 0.0;
  base.joint_max_deg = std::max(base.joint_max_deg, 60.0f);
  Harness h(base, pp);
  h.calibrate();
  h.run(0.01, orth::Request::Zero);
  handMove(h, 30.0);
  h.run(0.2);
  h.run(0.01, orth::Request::Identify);
  h.run(3.0);
  orth::Config c = h.ctl.config();
  c.joint_max_deg = base.joint_max_deg;
  return c;
}

}  // namespace sim
