// Module 9: the speed loop in C++17, header-only like firmware/emg_orthosis/orthosis_core.h. REFERENCE SOLUTION.
// Real project: orthosis_core.h, Config::kp() / ki() (lines 151-152) and VelocityPI::step (line 432).
// It uses float, because the ESP32-S3's FPU is single precision; this port uses double so it can match
// the Python reference to rounding error.
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rebuild {

struct Gains {
  double kp;  // duty per deg/s
  double ki;  // duty per deg
};

// IMC tuning for K / (tau s + 1): Kp = tau / (K lambda), Ki = 1 / (K lambda).   (Worksheet 9.1)
inline Gains imcGains(double K, double tau, double lambda) {
  // TODO (1 line): return {kp, ki};
  // --- solution ---
  return {tau / (K * lambda), 1.0 / (K * lambda)};
  // --- end ---
}

// Plant inversion: (ref + tau dref) / K, plus `friction` with ref's sign when |ref| > 0.5.
inline double feedforward(double ref, double dref, double K, double tau, double friction) {
  // TODO (3 lines): std::fabs, std::copysign
  // --- solution ---
  double ff = (ref + tau * dref) / K;
  if (std::fabs(ref) > 0.5) ff += std::copysign(friction, ref);
  return ff;
  // --- end ---
}

class VelocityPI {
 public:
  // The firmware's defaults (orthosis_core.h, Config): datasheet model, conservative tier.
  double K = 187.0, tau = 0.020, lambda = 0.050, friction = 0.05, duty_max = 0.60;
  bool antiwindup = true;
  double kp = 0.0, ki = 0.0;
  double integ = 0.0;     // the integral term, already in duty
  double prev_ref = 0.0;

  VelocityPI() { retune(); }
  void retune() {
    const Gains g = imcGains(K, tau, lambda);
    kp = g.kp;
    ki = g.ki;
  }

  // One 1 ms tick: the same steps, in the same order, as your Python VelocityPI.step (Module 6).
  //   e = ref - meas; dref from prev_ref; out = clamp(ff + kp e + integ)
  //   integrate unless antiwindup and (saturated and pushing further, or ramping and charging);
  //   then clamp integ.   (Worksheet 9.2)
  double step(double ref, double meas, double dt, bool ramping = false) {
    // TODO (about 10 lines). std::clamp(v, lo, hi) is in <algorithm>.
    // --- solution ---
    const double e = ref - meas;
    const double dref = (ref - prev_ref) / dt;
    prev_ref = ref;
    const double unsat = feedforward(ref, dref, K, tau, friction) + kp * e + integ;
    const double out = std::clamp(unsat, -duty_max, duty_max);
    const bool pushing_further = unsat != out && (e > 0.0) == (unsat > 0.0);
    const bool charging = ramping && (e > 0.0) == (ref > 0.0);
    if (!(antiwindup && (pushing_further || charging))) integ += ki * e * dt;
    integ = std::clamp(integ, -duty_max, duty_max);
    return out;
    // --- end ---
  }
};

}  // namespace rebuild
