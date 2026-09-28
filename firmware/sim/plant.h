/* Plant model for testing the orthosis controller on a laptop.
 *
 *   BTS7960 --> DC motor --> 5.2:1 planetary --> 4 mm screw --> cable --> 47 mm pulley --> joint
 *                  |                                                                      |
 *              encoder (28 counts/rev, imperfect edge spacing)          forearm: inertia + gravity
 *
 * Motor constants are the goBILDA 5203 (5.2:1, 1150 RPM) datasheet values at 12 V:
 * 9.2 A stall, 0.25 A no-load, 7.9 kg-cm stall torque at the output.
 *   R  = 12 V / 9.2 A                       = 1.30 ohm
 *   Ke = (12 - 0.25 * 1.30) / 626 rad/s    = 0.0186 V s/rad  (= Kt)
 *   gearbox efficiency = 0.775 N m / (0.1716 N m * 5.2) = 0.87
 * Rotor inertia and the limb numbers are estimates; tests sweep them.
 *
 * The electrical time constant (L/R, well under 1 ms) is ignored: current is
 * taken as quasi-static, which is accurate at a 1 kHz control rate.
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace sim {

constexpr double kPi = 3.14159265358979323846;

struct PlantParams {
  double vbus = 12.0;
  double R = 1.30;
  double Ke = 0.0186;                 // = Kt in SI units
  double J_rotor = 3.0e-6;            // kg m^2, estimate
  double J_gear = 0.5e-6;             // gearbox + screw reflected to the motor, estimate
  double gear = 5.2;
  double screw_pitch_m = 0.004;
  double pulley_r_m = 0.0235;
  double eta = 0.87 * 0.90;           // gearbox x ball screw
  double tau_coulomb = 0.25 * 0.0186; // N m at the motor: the no-load current's torque
  double stiction_ratio = 1.3;
  double b_visc = 2.0e-7;             // N m s/rad at the motor
  // limb + moving orthosis parts about the joint
  double limb_I = 0.085;              // kg m^2
  double limb_mgr = 3.5;              // N m, peak gravity torque (2 kg at 18 cm)
  double elbow_at_zero_deg = 45.0;    // elbow angle when the joint reads 0
  // encoder
  double counts_per_rev = 28.0;
  double edge_err[4] = {0.0, 0.12, 0.0, -0.10};   // edge placement error, fraction of a count
  bool encoder_reversed = false;
  // a hard stop, for stall tests
  bool hard_stop = false;
  double hard_stop_deg = 1e9;

  double N() const { return gear * 2.0 * kPi * pulley_r_m / screw_pitch_m; }  // motor rad / joint rad
  double J() const { return J_rotor + J_gear + limb_I / (N() * N()); }
};

struct Edge { uint8_t ab; double t; };

class Plant {
 public:
  explicit Plant(const PlantParams& p, double joint_deg0 = 0.0) : p_(p) {
    theta_m_ = joint_deg0 * kPi / 180.0 * p_.N();
    count_ = countAt(encPos());
  }

  // Advance by dt with the driver in a given mode. enable=false -> coast (i = 0).
  void step(double duty, bool enable, double dt, double t_now, double tau_ext_joint = 0.0) {
    const double N = p_.N();
    const double i = enable ? (duty * p_.vbus - p_.Ke * omega_m_) / p_.R : 0.0;
    current_ = i;
    const double tau_m = p_.Ke * i;
    const double elbow = (jointDeg() + p_.elbow_at_zero_deg) * kPi / 180.0;
    const double tau_load_joint = -gravity_scale_ * p_.limb_mgr * std::sin(elbow) + tau_ext_joint;  // + = toward +theta
    const double tau_load = tau_load_joint / N;
    // transmission loss behaves like friction proportional to the torque carried
    const double tau_f = p_.tau_coulomb + std::fabs(tau_load) * (1.0 / p_.eta - 1.0);
    const double net = tau_m + tau_load;
    double acc;
    if (std::fabs(omega_m_) < 1e-3) {
      if (std::fabs(net) <= tau_f * p_.stiction_ratio) { omega_m_ = 0.0; acc = 0.0; }
      else acc = (net - std::copysign(tau_f, net)) / p_.J();
    } else {
      acc = (net - std::copysign(tau_f, omega_m_) - p_.b_visc * omega_m_) / p_.J();
    }
    const double w_old = omega_m_;
    omega_m_ += acc * dt;
    if (w_old != 0.0 && (w_old > 0) != (omega_m_ > 0) && std::fabs(net) <= tau_f * p_.stiction_ratio)
      omega_m_ = 0.0;   // friction can stop the motor, not reverse it
    theta_m_ += omega_m_ * dt;
    if (p_.hard_stop && jointDeg() >= p_.hard_stop_deg && omega_m_ > 0.0) {
      theta_m_ = p_.hard_stop_deg * kPi / 180.0 * N;
      omega_m_ = 0.0;
    }
    emitEdges(t_now + dt);
  }

  // Back-drive the joint by hand (motor disarmed): set the angle, emit the edges.
  void handSet(double joint_deg, double t) {
    theta_m_ = joint_deg * kPi / 180.0 * p_.N();
    omega_m_ = 0.0;
    emitEdges(t);
  }

  // 0 = the user is supporting the limb (or it is not strapped in), 1 = full weight.
  void setGravityScale(double g) { gravity_scale_ = g; }

  double jointDeg() const { return theta_m_ / p_.N() * 180.0 / kPi; }
  double jointDps() const { return omega_m_ / p_.N() * 180.0 / kPi; }
  double current() const { return current_; }
  const PlantParams& params() const { return p_; }
  PlantParams& params() { return p_; }

  // Edges generated since the last call, oldest first.
  std::vector<Edge> takeEdges() { std::vector<Edge> e; e.swap(edges_); return e; }
  uint8_t ab() const { return abOf(count_); }

 private:
  double encPos() const {   // position in encoder counts, continuous
    const double x = theta_m_ / (2.0 * kPi) * p_.counts_per_rev;
    return p_.encoder_reversed ? -x : x;
  }
  static int mod4(long k) { return static_cast<int>(((k % 4) + 4) % 4); }
  double edgeAt(long k) const { return k + p_.edge_err[mod4(k)]; }
  long countAt(double x) const {
    long c = static_cast<long>(std::floor(x));
    while (edgeAt(c + 1) <= x) ++c;
    while (edgeAt(c) > x) --c;
    return c;
  }
  // Gray sequence that the firmware's quadrature table decodes as +1 per step.
  static uint8_t abOf(long c) { static const uint8_t seq[4] = {0, 2, 3, 1}; return seq[mod4(c)]; }

  void emitEdges(double t) {
    const long c = countAt(encPos());
    while (count_ < c) { ++count_; edges_.push_back({abOf(count_), t}); }
    while (count_ > c) { --count_; edges_.push_back({abOf(count_), t}); }
  }

  PlantParams p_;
  double theta_m_ = 0.0, omega_m_ = 0.0, current_ = 0.0, gravity_scale_ = 1.0;
  long count_ = 0;
  std::vector<Edge> edges_;
};

}  // namespace sim
