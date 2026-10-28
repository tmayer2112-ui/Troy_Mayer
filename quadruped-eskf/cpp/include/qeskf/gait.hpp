// Scripted trot controller for the Go1. Mirrors python/qeskf/gait.py (TrotController, GaitParams);
// cpp/tests/test_gait.cpp checks the two against a fixture written by python/scripts/make_gait_fixture.py.
//
// Inputs are what a robot can measure or estimate: body-frame velocity, roll/pitch, body rates and trunk
// height above the feet (e.g. from the ESKF in eskf.hpp plus leg kinematics). Never yaw or position.
// Output is 12 joint position targets for the joint servos (on the Go1: kp ~ 100 N·m/rad position loops).
// Everything is fixed-size Eigen, so footTargets() / jointTargets() do not allocate and can run in a
// real-time loop. The controller holds one piece of state, the height integrator; call it at a fixed rate
// (500 Hz in the simulation) and reset() before reusing it.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <optional>

#include "qeskf/kinematics.hpp"

namespace qeskf {

struct GaitParams {
  double period = 0.40;        // s, full gait cycle
  double duty = 0.60;          // stance fraction -> four-foot support overlap each half-cycle
  double height = 0.27;        // m, nominal trunk height above the feet
  double swing_height = 0.07;  // m
  double raibert_k = 0.06;     // s, foot placement gain on velocity error
  double k_att = 1.2;          // m/rad per m lever arm, attitude correction on stance legs
  double d_att = 0.06;         // s, damping on the attitude correction
  double stand_time = 2.0;     // s of standing still before the gait clock starts
  double k_height = 1.0;       // proportional height correction
  double k_height_int = 0.002; // height integrator gain per call (per 2 ms at 500 Hz)
};

using FootTargets = Eigen::Matrix<double, 4, 3, Eigen::RowMajor>;  // trunk-frame foot targets, FR FL RR RL
using Stance = std::array<bool, 4>;

// Analytic leg IK: trunk-frame foot target -> (abduction, hip, knee). knee_sign = -1: knee bent backwards.
Eigen::Vector3d legIK(int leg, const Eigen::Vector3d& p, double knee_sign = -1.0);

// Foot position with the leg straight under the hip, at zero height (the trot's neutral point).
Eigen::Vector3d nominalFoot(int leg);

// Go1 joint ranges from models/go1/go1.xml. The controller doesn't clamp (the Python reference doesn't
// either); a robot driver should, before sending targets to the motors.
Joints clampToJointLimits(const Joints& q);

class TrotController {
 public:
  explicit TrotController(const GaitParams& p = GaitParams()) : p_(p) {}

  Eigen::Vector4d legPhase(double t) const;  // in [0, 1); stance while < duty
  Stance scheduledStance(double t) const;

  // cmd = (vx, vy, wz) in the heading frame. height: trunk height above the feet; std::nullopt
  // disables the height loop for this call. Updates the height integrator.
  FootTargets footTargets(double t, const Eigen::Vector3d& cmd, const Eigen::Vector3d& v_body,
                          const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega_body,
                          std::optional<double> height = std::nullopt);
  Joints jointTargets(double t, const Eigen::Vector3d& cmd, const Eigen::Vector3d& v_body,
                      const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega_body,
                      std::optional<double> height = std::nullopt);

  void reset() { h_int_ = 0.0; }
  const GaitParams& params() const { return p_; }

 private:
  GaitParams p_;
  double h_int_ = 0.0;
};

}  // namespace qeskf
