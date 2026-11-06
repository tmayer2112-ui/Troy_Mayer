// The estimator driving the trot controller, in C++: one call per 1 kHz control tick.
// Mirrors the estimate-feedback path of python/qeskf/closed_loop.py (run(feedback="estimate"));
// cpp/tests/test_loop.cpp replays a Python closed-loop run through it and compares the joint targets.
//
// Per tick k (t_k = k * dt), step() does, in this order:
//   1. take the IMU sample measured over the previous tick (k-1): add it to the standing calibration
//      window, or, once the filter runs, predict t_{k-1} -> t_k with it;
//   2. at t = t_start: level + gyro-bias calibration from the standing window (staticAlignment) and
//      the tilt/accel-bias-correlated P0 (initialCovariance), then start the filter;
//   3. contact = sensed contact (from the previous tick) AND the gait schedule, ended `liftoff_advance`
//      early; touchdown/liftoff bookkeeping, then the kinematic update at t_k;
//   4. every `ctrl_decim` ticks: the trot controller reads body-frame velocity, roll/pitch, the
//      bias-corrected gyro and trunk height from leg kinematics, and sets 12 joint targets (held between
//      controller ticks). Before t_start there is no attitude estimate, so it holds a fixed stance:
//      zero velocity, level, zero rates, height from leg kinematics.
// The controller never uses position or yaw, which the estimator can't observe.
//
// What a robot driver adds around it: reading the sensors, the motors' position loop, joint-limit
// clamping (clampToJointLimits), and safety (fall detection, e-stop, stand-up / lie-down).
#pragma once

#include <array>
#include <optional>

#include "qeskf/eskf.hpp"
#include "qeskf/gait.hpp"

namespace qeskf {

// IMU data-sheet terms the start-up covariance needs (python/qeskf/imu_model.py ImuSpec; BMI088 defaults).
struct ImuNoise {
  double rate_hz = 1000.0;
  double acc_noise_density = 175e-6 * 9.80665;   // m/s^2/sqrt(Hz)
  double gyro_noise_density = 0.014 * M_PI / 180.0;  // rad/s/sqrt(Hz)
  double acc_turn_on_bias = 20e-3 * 9.80665;     // m/s^2, 1 sigma
  double acc_rrw = 1.698561605204533e-4;         // m/s^3/sqrt(Hz), from the bias instability
};

struct LoopConfig {
  EskfParams eskf;            // filter parameters (results/tuned_params.json for the simulated Go1)
  ImuNoise imu;
  GaitParams gait;
  double dt = 0.001;          // s, IMU / control tick
  double cal_start = 0.5;     // s, standing calibration window [cal_start, t_start)
  double t_start = 2.0;       // s, the filter starts here
  int ctrl_decim = 2;         // controller runs every ctrl_decim ticks (500 Hz)
  bool use_schedule = true;   // AND sensed contact with the gait schedule
  double liftoff_advance = 0.005;  // s, end the scheduled stance this early
  Vec3 p0 = Vec3::Zero();     // initial position of the navigation frame (unobservable; any value)
  double yaw0 = 0.0;          // initial yaw (unobservable; any value)
};

// Roll/pitch from the mean specific force, gyro bias = mean rate, accel z-bias from |f| - g.
void staticAlignment(const Vec3& acc_mean, const Vec3& gyro_mean, double yaw0, Mat3& R, Vec3& ba, Vec3& bg);
// P0 with the tilt / horizontal-accel-bias correlation explicit (python/qeskf/eskf.py initial_covariance).
CoreMat initialCovariance(const ImuNoise& imu, int n_cal, double p_sigma = 1e-3, double v_sigma = 0.01,
                          double yaw_sigma = 1e-3);
Quat rotToQuat(const Mat3& R);
Vec3 rpyFromRot(const Mat3& R);
// Trunk height above the two lowest feet, from leg kinematics and an attitude.
double feetHeight(const Mat3& R, const Joints& qenc);

// What the controller was fed on the last controller tick (for logging / checks).
struct Feedback {
  Vec3 v_body = Vec3::Zero(), rpy = Vec3::Zero(), omega_body = Vec3::Zero();
  double height = 0.0;
};

class EstimatorController {
 public:
  explicit EstimatorController(const LoopConfig& cfg = LoopConfig());

  // One tick. acc, gyro: the IMU sample measured over the previous tick (ignored on the first call).
  // qenc: encoder angles now. contact: sensed foot contact from the previous tick. cmd: (vx, vy, wz).
  // Returns the joint position targets to send to the servos.
  const Joints& step(const Vec3& acc, const Vec3& gyro, const Joints& qenc, const Contacts& contact,
                     const Vec3& cmd);

  bool running() const { return filter_.has_value(); }
  const Eskf* filter() const { return filter_ ? &*filter_ : nullptr; }
  const Feedback& feedback() const { return fb_; }
  const Joints& targets() const { return q_cmd_; }
  long tick() const { return k_; }
  double time() const { return k_ * cfg_.dt; }

 private:
  LoopConfig cfg_;
  TrotController ctrl_;
  std::optional<Eskf> filter_;
  long k_ = 0;                  // index of the next tick
  long k0_, k_cal0_, adv_;
  Vec3 acc_sum_ = Vec3::Zero(), gyro_sum_ = Vec3::Zero();
  int n_cal_ = 0;
  Feedback fb_;
  Joints q_cmd_ = Joints::Zero();
};

}  // namespace qeskf
