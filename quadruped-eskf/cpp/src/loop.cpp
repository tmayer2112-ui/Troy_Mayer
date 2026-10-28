#include "qeskf/loop.hpp"

#include <algorithm>
#include <cmath>

namespace qeskf {

void staticAlignment(const Vec3& acc_mean, const Vec3& gyro_mean, double yaw0, Mat3& R, Vec3& ba, Vec3& bg) {
  const Vec3& f = acc_mean;
  bg = gyro_mean;
  const Vec3 fn = f / f.norm();
  const double roll = std::atan2(fn(1), fn(2));
  const double pitch = std::atan2(-fn(0), std::sqrt(fn(1) * fn(1) + fn(2) * fn(2)));
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw0), sy = std::sin(yaw0);
  Mat3 Rz, Ry, Rx;
  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
  R = Rz * Ry * Rx;
  ba = Vec3(0.0, 0.0, f.norm() - 9.80665);  // |f| = g + b_z when level (first order)
}

CoreMat initialCovariance(const ImuNoise& imu, int n_cal, double p_sigma, double v_sigma, double yaw_sigma) {
  const double g = 9.80665;
  CoreMat P = CoreMat::Zero();
  P.block<3, 3>(kIP, kIP) = Mat3::Identity() * p_sigma * p_sigma;
  P.block<3, 3>(kIV, kIV) = Mat3::Identity() * v_sigma * v_sigma;
  const double sb = imu.acc_turn_on_bias;
  Mat3 A;  // db_a -> dtheta: leveling absorbs the horizontal accel bias into tilt
  A << 0.0, -1.0 / g, 0.0, 1.0 / g, 0.0, 0.0, 0.0, 0.0, 0.0;
  Mat3 Sb = Vec3(sb * sb, sb * sb, 0.0).asDiagonal();
  const double s_avg_a = imu.acc_noise_density * std::sqrt(imu.rate_hz) / std::sqrt(static_cast<double>(n_cal));
  const double s_avg_g = imu.gyro_noise_density * std::sqrt(imu.rate_hz) / std::sqrt(static_cast<double>(n_cal));
  Sb(2, 2) = s_avg_a * s_avg_a + (imu.acc_rrw * imu.acc_rrw) * 10.0;
  P.block<3, 3>(kIBA, kIBA) = Sb;
  const double st = s_avg_a / g;
  P.block<3, 3>(kITH, kITH) = A * Sb * A.transpose() + Mat3(Vec3(st * st, st * st, yaw_sigma * yaw_sigma).asDiagonal());
  P.block<3, 3>(kITH, kIBA) = A * Sb;
  P.block<3, 3>(kIBA, kITH) = (A * Sb).transpose();
  P.block<3, 3>(kIBG, kIBG) = Mat3::Identity() * s_avg_g * s_avg_g;
  return P;
}

Quat rotToQuat(const Mat3& R) {
  const double tr = R.trace();
  double w, x, y, z;
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    w = 0.25 * s; x = (R(2, 1) - R(1, 2)) / s; y = (R(0, 2) - R(2, 0)) / s; z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
    w = (R(2, 1) - R(1, 2)) / s; x = 0.25 * s; y = (R(0, 1) + R(1, 0)) / s; z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
    w = (R(0, 2) - R(2, 0)) / s; x = (R(0, 1) + R(1, 0)) / s; y = 0.25 * s; z = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
    w = (R(1, 0) - R(0, 1)) / s; x = (R(0, 2) + R(2, 0)) / s; y = (R(1, 2) + R(2, 1)) / s; z = 0.25 * s;
  }
  return w >= 0 ? Quat(w, x, y, z) : Quat(-w, -x, -y, -z);
}

Vec3 rpyFromRot(const Mat3& R) {
  return {std::atan2(R(2, 1), R(2, 2)), -std::asin(std::clamp(R(2, 0), -1.0, 1.0)), std::atan2(R(1, 0), R(0, 0))};
}

double feetHeight(const Mat3& R, const Joints& qenc) {
  std::array<double, 4> z;
  for (int i = 0; i < 4; ++i) z[i] = (R * footPosition(i, qenc.segment<3>(3 * i)))(2);
  std::sort(z.begin(), z.end());
  return -0.5 * (z[0] + z[1]);
}

EstimatorController::EstimatorController(const LoopConfig& cfg) : cfg_(cfg), ctrl_(cfg.gait) {
  k0_ = std::lround(cfg_.t_start / cfg_.dt);
  k_cal0_ = static_cast<long>(cfg_.cal_start / cfg_.dt);   // int() truncation, as in python
  adv_ = std::lround(cfg_.liftoff_advance / cfg_.dt);
}

const Joints& EstimatorController::step(const Vec3& acc, const Vec3& gyro, const Joints& qenc,
                                        const Contacts& contact, const Vec3& cmd) {
  const long k = k_++;
  const double t = k * cfg_.dt;

  // 1. the IMU sample from tick k-1: calibration window, or propagation to t_k
  if (k >= 1) {
    const long j = k - 1;
    if (j >= k_cal0_ && j < k0_) {
      acc_sum_ += acc;
      gyro_sum_ += gyro;
      ++n_cal_;
    }
    if (filter_) filter_->predict(acc, gyro, cfg_.dt);
  }

  // 2. start the filter from the standing window
  if (k == k0_) {
    Mat3 R0;
    Vec3 ba0, bg0;
    staticAlignment(acc_sum_ / n_cal_, gyro_sum_ / n_cal_, cfg_.yaw0, R0, ba0, bg0);
    filter_.emplace(cfg_.eskf, cfg_.p0, Vec3::Zero(), rotToQuat(R0), ba0, bg0,
                    Eigen::MatrixXd(initialCovariance(cfg_.imu, n_cal_)));
  }

  // 3. contact bookkeeping + kinematic update at t_k
  if (filter_) {
    Contacts c = contact;
    if (cfg_.use_schedule) {
      const Stance s_now = ctrl_.scheduledStance(t), s_adv = ctrl_.scheduledStance(t + adv_ * cfg_.dt);
      for (int i = 0; i < 4; ++i) c[i] = c[i] && s_now[i] && s_adv[i];
    }
    const std::vector<int> fresh = filter_->handleContacts(c, qenc);
    filter_->update(qenc, fresh);   // a foot augmented from z_k must not also be updated with z_k
  }

  // 4. controller
  if (k % cfg_.ctrl_decim == 0) {
    if (!filter_) {   // standing calibration: no attitude estimate yet -> fixed stance
      fb_ = Feedback{};
      fb_.height = feetHeight(Mat3::Identity(), qenc);
    } else {
      const Mat3 Rh = quatToRot(filter_->q());
      fb_.v_body = Rh.transpose() * filter_->v();
      fb_.rpy = rpyFromRot(Rh);
      fb_.omega_body = gyro - filter_->bg();   // latest gyro sample, bias-corrected
      fb_.height = feetHeight(Rh, qenc);
    }
    q_cmd_ = ctrl_.jointTargets(t, cmd, fb_.v_body, fb_.rpy, fb_.omega_body, fb_.height);
  }
  return q_cmd_;
}

}  // namespace qeskf
