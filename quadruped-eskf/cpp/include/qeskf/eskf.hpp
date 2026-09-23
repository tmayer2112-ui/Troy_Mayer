// Contact-aided error-state EKF. Line-for-line port of python/qeskf/eskf.py.
//
// Error state: [dp, dv, dtheta, dba, dbg, dd_1 .. dd_m], dim 15 + 3m (m = feet in contact).
// dtheta is a local (body-frame) rotation error. Feet are appended on touchdown and
// removed on liftoff, so P changes size at runtime (Eigen::MatrixXd).
//
// Difference from the Python reference (on purpose): predict() exploits the structure of F.
// F is identity outside the 15x15 core block, so
//   P_cc <- F_c P_cc F_c^T,  P_cf <- F_c P_cf,  P_ff unchanged
// which is O(15^2 n) instead of O(n^3). The parity test checks it gives the same answer.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

#include "qeskf/kinematics.hpp"
#include "qeskf/so3.hpp"

namespace qeskf {

constexpr int kNB = 15;
constexpr int kIP = 0, kIV = 3, kITH = 6, kIBA = 9, kIBG = 12;
inline const Vec3 kGravity(0.0, 0.0, -9.80665);

struct EskfParams {
  double acc_noise = 175e-6 * 9.80665;      // m/s^2/sqrt(Hz)
  double gyro_noise = 0.014 * M_PI / 180.0;  // rad/s/sqrt(Hz)
  double acc_rw = 1e-4;                      // m/s^3/sqrt(Hz)
  double gyro_rw = 1e-6;                     // rad/s^2/sqrt(Hz)
  double foot_noise = 0.01;                  // m/sqrt(s)
  double kin_noise = 0.003;                  // m
  double enc_lsb = 2.0 * M_PI / 16384.0;     // rad
  double gate_chi2 = 0.0;                    // 0 disables
  int settle_steps = 0;
};

struct Snapshot {
  Vec3 p, v;
  Quat q;
  Vec3 ba, bg;
  Eigen::Matrix<double, kNB, 1> Pdiag;
  int nfeet;
};

using Contacts = std::array<bool, 4>;
using CoreMat = Eigen::Matrix<double, kNB, kNB>;

class Eskf {
 public:
  Eskf(const EskfParams& prm, const Vec3& p, const Vec3& v, const Quat& q, const Vec3& ba,
       const Vec3& bg, const Eigen::MatrixXd& P0);

  // --- the loop ---
  Snapshot step(const Vec3& acc, const Vec3& gyro, const Joints& qenc, const Contacts& contact,
                double dt, bool use_kinematics = true);

  // --- building blocks (public for tests) ---
  void propagateNominal(const Vec3& acc, const Vec3& gyro, double dt);
  CoreMat coreTransition(const Vec3& acc, const Vec3& gyro, double dt) const;
  Eigen::MatrixXd errorTransition(const Vec3& acc, const Vec3& gyro, double dt) const;
  void predict(const Vec3& acc, const Vec3& gyro, double dt);
  void augment(int leg, const Joints& qenc);
  void marginalize(int leg);
  std::vector<int> handleContacts(const Contacts& contact, const Joints& qenc);
  void measurement(const std::vector<int>& legs, const Joints& qenc, Eigen::VectorXd& r,
                   Eigen::MatrixXd& H, Eigen::MatrixXd& Rn) const;
  void update(const Joints& qenc, const std::vector<int>& skip = {});
  void inject(const Eigen::VectorXd& dx);
  Eigen::Matrix3d kinCov(int leg, const Joints& qenc) const;

  // --- state access ---
  int dim() const { return static_cast<int>(P_.rows()); }
  const Eigen::MatrixXd& P() const { return P_; }
  Eigen::MatrixXd& mutableP() { return P_; }
  const Vec3& p() const { return p_; }
  const Vec3& v() const { return v_; }
  const Quat& q() const { return q_; }
  const Vec3& ba() const { return ba_; }
  const Vec3& bg() const { return bg_; }
  const std::vector<int>& order() const { return order_; }
  const Vec3& foot(int leg) const { return feet_[leg]; }
  void setState(const Vec3& p, const Vec3& v, const Quat& q, const Vec3& ba, const Vec3& bg) {
    p_ = p; v_ = v; q_ = q; ba_ = ba; bg_ = bg;
  }
  void setFoot(int leg, const Vec3& d) { feet_[leg] = d; }
  int footIndex(int leg) const;
  bool inContact(int leg) const;
  const EskfParams& params() const { return prm_; }

  struct Stats { long updates = 0, gated = 0, augment = 0, marginalize = 0; } stats;

 private:
  EskfParams prm_;
  Vec3 p_, v_;
  Quat q_;
  Vec3 ba_, bg_;
  std::array<Vec3, 4> feet_;
  std::vector<int> order_;       // leg ids in error-state order
  std::array<int, 4> contact_count_{0, 0, 0, 0};
  Eigen::MatrixXd P_;
};

}  // namespace qeskf
