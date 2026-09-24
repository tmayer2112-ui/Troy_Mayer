#include "qeskf/eskf.hpp"

#include <algorithm>
#include <cmath>

namespace qeskf {

Eskf::Eskf(const EskfParams& prm, const Vec3& p, const Vec3& v, const Quat& q, const Vec3& ba,
           const Vec3& bg, const Eigen::MatrixXd& P0)
    : prm_(prm), p_(p), v_(v), q_(q), ba_(ba), bg_(bg), P_(P0) {
  for (auto& f : feet_) f.setZero();
}

int Eskf::footIndex(int leg) const {
  const auto it = std::find(order_.begin(), order_.end(), leg);
  return kNB + 3 * static_cast<int>(it - order_.begin());
}

bool Eskf::inContact(int leg) const {
  return std::find(order_.begin(), order_.end(), leg) != order_.end();
}

// ---------------------------------------------------------------- predict
void Eskf::propagateNominal(const Vec3& acc, const Vec3& gyro, double dt) {
  const Mat3 R = quatToRot(q_);
  const Vec3 a_w = R * (acc - ba_) + kGravity;
  const Vec3 w = gyro - bg_;
  p_ = p_ + v_ * dt + 0.5 * a_w * dt * dt;
  v_ = v_ + a_w * dt;
  q_ = quatMul(q_, quatExp(w * dt));
  q_.coeffs() /= q_.norm();
}

CoreMat Eskf::coreTransition(const Vec3& acc, const Vec3& gyro, double dt) const {
  const Mat3 R = quatToRot(q_);
  const Vec3 a = acc - ba_;
  const Vec3 w = gyro - bg_;
  const Mat3 RA = R * skew(a);
  CoreMat F = CoreMat::Identity();
  F.block<3, 3>(kIP, kIV) = Mat3::Identity() * dt;
  F.block<3, 3>(kIP, kITH) = -0.5 * RA * dt * dt;
  F.block<3, 3>(kIP, kIBA) = -0.5 * R * dt * dt;
  F.block<3, 3>(kIV, kITH) = -RA * dt;
  F.block<3, 3>(kIV, kIBA) = -R * dt;
  F.block<3, 3>(kITH, kITH) = expSO3(-w * dt);
  F.block<3, 3>(kITH, kIBG) = -rightJacobian(w * dt) * dt;
  return F;
}

Eigen::MatrixXd Eskf::errorTransition(const Vec3& acc, const Vec3& gyro, double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(dim(), dim());
  F.topLeftCorner<kNB, kNB>() = coreTransition(acc, gyro, dt);
  return F;
}

void Eskf::predict(const Vec3& acc, const Vec3& gyro, double dt) {
  const CoreMat Fc = coreTransition(acc, gyro, dt);
  propagateNominal(acc, gyro, dt);
  const int n = dim();
  const int nf = n - kNB;
  // Structured F P F^T: F = blkdiag(Fc, I).
  P_.topLeftCorner<kNB, kNB>() = Fc * P_.topLeftCorner<kNB, kNB>() * Fc.transpose();
  if (nf > 0) {
    P_.topRightCorner(kNB, nf) = Fc * P_.topRightCorner(kNB, nf);
    P_.bottomLeftCorner(nf, kNB) = P_.topRightCorner(kNB, nf).transpose();
  }
  // Q
  P_.block<3, 3>(kIV, kIV).diagonal().array() += prm_.acc_noise * prm_.acc_noise * dt;
  P_.block<3, 3>(kITH, kITH).diagonal().array() += prm_.gyro_noise * prm_.gyro_noise * dt;
  P_.block<3, 3>(kIBA, kIBA).diagonal().array() += prm_.acc_rw * prm_.acc_rw * dt;
  P_.block<3, 3>(kIBG, kIBG).diagonal().array() += prm_.gyro_rw * prm_.gyro_rw * dt;
  for (int i = kNB; i < n; ++i) P_(i, i) += prm_.foot_noise * prm_.foot_noise * dt;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

// ---------------------------------------------------------------- contacts
Eigen::Matrix3d Eskf::kinCov(int leg, const Joints& qenc) const {
  const Eigen::Matrix3d Jq = footJacobian(leg, legJoints(qenc, leg));
  const double s2 = prm_.enc_lsb * prm_.enc_lsb / 12.0;
  return s2 * Jq * Jq.transpose() + Eigen::Matrix3d::Identity() * prm_.kin_noise * prm_.kin_noise;
}

void Eskf::augment(int leg, const Joints& qenc) {
  const Mat3 R = quatToRot(q_);
  const Vec3 f = footPosition(leg, legJoints(qenc, leg));
  const int n = dim();
  // J = d(new foot)/d(dx): only dp and dtheta columns are non-zero.
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n);
  J.block<3, 3>(0, kIP) = Mat3::Identity();
  J.block<3, 3>(0, kITH) = -R * skew(f);
  const Mat3 Rk = R * kinCov(leg, qenc) * R.transpose();
  const Eigen::MatrixXd PJt = P_ * J.transpose();
  Eigen::MatrixXd Pn(n + 3, n + 3);
  Pn.topLeftCorner(n, n) = P_;
  Pn.topRightCorner(n, 3) = PJt;
  Pn.bottomLeftCorner(3, n) = PJt.transpose();
  Pn.bottomRightCorner(3, 3) = J * PJt + Rk;
  P_ = 0.5 * (Pn + Pn.transpose());
  feet_[leg] = p_ + R * f;
  order_.push_back(leg);
  ++stats.augment;
}

void Eskf::marginalize(int leg) {
  const int i0 = footIndex(leg);
  const int n = dim();
  const int tail = n - i0 - 3;
  Eigen::MatrixXd Pn(n - 3, n - 3);
  Pn.topLeftCorner(i0, i0) = P_.topLeftCorner(i0, i0);
  Pn.topRightCorner(i0, tail) = P_.topRightCorner(i0, tail);
  Pn.bottomLeftCorner(tail, i0) = P_.bottomLeftCorner(tail, i0);
  Pn.bottomRightCorner(tail, tail) = P_.bottomRightCorner(tail, tail);
  P_ = std::move(Pn);
  order_.erase(std::find(order_.begin(), order_.end(), leg));
  ++stats.marginalize;
}

std::vector<int> Eskf::handleContacts(const Contacts& contact, const Joints& qenc) {
  std::vector<int> added;
  for (int leg = 0; leg < 4; ++leg) {
    contact_count_[leg] = contact[leg] ? contact_count_[leg] + 1 : 0;
    if (!contact[leg] && inContact(leg)) marginalize(leg);
  }
  for (int leg = 0; leg < 4; ++leg) {
    if (contact[leg] && !inContact(leg) && contact_count_[leg] > prm_.settle_steps) {
      augment(leg, qenc);
      added.push_back(leg);
    }
  }
  return added;
}

// ---------------------------------------------------------------- update
void Eskf::measurement(const std::vector<int>& legs, const Joints& qenc, Eigen::VectorXd& r,
                       Eigen::MatrixXd& H, Eigen::MatrixXd& Rn) const {
  const Mat3 R = quatToRot(q_);
  const Mat3 Rt = R.transpose();
  const int m = static_cast<int>(legs.size());
  r.setZero(3 * m);
  H.setZero(3 * m, dim());
  Rn.setZero(3 * m, 3 * m);
  for (int j = 0; j < m; ++j) {
    const int leg = legs[j];
    const Vec3 z = footPosition(leg, legJoints(qenc, leg));
    const Vec3 h = Rt * (feet_[leg] - p_);
    r.segment<3>(3 * j) = z - h;
    H.block<3, 3>(3 * j, kIP) = -Rt;
    H.block<3, 3>(3 * j, kITH) = skew(h);
    H.block<3, 3>(3 * j, footIndex(leg)) = Rt;
    Rn.block<3, 3>(3 * j, 3 * j) = kinCov(leg, qenc);
  }
}

void Eskf::update(const Joints& qenc, const std::vector<int>& skip) {
  std::vector<int> legs;
  for (int leg : order_)
    if (std::find(skip.begin(), skip.end(), leg) == skip.end()) legs.push_back(leg);
  if (legs.empty()) return;
  Eigen::VectorXd r;
  Eigen::MatrixXd H, Rn;
  if (prm_.gate_chi2 > 0) {
    std::vector<int> ok;
    for (int leg : legs) {
      measurement({leg}, qenc, r, H, Rn);
      const Eigen::MatrixXd S = H * P_ * H.transpose() + Rn;
      const double nis = r.dot(S.ldlt().solve(r));
      if (nis <= prm_.gate_chi2) ok.push_back(leg); else ++stats.gated;
    }
    legs = ok;
    if (legs.empty()) return;
  }
  measurement(legs, qenc, r, H, Rn);
  const Eigen::MatrixXd PHt = P_ * H.transpose();
  const Eigen::MatrixXd S = H * PHt + Rn;
  const Eigen::MatrixXd K = S.ldlt().solve(PHt.transpose()).transpose();
  const Eigen::VectorXd dx = K * r;
  Eigen::MatrixXd IKH = -K * H;
  IKH.diagonal().array() += 1.0;
  P_ = IKH * P_ * IKH.transpose() + K * Rn * K.transpose();  // Joseph form
  inject(dx);
  ++stats.updates;
}

void Eskf::inject(const Eigen::VectorXd& dx) {
  p_ += dx.segment<3>(kIP);
  v_ += dx.segment<3>(kIV);
  const Vec3 dth = dx.segment<3>(kITH);
  q_ = quatMul(q_, quatExp(dth));
  q_.coeffs() /= q_.norm();
  ba_ += dx.segment<3>(kIBA);
  bg_ += dx.segment<3>(kIBG);
  for (size_t j = 0; j < order_.size(); ++j) feet_[order_[j]] += dx.segment<3>(kNB + 3 * j);
  // ESKF reset Jacobian G = blkdiag(I, I, I - 0.5[dth]x, I, ...): only rows/cols of dtheta change.
  const Mat3 G = Mat3::Identity() - 0.5 * skew(dth);
  P_.middleRows<3>(kITH) = (G * P_.middleRows<3>(kITH)).eval();
  P_.middleCols<3>(kITH) = (P_.middleCols<3>(kITH) * G.transpose()).eval();
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

// ---------------------------------------------------------------- one sample
Snapshot Eskf::step(const Vec3& acc, const Vec3& gyro, const Joints& qenc, const Contacts& contact,
                    double dt, bool use_kinematics) {
  if (use_kinematics) {
    const std::vector<int> added = handleContacts(contact, qenc);
    update(qenc, added);  // a foot initialized from z_k is not also updated with z_k
  }
  Snapshot s{p_, v_, q_, ba_, bg_, P_.diagonal().head<kNB>(), static_cast<int>(order_.size())};
  predict(acc, gyro, dt);
  return s;
}

}  // namespace qeskf
