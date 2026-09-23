// Tiny text I/O for the replay tools: numeric CSV with a header row, and "key v1 v2 ..." config files.
#pragma once

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "qeskf/eskf.hpp"

namespace qeskf {

inline std::vector<std::vector<double>> readCsv(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::string line;
  std::getline(in, line);  // header
  std::vector<std::vector<double>> rows;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> row;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
    rows.push_back(std::move(row));
  }
  return rows;
}

using Config = std::map<std::string, std::vector<double>>;

inline Config readConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  Config cfg;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string key;
    ss >> key;
    double x;
    while (ss >> x) cfg[key].push_back(x);
  }
  return cfg;
}

inline std::vector<double> need(const Config& c, const std::string& k, size_t n) {
  auto it = c.find(k);
  if (it == c.end() || it->second.size() != n)
    throw std::runtime_error("config key '" + k + "' missing or wrong length");
  return it->second;
}

inline double scalar(const Config& c, const std::string& k) {
  auto it = c.find(k);
  if (it == c.end() || it->second.size() != 1) throw std::runtime_error("config key '" + k + "' missing or not scalar");
  return it->second.front();
}

inline EskfParams paramsFromConfig(const Config& c) {
  EskfParams p;
  p.acc_noise = scalar(c, "acc_noise");
  p.gyro_noise = scalar(c, "gyro_noise");
  p.acc_rw = scalar(c, "acc_rw");
  p.gyro_rw = scalar(c, "gyro_rw");
  p.foot_noise = scalar(c, "foot_noise");
  p.kin_noise = scalar(c, "kin_noise");
  p.enc_lsb = scalar(c, "enc_lsb");
  p.gate_chi2 = scalar(c, "gate_chi2");
  p.settle_steps = static_cast<int>(scalar(c, "settle_steps"));
  return p;
}

inline Eskf filterFromConfig(const Config& c) {
  auto v3 = [&](const std::string& k) {
    const auto x = need(c, k, 3);
    return Vec3(x[0], x[1], x[2]);
  };
  const auto q = need(c, "q0", 4);
  const auto P = need(c, "P0", kNB * kNB);
  Eigen::MatrixXd P0(kNB, kNB);
  for (int i = 0; i < kNB; ++i)
    for (int j = 0; j < kNB; ++j) P0(i, j) = P[i * kNB + j];
  return Eskf(paramsFromConfig(c), v3("p0"), v3("v0"), Quat(q[0], q[1], q[2], q[3]), v3("ba0"),
              v3("bg0"), P0);
}

// One sensor row: t, acc(3), gyro(3), qenc(12), contact(4)
struct SensorRow {
  double t;
  Vec3 acc, gyro;
  Joints q;
  Contacts c;
};

inline SensorRow parseRow(const std::vector<double>& r) {
  if (r.size() < 23) throw std::runtime_error("sensor row has fewer than 23 columns");
  SensorRow s;
  s.t = r[0];
  s.acc = Vec3(r[1], r[2], r[3]);
  s.gyro = Vec3(r[4], r[5], r[6]);
  for (int i = 0; i < 12; ++i) s.q(i) = r[7 + i];
  for (int i = 0; i < 4; ++i) s.c[i] = r[19 + i] > 0.5;
  return s;
}

}  // namespace qeskf
