"""Synthetic BMI088 IMU and joint encoders.

Every number below has a source. Where Bosch does not publish a number I say so
and state the assumption instead of hiding it in a constant.

Sources
  [DS]  Bosch Sensortec, BMI088 data sheet, BST-BMI088-DS001 (Table 1 accel, Table 2 gyro).
  [FL]  Bosch Sensortec, BMI088 product flyer, BST-BMI088-FL000.
  [AS]  ams AS5047P data sheet (14-bit absolute magnetic rotary encoder).
The build sandbox could not download the PDFs (egress blocked), so the values
were cross-checked against distributor listings rather than read off the
tables directly; re-verify against the PDF before quoting them. See README.

Discrete-time conversion (Kalibr / Furgale convention, used consistently by the
simulator here and by the filter's Q):
  white noise, per sample:   sigma_d = N * sqrt(f_s)      (N = noise density)
  bias random walk, per step: sigma_bd = K * sqrt(dt)     (K = rate random walk)
Note: sqrt(f_s), not f_s. The sensor's internal low-pass lowers the per-sample
noise at high frequency, but drift is driven by the low-frequency PSD, which is N^2.
"""
from dataclasses import asdict, dataclass

import numpy as np

G0 = 9.80665
DEG = np.pi / 180.0


def rrw_from_bias_instability(b_inst, tau):
    """Rate-random-walk coefficient whose Allan deviation equals b_inst at tau.

    Allan deviation of a random walk: sigma(tau) = K * sqrt(tau / 3).
    This is a modelling choice: bias instability is flicker noise (flat Allan
    floor); a random walk that matches it at one tau is the usual Kalman-filter
    stand-in. Below tau it under-states the drift, above tau it over-states it.
    """
    return b_inst * np.sqrt(3.0 / tau)


@dataclass
class ImuSpec:
    rate_hz: float = 1000.0
    # --- gyroscope ---
    gyro_range_dps: float = 1000.0                      # [DS] selectable +-125..2000 deg/s; we pick 1000
    gyro_bits: int = 16                                 # [DS] 16-bit output
    gyro_noise_density: float = 0.014 * DEG             # [DS] 0.014 deg/s/sqrt(Hz) typ -> rad/s/sqrt(Hz)
    gyro_turn_on_bias: float = 1.0 * DEG                # [DS] zero-rate offset +-1 deg/s typ (1-sigma here)
    gyro_bias_instability: float = 2.0 * DEG / 3600.0   # [FL] "< 2 deg/h" -- flyer, NOT in the data sheet
    gyro_tempco: float = 0.015 * DEG                    # [DS] offset tempco 0.015 deg/s/K (thermal option only)
    # --- accelerometer ---
    acc_range_g: float = 6.0                            # [DS] +-3/6/12/24 g; sim peaks ~2.5 g at touchdown
    acc_bits: int = 16                                  # [DS]
    acc_noise_density: float = 175e-6 * G0              # [DS]/[FL] 175 ug/sqrt(Hz) typ -> m/s^2/sqrt(Hz)
    acc_turn_on_bias: float = 20e-3 * G0                # [DS] zero-g offset +-20 mg typ (1-sigma here)
    acc_bias_instability: float = 100e-6 * G0           # ASSUMPTION: Bosch publishes none; 100 ug is typical consumer MEMS
    # --- random-walk conversion ---
    bias_tau: float = 100.0                             # s, Allan tau at which the RW matches bias instability
    # --- optional thermal warm-up drift of gyro bias (off for the headline run) ---
    thermal_dT: float = 0.0                             # K of warm-up
    thermal_tau: float = 300.0                          # s

    @property
    def dt(self):
        return 1.0 / self.rate_hz

    @property
    def gyro_rrw(self):
        return rrw_from_bias_instability(self.gyro_bias_instability, self.bias_tau)

    @property
    def acc_rrw(self):
        return rrw_from_bias_instability(self.acc_bias_instability, self.bias_tau)

    @property
    def gyro_lsb(self):
        return 2 * self.gyro_range_dps * DEG / 2 ** self.gyro_bits

    @property
    def acc_lsb(self):
        return 2 * self.acc_range_g * G0 / 2 ** self.acc_bits

    def summary(self):
        d = asdict(self)
        d.update(gyro_rrw=self.gyro_rrw, acc_rrw=self.acc_rrw, gyro_lsb=self.gyro_lsb, acc_lsb=self.acc_lsb,
                 gyro_sigma_per_sample=self.gyro_noise_density * np.sqrt(self.rate_hz),
                 acc_sigma_per_sample=self.acc_noise_density * np.sqrt(self.rate_hz))
        return d


@dataclass
class EncoderSpec:
    bits: int = 14            # [AS] AS5047P-class absolute encoder on the joint output (ASSUMPTION: placement)

    @property
    def lsb(self):
        return 2 * np.pi / 2 ** self.bits


def _quantize(x, lsb, full_scale):
    return np.clip(np.round(x / lsb) * lsb, -full_scale, full_scale - lsb)


def synthesize_imu(acc_true, gyro_true, spec: ImuSpec, rng: np.random.Generator):
    """Ideal specific force / angular rate (body frame, N x 3) -> quantized BMI088 samples.

    Returns (acc_meas, gyro_meas, bias_acc, bias_gyro); the biases are the true
    time-varying biases, kept as ground truth for bias-estimation plots.
    """
    n = acc_true.shape[0]
    dt = spec.dt
    ba0 = rng.normal(0.0, spec.acc_turn_on_bias, 3)
    bg0 = rng.normal(0.0, spec.gyro_turn_on_bias, 3)
    ba = ba0 + np.cumsum(rng.normal(0.0, spec.acc_rrw * np.sqrt(dt), (n, 3)), axis=0)
    bg = bg0 + np.cumsum(rng.normal(0.0, spec.gyro_rrw * np.sqrt(dt), (n, 3)), axis=0)
    if spec.thermal_dT:
        t = np.arange(n) * dt
        dT = spec.thermal_dT * (1 - np.exp(-t / spec.thermal_tau))
        bg = bg + (spec.gyro_tempco * dT)[:, None] * rng.choice([-1.0, 1.0], 3)[None, :]
    wa = rng.normal(0.0, spec.acc_noise_density * np.sqrt(spec.rate_hz), (n, 3))
    wg = rng.normal(0.0, spec.gyro_noise_density * np.sqrt(spec.rate_hz), (n, 3))
    acc = _quantize(acc_true + ba + wa, spec.acc_lsb, spec.acc_range_g * G0)
    gyr = _quantize(gyro_true + bg + wg, spec.gyro_lsb, spec.gyro_range_dps * DEG)
    return acc, gyr, ba, bg


def quantize_encoders(qj, spec: EncoderSpec):
    return np.round(qj / spec.lsb) * spec.lsb
