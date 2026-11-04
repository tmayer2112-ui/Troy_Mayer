"""A synthetic trotting quadruped (provided, you don't write this). Pure numpy, no MuJoCo.

It stands in for the MuJoCo simulation, which you can't run on Windows on ARM. It keeps what the
estimator cares about and drops the rest.

Kept:
  * a body that stands for 2 s, then trots at 0.5 m/s with a little sway and bob,
    turns left 90 degrees at t = 12-16 s (unless turn=False), then walks straight again
  * an IMU at 1 kHz with BMI088-class white noise, a random turn-on bias and a slow bias drift
  * four feet, each planted for 60 % of every 0.4 s trot cycle (diagonal pairs half a cycle apart)
  * a "leg sensor" that reports each planted foot's position in the body frame with 3 mm of
    white noise. It plays the role of forward kinematics on the joint encoders.
  * optional foot slip (each planted foot slides backwards at `slip` m/s)
Dropped:
  * leg dynamics, encoders and forward kinematics (you get the FK answer directly)
  * contact forces and contact detection (contact comes straight from the gait schedule)
  * soft-floor sinking, vibration, and noise that is correlated over time

The truth is built so that the filter's own discrete IMU model is exact:
    R[k+1] = R[k] @ Exp(w[k] dt)
    v[k+1] = v[k] + a[k] dt
    p[k+1] = p[k] + v[k] dt + a[k] dt^2 / 2
and the IMU reports f[k] = R[k]^T (a[k] - g) and w[k] (plus bias and noise). So with noise and
biases switched off, IMU dead reckoning reproduces the truth to rounding error (Module 4 checks this).

Index convention: p[k], v[k], q[k] are the true state at t[k]; legs[k] is measured at that state;
acc[k] and gyro[k] carry the state from k to k+1.
"""
from dataclasses import dataclass

import numpy as np

from so3 import log_so3, rot_from_rpy, rot_to_quat

G = 9.80665
GRAVITY = np.array([0.0, 0.0, -G])
DEG = np.pi / 180.0

STAND = 2.0          # s standing still before walking (the filter calibrates on this)
SPEED = 0.5          # m/s forward
T_TURN = 12.0        # s, start of the left turn
TURN_LEN = 4.0       # s
TURN_RATE = (np.pi / 2) / TURN_LEN   # 90 degrees in total
PERIOD = 0.4         # s, trot cycle
DUTY = 0.6           # fraction of the cycle each foot is planted
PHASE_OFFSET = np.array([0.0, 0.5, 0.5, 0.0])   # FR, FL, RR, RL: diagonal pairs move together
LEG_NAMES = ["FR", "FL", "RR", "RL"]
# Where each foot sits relative to the body when standing (body frame, metres: x forward, y left, z up)
NOMINAL_FEET = np.array([[0.19, -0.13, -0.27],
                         [0.19, 0.13, -0.27],
                         [-0.19, -0.13, -0.27],
                         [-0.19, 0.13, -0.27]])


@dataclass
class SensorSpec:
    rate_hz: float = 1000.0
    acc_noise_density: float = 175e-6 * G    # m/s^2/sqrt(Hz), BMI088-class
    gyro_noise_density: float = 0.014 * DEG  # rad/s/sqrt(Hz), BMI088-class
    acc_bias_sigma: float = 20e-3 * G        # m/s^2, turn-on bias, 1 sigma per axis (20 mg)
    gyro_bias_sigma: float = 1.0 * DEG       # rad/s, turn-on bias, 1 sigma per axis (1 deg/s)
    acc_bias_rw: float = 1.7e-4              # m/s^2/sqrt(s), slow bias drift
    gyro_bias_rw: float = 1.7e-6             # rad/s/sqrt(s)
    leg_noise: float = 0.003                 # m per axis, white noise on the body-frame foot vector


def _smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def scheduled_stance(t):
    """(n, 4) bool: which feet the trot schedule has planted at each time."""
    t = np.atleast_1d(t)
    phase = ((t[:, None] - STAND) / PERIOD + PHASE_OFFSET[None, :]) % 1.0
    return (phase < DUTY) | (t[:, None] < STAND)


def make_walk(seed=0, duration=30.0, turn=True, noise=True, biases=True, slip=0.0, spec=SensorSpec()):
    """Generate one walk. Returns a dict of numpy arrays (see the keys at the bottom)."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / spec.rate_hz
    n = int(round(duration / dt))
    t = np.arange(n + 1) * dt                  # one extra sample so that a[k] exists for every k < n

    # --- motion plan ---
    walk = _smoothstep((t - STAND) / 2.0)      # 0 while standing, ramps to 1 over 2 s
    speed = SPEED * walk
    if turn:
        yaw_rate = TURN_RATE * (_smoothstep((t - T_TURN) / 0.5) - _smoothstep((t - T_TURN - TURN_LEN) / 0.5))
    else:
        yaw_rate = np.zeros_like(t)
    yaw = np.concatenate([[0.0], np.cumsum(yaw_rate[:-1]) * dt])
    roll = 0.02 * walk * np.sin(2 * np.pi * 2.5 * t)            # ~1 deg of sway at the trot frequency
    pitch = 0.015 * walk * np.sin(2 * np.pi * 2.5 * t + 1.0)
    height = 0.27 + 0.004 * walk * np.sin(2 * np.pi * 5.0 * t)  # 4 mm of bob, twice per cycle
    heading = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], 1)

    R = np.array([rot_from_rpy(r, p, y) for r, p, y in zip(roll, pitch, yaw)])
    v = speed[:, None] * heading
    v[:, 2] = np.gradient(height, dt)

    # --- exact discrete truth ---
    a = (v[1:] - v[:-1]) / dt
    w = np.array([log_so3(R[k].T @ R[k + 1]) / dt for k in range(n)])
    p = np.zeros((n, 3))
    p[0] = [0.0, 0.0, height[0]]
    for k in range(n - 1):
        p[k + 1] = p[k] + v[k] * dt + 0.5 * a[k] * dt * dt
    R, v, t, heading, speed = R[:n], v[:n], t[:n], heading[:n], speed[:n]
    f = np.einsum("kji,kj->ki", R, a - GRAVITY)                # R^T (a - g), body frame

    # --- IMU: truth + bias + white noise ---
    sa = spec.acc_noise_density * np.sqrt(spec.rate_hz)        # per-sample sigma
    sg = spec.gyro_noise_density * np.sqrt(spec.rate_hz)
    ba = np.zeros((n, 3))
    bg = np.zeros((n, 3))
    if biases:
        ba = rng.normal(0, spec.acc_bias_sigma, 3) + np.cumsum(rng.normal(0, spec.acc_bias_rw * np.sqrt(dt), (n, 3)), 0)
        bg = rng.normal(0, spec.gyro_bias_sigma, 3) + np.cumsum(rng.normal(0, spec.gyro_bias_rw * np.sqrt(dt), (n, 3)), 0)
    acc = f + ba + (rng.normal(0, sa, (n, 3)) if noise else 0.0)
    gyro = w + bg + (rng.normal(0, sg, (n, 3)) if noise else 0.0)

    # --- feet and the leg sensor ---
    contact = scheduled_stance(t)
    legs = np.full((n, 4, 3), np.nan)
    feet = np.zeros((4, 3))
    t_stance = PERIOD * DUTY
    for k in range(n):
        for i in range(4):
            if not contact[k, i]:
                continue
            if k == 0 or not contact[k - 1, i]:
                # touchdown: land half a stance ahead of the hip so the foot ends half a stance behind it
                feet[i] = p[k] + R[k] @ NOMINAL_FEET[i] + heading[k] * speed[k] * t_stance / 2
                feet[i, 2] = 0.0                               # flat ground at z = 0
            elif slip:
                feet[i] -= heading[k] * slip * dt             # the planted foot slides backwards
            legs[k, i] = R[k].T @ (feet[i] - p[k])
            if noise:
                legs[k, i] += rng.normal(0, spec.leg_noise, 3)

    q = np.array([rot_to_quat(Rk) for Rk in R])
    return dict(t=t, dt=dt, acc=acc, gyro=gyro, contact=contact, legs=legs,
                p=p, v=v, q=q, ba=ba, bg=bg, spec=spec, turn=turn, slip=slip, seed=seed)
