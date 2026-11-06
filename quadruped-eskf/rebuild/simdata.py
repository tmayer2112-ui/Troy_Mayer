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
  * swing feet: between stances each foot follows a 7 cm-high arc to its next touchdown, and the leg
    sensor keeps reporting it (as encoders would), so a contact detector that is late uses a moving foot
  * a vertical foot force per leg: the robot's weight shared among the planted feet, ramping up after
    touchdown and down over the last 20 % of stance as the other pair takes the load (Module 7)
  * optional foot slip (each planted foot slides backwards at `slip` m/s)
  * optional foot_motion=True: each planted foot sinks 2 mm in its first 20 ms (soft ground) and rises
    3.4 mm over the last 20 % of stance as it unloads. These sizes come from the real project's
    DEBUG_LOG #4 (sinks ~2 mm early, rises ~4 mm late, net +1.4 mm per stance).
Dropped:
  * leg dynamics. The leg sensor reports the foot vector; Module 6 turns it into joint angles (inverse
    kinematics) and back (your forward kinematics) to model the encoders.
  * a real contact model: the force is a smooth weight-sharing profile, not a contact solver
  * vibration, and sensor noise that is correlated over time

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


WEIGHT = 12.74 * G   # N, the Go1 model's mass (12.74 kg) times g
SWING_HEIGHT = 0.07  # m


def _runs(mask):
    """[(start, end), ...] of the True runs in a 1-D bool array (end exclusive)."""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    d = np.diff(m)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def make_walk(seed=0, duration=30.0, turn=True, noise=True, biases=True, slip=0.0, foot_motion=False,
              spec=SensorSpec()):
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

    # --- feet: where each one is in the world, planted or swinging ---
    contact = scheduled_stance(t)
    t_stance = PERIOD * DUTY
    feet_world = np.zeros((n, 4, 3))
    weight_share = np.zeros((n, 4))
    n_sink = int(round(0.020 / dt))
    for i in range(4):
        runs = _runs(contact[:, i])
        for r, (a, b) in enumerate(runs):
            # touchdown: land half a stance ahead of the hip so the foot ends half a stance behind it
            foot = p[a] + R[a] @ NOMINAL_FEET[i] + heading[a] * speed[a] * t_stance / 2
            foot[2] = 0.0                                        # flat ground at z = 0
            ks = np.arange(a, b)
            feet_world[a:b, i] = foot
            if slip:                                             # the planted foot slides backwards
                feet_world[a:b, i] -= np.cumsum(heading[a:b], 0) * slip * dt - heading[a] * slip * dt
            s_prog = (ks - a) / max(b - a, 1)                    # 0 at touchdown, ~1 at liftoff
            if foot_motion:
                sink = -0.002 * _smoothstep((ks - a) / n_sink)
                rise = 0.0034 * np.clip((s_prog - 0.8) / 0.2, 0.0, 1.0) ** 2
                feet_world[a:b, i, 2] += sink + rise
            weight_share[a:b, i] = _smoothstep(s_prog / 0.1) * (1.0 - _smoothstep((s_prog - 0.8) / 0.2))
            if b < n:                                            # swing to the next touchdown
                c = runs[r + 1][0] if r + 1 < len(runs) else n
                start = feet_world[b - 1, i]
                end = start + heading[b] * speed[b] * PERIOD if c >= n else None
                if end is None:
                    end = p[c] + R[c] @ NOMINAL_FEET[i] + heading[c] * speed[c] * t_stance / 2
                    end[2] = 0.0
                u = (np.arange(b, c) - b + 1) / (c - b + 1)
                feet_world[b:c, i] = start + _smoothstep(u)[:, None] * (end - start)
                feet_world[b:c, i, 2] += SWING_HEIGHT * 0.5 * (1.0 - np.cos(2 * np.pi * u))
    weight_share[t < STAND] = 1.0                                 # standing: all four share the weight
    force = WEIGHT * weight_share / np.maximum(weight_share.sum(1, keepdims=True), 1e-9)

    # --- the leg sensor: each foot's position in the body frame (stands in for FK on the encoders) ---
    legs_true = np.einsum("kji,kfj->kfi", R, feet_world - p[:, None, :])
    legs = legs_true.copy()
    for k in range(n):                                           # stance noise first, in this order,
        for i in range(4):                                       # so every seed keeps its numbers
            if contact[k, i] and noise:
                legs[k, i] += rng.normal(0, spec.leg_noise, 3)
    if noise:                                                    # swing feet: their own random stream
        rng_swing = np.random.default_rng(seed + 10_000)
        legs[~contact] += rng_swing.normal(0, spec.leg_noise, (int((~contact).sum()), 3))

    q = np.array([rot_to_quat(Rk) for Rk in R])
    return dict(t=t, dt=dt, acc=acc, gyro=gyro, contact=contact, legs=legs, legs_true=legs_true,
                feet_world=feet_world, force=force, p=p, v=v, q=q, ba=ba, bg=bg, spec=spec,
                turn=turn, slip=slip, foot_motion=foot_motion, seed=seed)
