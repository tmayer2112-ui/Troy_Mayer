"""Model-based trot controller for the simulated Go1.

This exists only to make the robot walk so the estimator has something to
estimate. It reads ground-truth base state from the simulator (the estimator
never does). Foot targets are generated in the trunk frame from a phase clock
and converted to joint targets with analytic IK; MuJoCo's position actuators
track them.

Why not a Playground PPO policy: see README section "Deviations from the plan".
"""
from dataclasses import dataclass

import numpy as np

from . import kinematics as K

# Trot: diagonal pairs FR+RL and FL+RR are half a cycle apart.
PHASE_OFFSET = np.array([0.0, 0.5, 0.5, 0.0])


@dataclass
class GaitParams:
    period: float = 0.40        # s, full gait cycle
    duty: float = 0.60          # stance fraction -> 4-foot support overlap each half-cycle
    height: float = 0.27        # m, nominal trunk height above feet
    swing_height: float = 0.07  # m
    raibert_k: float = 0.06     # s, foot placement gain on velocity error
    k_att: float = 1.2          # m/rad per m lever arm, attitude correction on stance legs
    d_att: float = 0.06         # s, damping on attitude correction
    stand_time: float = 2.0     # s of standing still before the gait clock starts
    k_height: float = 1.0       # proportional height correction


def nominal_foot(leg: int) -> np.ndarray:
    return K.HIP_OFFSETS[leg] + np.array([0.0, K.SIDE[leg] * K.THIGH_Y, 0.0])


def _smoothstep(x: float) -> float:
    return x * x * (3.0 - 2.0 * x)


def _rot2(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


class TrotController:
    def __init__(self, params: GaitParams = GaitParams()):
        self.p = params

    def leg_phase(self, t: float) -> np.ndarray:
        tg = max(t - self.p.stand_time, 0.0)
        return (tg / self.p.period + PHASE_OFFSET) % 1.0

    def scheduled_stance(self, t: float) -> np.ndarray:
        if t < self.p.stand_time:
            return np.ones(4, dtype=bool)
        return self.leg_phase(t) < self.p.duty

    def foot_targets(self, t, cmd, v_body, rpy, omega_body, height=None):
        """cmd = (vx, vy, wz) in the heading frame. Returns (4,3) trunk-frame targets."""
        p = self.p
        targets = np.zeros((4, 3))
        vx, vy, wz = cmd
        t_st = p.period * p.duty
        roll, pitch = rpy[0], rpy[1]
        # Servo compliance lets the trunk sag under load; push the stance feet down to compensate.
        h_err = 0.0 if height is None else (p.height - height)
        self._h_int = getattr(self, "_h_int", 0.0) + 0.002 * h_err
        h_cmd = p.height + p.k_height * h_err + self._h_int
        walking = t >= p.stand_time
        phase = self.leg_phase(t)
        for i in range(4):
            nom = nominal_foot(i)
            dz = p.k_att * (roll * nom[1] - pitch * nom[0]) \
                + p.d_att * (omega_body[0] * nom[1] - omega_body[1] * nom[0])
            if not walking:
                targets[i] = [nom[0], nom[1], -h_cmd + dz]
                continue
            # Foot xy at stance progress s in [0,1]: fixed in world => moves backwards in body frame.
            def stance_xy(s):
                tau = (s - 0.5) * t_st
                return _rot2(-wz * tau) @ nom[:2] - np.array([vx, vy]) * tau
            ph = phase[i]
            if ph < p.duty:
                s = ph / p.duty
                xy = stance_xy(s)
                z = -h_cmd + dz
            else:
                s = (ph - p.duty) / (1.0 - p.duty)
                start = stance_xy(1.0)
                # Ground-speed matching: the touchdown target moves backwards along the
                # extrapolated stance path, so by s = 0.8 the foot already moves at -v in
                # the body frame (~zero velocity over ground) and lands without skidding.
                s_early = -(1.0 - s) * (1.0 - p.duty) / p.duty
                end = stance_xy(s_early) + p.raibert_k * (v_body[:2] - np.array([vx, vy]))
                b = _smoothstep(min(s / 0.8, 1.0))
                xy = (1 - b) * start + b * end
                # Raised-cosine lift: zero vertical velocity at liftoff and touchdown.
                z = -h_cmd + dz + p.swing_height * 0.5 * (1.0 - np.cos(2.0 * np.pi * s))
            targets[i] = [xy[0], xy[1], z]
        return targets

    def joint_targets(self, *args):
        tg = self.foot_targets(*args)
        return np.concatenate([K.leg_ik(i, tg[i]) for i in range(4)])


def command_profile(t: float, stand_time: float = 2.0) -> np.ndarray:
    """Headline trajectory: stand, walk, turn both ways, side-step, stop.

    Piecewise-linear knots (time, vx, vy, wz). Deliberately includes turning in
    both directions so yaw and gyro bias get exercised, and a lateral segment so
    the filter sees velocity along all horizontal axes.
    """
    knots = np.array([
        [0.0, 0.0, 0.0, 0.0],
        [stand_time, 0.0, 0.0, 0.0],
        [stand_time + 3, 0.40, 0.0, 0.0],
        [12, 0.40, 0.0, 0.0],
        [14, 0.35, 0.0, 0.45],
        [22, 0.35, 0.0, 0.45],
        [24, 0.45, 0.0, 0.0],
        [30, 0.45, 0.0, 0.0],
        [32, 0.30, 0.0, -0.50],
        [40, 0.30, 0.0, -0.50],
        [42, 0.0, 0.20, 0.0],
        [47, 0.0, 0.20, 0.0],
        [49, 0.30, -0.10, 0.25],
        [55, 0.30, -0.10, 0.25],
        [57, 0.0, 0.0, 0.0],
        [1e9, 0.0, 0.0, 0.0],
    ])
    return np.array([np.interp(t, knots[:, 0], knots[:, k]) for k in (1, 2, 3)])


def long_walk_profile(t: float, stand_time: float = 2.0) -> np.ndarray:
    """Slow wander for the long observability run: periodic turns, never stops."""
    if t < stand_time:
        return np.zeros(3)
    tt = t - stand_time
    ramp = min(tt / 3.0, 1.0)
    vx = ramp * (0.30 + 0.08 * np.sin(2 * np.pi * tt / 23.0))
    vy = ramp * 0.06 * np.sin(2 * np.pi * tt / 31.0)
    wz = ramp * 0.35 * np.sin(2 * np.pi * tt / 37.0)
    return np.array([vx, vy, wz])
