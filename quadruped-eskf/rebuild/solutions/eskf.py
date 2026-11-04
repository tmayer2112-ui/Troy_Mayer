"""Modules 4 and 5: a contact-aided error-state Kalman filter. REFERENCE SOLUTION.

Same filter as the main project (python/qeskf/eskf.py), cut down so it fits in one file:
  nominal state  p, v, q (quaternion), ba, bg, and one 3D world position per planted foot
  error state    dx = [dp, dv, dtheta, dba, dbg, dd_FR, dd_FL, dd_RR, dd_RL]   (15 + 4*3 = 27 numbers)
Each foot has a fixed slot. A foot that is in the air keeps its slot, but its rows and columns of P
are cleared, so it is ignored until it touches down again (that is what "marginalize" does in
the main project, without resizing P).

Every millisecond, step() does:  contacts -> update (legs) -> predict (IMU).
"""
from dataclasses import dataclass

import numpy as np

from so3 import exp_so3, quat_exp, quat_mul, quat_to_rot, right_jacobian, skew

G = 9.80665
GRAVITY = np.array([0.0, 0.0, -G])

# Where each piece of the error state lives inside dx (and inside P)
IP, IV, ITH, IBA, IBG = 0, 3, 6, 9, 12
NB = 15                      # body part of the error state
NFEET = 4
N = NB + 3 * NFEET           # 27


def foot_index(i):
    """Index in dx of foot i's first component."""
    return NB + 3 * i


@dataclass
class Params:
    acc_noise: float = 175e-6 * G           # accelerometer noise density, m/s^2/sqrt(Hz)
    gyro_noise: float = 0.014 * np.pi / 180  # gyro noise density, rad/s/sqrt(Hz)
    acc_rw: float = 1.7e-4                   # accel bias random walk, m/s^2/sqrt(s)
    gyro_rw: float = 1.7e-6                  # gyro bias random walk, rad/s/sqrt(s)
    leg_noise: float = 0.003                 # m, leg measurement noise per axis
    foot_noise: float = 1e-3                 # m/sqrt(s), how much a planted foot may drift


class Eskf:
    def __init__(self, params, p, v, q, ba, bg, P0):
        self.prm = params
        self.p, self.v, self.q = np.array(p, float), np.array(v, float), np.array(q, float)
        self.ba, self.bg = np.array(ba, float), np.array(bg, float)
        self.d = np.zeros((NFEET, 3))            # world position of each foot (valid when active)
        self.active = np.zeros(NFEET, bool)
        self.P = np.eye(N)
        self.P[:NB, :NB] = P0

    @property
    def R(self):
        return quat_to_rot(self.q)

    # =====================================================================================
    # Module 4: predict
    # =====================================================================================
    def propagate_nominal(self, acc, gyro, dt):
        """Move the nominal state forward one IMU sample."""
        R = self.R
        a_w = R @ (acc - self.ba) + GRAVITY        # remove bias, rotate to world, add gravity back
        w = gyro - self.bg
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        self.q = quat_mul(self.q, quat_exp(w * dt))
        self.q = self.q / np.linalg.norm(self.q)

    def error_transition(self, acc, gyro, dt):
        """F: how an error now becomes an error one step later. 27 x 27, evaluated BEFORE propagating."""
        R = self.R
        a = acc - self.ba
        w = gyro - self.bg
        RA = R @ skew(a)
        F = np.eye(N)
        F[IP:IP + 3, IV:IV + 3] = np.eye(3) * dt
        F[IP:IP + 3, ITH:ITH + 3] = -0.5 * RA * dt * dt
        F[IP:IP + 3, IBA:IBA + 3] = -0.5 * R * dt * dt
        F[IV:IV + 3, ITH:ITH + 3] = -RA * dt
        F[IV:IV + 3, IBA:IBA + 3] = -R * dt
        F[ITH:ITH + 3, ITH:ITH + 3] = exp_so3(-w * dt)
        F[ITH:ITH + 3, IBG:IBG + 3] = -right_jacobian(w * dt) * dt
        return F

    def process_noise(self, dt):
        """Q: the uncertainty one IMU sample adds. Diagonal, 27 x 27."""
        prm = self.prm
        Q = np.zeros((N, N))
        Q[IV:IV + 3, IV:IV + 3] = np.eye(3) * prm.acc_noise ** 2 * dt
        Q[ITH:ITH + 3, ITH:ITH + 3] = np.eye(3) * prm.gyro_noise ** 2 * dt
        Q[IBA:IBA + 3, IBA:IBA + 3] = np.eye(3) * prm.acc_rw ** 2 * dt
        Q[IBG:IBG + 3, IBG:IBG + 3] = np.eye(3) * prm.gyro_rw ** 2 * dt
        Q[NB:, NB:] = np.eye(3 * NFEET) * prm.foot_noise ** 2 * dt
        return Q

    def predict(self, acc, gyro, dt):
        F = self.error_transition(acc, gyro, dt)   # before the nominal state moves
        self.propagate_nominal(acc, gyro, dt)
        self.P = F @ self.P @ F.T + self.process_noise(dt)
        self.P = 0.5 * (self.P + self.P.T)

    # =====================================================================================
    # Module 5: update
    # =====================================================================================
    def measurement(self, i, z):
        """Residual r (3,) and Jacobian H (3 x 27) for planted foot i measured at z (body frame)."""
        R = self.R
        h = R.T @ (self.d[i] - self.p)             # where the filter thinks the foot is, body frame
        r = z - h
        H = np.zeros((3, N))
        H[:, IP:IP + 3] = -R.T
        H[:, ITH:ITH + 3] = skew(h)
        s = foot_index(i)
        H[:, s:s + 3] = R.T
        return r, H

    def update(self, feet, legs):
        """One Kalman update using every foot in `feet`; legs[i] is foot i's body-frame vector."""
        rs, Hs = [], []
        for i in feet:
            r, H = self.measurement(i, legs[i])
            rs.append(r)
            Hs.append(H)
        r = np.concatenate(rs)
        H = np.vstack(Hs)
        Rn = np.eye(len(r)) * self.prm.leg_noise ** 2
        S = H @ self.P @ H.T + Rn
        K = np.linalg.solve(S, H @ self.P).T       # = P H^T S^-1 (S and P are symmetric)
        dx = K @ r
        IKH = np.eye(N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rn @ K.T  # Joseph form: stays symmetric and positive
        self.inject(dx)

    def inject(self, dx):
        """Fold the error estimate into the nominal state, then reset the error to zero."""
        self.p = self.p + dx[IP:IP + 3]
        self.v = self.v + dx[IV:IV + 3]
        dth = dx[ITH:ITH + 3]
        self.q = quat_mul(self.q, quat_exp(dth))   # multiply, never add, for a rotation
        self.q = self.q / np.linalg.norm(self.q)
        self.ba = self.ba + dx[IBA:IBA + 3]
        self.bg = self.bg + dx[IBG:IBG + 3]
        for i in range(NFEET):
            if self.active[i]:
                s = foot_index(i)
                self.d[i] = self.d[i] + dx[s:s + 3]
        G = np.eye(N)
        G[ITH:ITH + 3, ITH:ITH + 3] = np.eye(3) - 0.5 * skew(dth)
        self.P = G @ self.P @ G.T

    # =====================================================================================
    # Provided: contact bookkeeping and the per-sample loop
    # =====================================================================================
    def touchdown(self, i, z):
        """Add foot i: its world position is d = p + R z. It inherits the body's uncertainty."""
        R = self.R
        s = foot_index(i)
        self.P[s:s + 3, :] = 0.0
        self.P[:, s:s + 3] = 0.0
        J = np.zeros((3, N))                        # d(new foot) / d(error state)
        J[:, IP:IP + 3] = np.eye(3)
        J[:, ITH:ITH + 3] = -R @ skew(z)
        JP = J @ self.P
        self.P[s:s + 3, :] = JP
        self.P[:, s:s + 3] = JP.T
        self.P[s:s + 3, s:s + 3] = JP @ J.T + R @ (np.eye(3) * self.prm.leg_noise ** 2) @ R.T
        self.d[i] = self.p + R @ z
        self.active[i] = True

    def liftoff(self, i):
        """Forget foot i: clear its rows and columns so nothing depends on it any more."""
        s = foot_index(i)
        self.P[s:s + 3, :] = 0.0
        self.P[:, s:s + 3] = 0.0
        self.P[s:s + 3, s:s + 3] = np.eye(3)
        self.active[i] = False

    def step(self, acc, gyro, contact, legs, dt, use_legs=True):
        """One IMU sample. Returns the estimate at this sample (after the leg update, before predict)."""
        if use_legs:
            for i in range(NFEET):
                if self.active[i] and not contact[i]:
                    self.liftoff(i)
            new = []
            for i in range(NFEET):
                if contact[i] and not self.active[i]:
                    self.touchdown(i, legs[i])
                    new.append(i)
            feet = [i for i in range(NFEET) if self.active[i] and i not in new]  # don't reuse z_k
            if feet:
                self.update(feet, legs)
        snap = (self.p.copy(), self.v.copy(), self.q.copy(), self.ba.copy(), self.bg.copy(), self.P[:NB, :NB].copy())
        self.predict(acc, gyro, dt)
        return snap


# =========================================================================================
# Provided: starting the filter
# =========================================================================================
def static_alignment(acc, gyro, yaw0=0.0):
    """Level the filter from a window of standing still.

    Roll and pitch come from the direction of the mean accelerometer reading (it should point up).
    Gyro bias = the mean gyro reading (the body isn't rotating). Horizontal accelerometer bias
    CANNOT be told apart from tilt here, so it is left at zero and the tilt absorbs it; P0 says so.
    Yaw can't be measured at all; we start it from the truth, as the main project does.
    """
    f = acc.mean(0)
    fn = f / np.linalg.norm(f)
    roll = np.arctan2(fn[1], fn[2])
    pitch = np.arctan2(-fn[0], np.sqrt(fn[1] ** 2 + fn[2] ** 2))
    from so3 import rot_from_rpy
    R = rot_from_rpy(roll, pitch, yaw0)
    ba = np.array([0.0, 0.0, np.linalg.norm(f) - G])
    bg = gyro.mean(0)
    return R, ba, bg


def initial_covariance(spec, n_cal, p_sigma=1e-3, v_sigma=0.01, yaw_sigma=1e-3):
    """P0 (15 x 15) with the tilt / horizontal-accel-bias link written in.

    Levelling turns a horizontal bias into a tilt error: dtheta_x = -dba_y / g, dtheta_y = dba_x / g.
    So tilt and bias errors start as one variable, not two independent ones.
    """
    P = np.zeros((NB, NB))
    P[IP:IP + 3, IP:IP + 3] = np.eye(3) * p_sigma ** 2
    P[IV:IV + 3, IV:IV + 3] = np.eye(3) * v_sigma ** 2
    sb = spec.acc_bias_sigma
    A = np.array([[0.0, -1.0 / G, 0.0], [1.0 / G, 0.0, 0.0], [0.0, 0.0, 0.0]])   # dba -> dtheta
    s_avg_a = spec.acc_noise_density * np.sqrt(spec.rate_hz) / np.sqrt(n_cal)
    s_avg_g = spec.gyro_noise_density * np.sqrt(spec.rate_hz) / np.sqrt(n_cal)
    Sb = np.diag([sb ** 2, sb ** 2, s_avg_a ** 2 + spec.acc_bias_rw ** 2 * 10.0])
    P[IBA:IBA + 3, IBA:IBA + 3] = Sb
    P[ITH:ITH + 3, ITH:ITH + 3] = A @ Sb @ A.T + np.diag([(s_avg_a / G) ** 2] * 2 + [yaw_sigma ** 2])
    P[ITH:ITH + 3, IBA:IBA + 3] = A @ Sb
    P[IBA:IBA + 3, ITH:ITH + 3] = (A @ Sb).T
    P[IBG:IBG + 3, IBG:IBG + 3] = np.eye(3) * (s_avg_g ** 2 + 1e-8)
    return P


# Provided: nudging the state on the manifold, for the finite-difference checks
def boxplus(f, dx):
    """A copy of filter f with the error dx applied, the same way inject() applies it (P untouched)."""
    g = Eskf.__new__(Eskf)
    g.__dict__.update({k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in f.__dict__.items()})
    g.p = f.p + dx[IP:IP + 3]
    g.v = f.v + dx[IV:IV + 3]
    g.q = quat_mul(f.q, quat_exp(dx[ITH:ITH + 3]))
    g.ba = f.ba + dx[IBA:IBA + 3]
    g.bg = f.bg + dx[IBG:IBG + 3]
    for i in range(NFEET):
        s = foot_index(i)
        g.d[i] = f.d[i] + dx[s:s + 3]
    return g


def boxminus(a, b):
    """The error dx that takes filter state b to filter state a (a [-] b)."""
    from so3 import log_so3
    dx = np.zeros(N)
    dx[IP:IP + 3] = a.p - b.p
    dx[IV:IV + 3] = a.v - b.v
    dx[ITH:ITH + 3] = log_so3(b.R.T @ a.R)
    dx[IBA:IBA + 3] = a.ba - b.ba
    dx[IBG:IBG + 3] = a.bg - b.bg
    for i in range(NFEET):
        s = foot_index(i)
        dx[s:s + 3] = a.d[i] - b.d[i]
    return dx
