"""Contact-aided error-state EKF (reference implementation; C++ mirrors this 1:1).

Nominal state  x = (p, v, q, b_a, b_g, d_1..d_m)
  p, v   IMU/trunk position and velocity in WORLD
  q      world-from-body unit quaternion
  b_a,b_g accelerometer / gyro biases (body)
  d_i    WORLD position of each foot currently in contact (0..4 of them)
Error state  dx = (dp, dv, dtheta, db_a, db_g, dd_1..dd_m), dim 15 + 3m.
  dtheta is a LOCAL (body-frame) rotation error: R_true = R_hat Exp(dtheta).
  Minimal 3-parameter attitude error keeps P full-rank; a 4-parameter quaternion
  covariance would be singular along the unit-norm constraint.

IMU model: acc_m = R^T (a_w - g_w) + b_a + n_a   (specific force, body frame)
           gyr_m = w_b + b_g + n_g
Gravity g_w = (0, 0, -9.80665) is added back in WORLD after rotating the
bias-corrected specific force: a_w = R (acc_m - b_a) + g_w.

Measurement per stance foot i:  z_i = FK_i(q_enc)  (body frame)
                                h_i = R^T (d_i - p)
Contact events:
  touchdown  -> augment: d_i = p + R FK_i(q), P grown with the exact Jacobian
  liftoff    -> marginalize: drop d_i's rows/cols (Gaussian marginal)
"""
from dataclasses import dataclass, field

import numpy as np

from . import kinematics as K
from .so3 import exp_so3, quat_exp, quat_mul, quat_to_rot, skew

GRAVITY = np.array([0.0, 0.0, -9.80665])
NB = 15  # core error-state size
IP, IV, ITH, IBA, IBG = 0, 3, 6, 9, 12


def right_jacobian(phi):
    th = np.linalg.norm(phi)
    Kx = skew(phi)
    if th < 1e-6:
        return np.eye(3) - 0.5 * Kx + Kx @ Kx / 6.0
    return (np.eye(3) - (1 - np.cos(th)) / th ** 2 * Kx + (th - np.sin(th)) / th ** 3 * Kx @ Kx)


@dataclass
class EskfParams:
    acc_noise: float = 175e-6 * 9.80665          # N_a, m/s^2/sqrt(Hz)      (BMI088)
    gyro_noise: float = 0.014 * np.pi / 180.0    # N_g, rad/s/sqrt(Hz)      (BMI088)
    acc_rw: float = 1.0e-4                       # K_a, m/s^3/sqrt(Hz)      (set from ImuSpec)
    gyro_rw: float = 1.0e-6                      # K_g, rad/s^2/sqrt(Hz)    (set from ImuSpec)
    foot_noise: float = 0.01                     # m/sqrt(s), world-frame foot random walk ("soft contact")
    kin_noise: float = 0.003                     # m, FK model error per axis (foot pad, rolling)
    enc_lsb: float = 2 * np.pi / 2 ** 14         # rad, encoder quantization step
    gate_chi2: float = 0.0                       # per-foot chi^2 gate (3 dof); 0 disables
    settle_steps: int = 0                        # contact must persist this many samples before augmenting

    @classmethod
    def from_imu_spec(cls, spec, **kw):
        return cls(acc_noise=spec.acc_noise_density, gyro_noise=spec.gyro_noise_density,
                   acc_rw=spec.acc_rrw, gyro_rw=spec.gyro_rrw, **kw)


@dataclass
class EskfState:
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    ba: np.ndarray
    bg: np.ndarray
    feet: dict = field(default_factory=dict)   # leg id -> world position
    order: list = field(default_factory=list)  # leg ids in error-state order


class Eskf:
    def __init__(self, params: EskfParams, p, v, q, ba, bg, P0):
        self.prm = params
        self.x = EskfState(np.array(p, float), np.array(v, float), np.array(q, float),
                           np.array(ba, float), np.array(bg, float))
        self.P = np.array(P0, float)
        self.contact_count = np.zeros(4, dtype=int)
        self.stats = dict(updates=0, gated=0, augment=0, marginalize=0)
        self.last_nis = np.full(4, np.nan)
        self.fk_offset = None   # optional (4,3) body-frame additive FK error, for slip-injection experiments

    # ------------------------------------------------------------------ helpers
    @property
    def dim(self):
        return self.P.shape[0]

    def R(self):
        return quat_to_rot(self.x.q)

    def fk(self, leg, qenc):
        f = K.foot_position(leg, qenc[3 * leg:3 * leg + 3])
        return f if self.fk_offset is None else f + self.fk_offset[leg]

    def foot_index(self, leg):
        return NB + 3 * self.x.order.index(leg)

    # ------------------------------------------------------------------ predict
    def propagate_nominal(self, acc, gyro, dt):
        x = self.x
        R = quat_to_rot(x.q)
        a_w = R @ (acc - x.ba) + GRAVITY
        w = gyro - x.bg
        x.p = x.p + x.v * dt + 0.5 * a_w * dt * dt
        x.v = x.v + a_w * dt
        x.q = quat_mul(x.q, quat_exp(w * dt))
        x.q /= np.linalg.norm(x.q)

    def error_transition(self, acc, gyro, dt):
        """Exact Jacobian F = d(dx_{k+1})/d(dx_k) of propagate_nominal (tested against finite differences)."""
        x = self.x
        R = quat_to_rot(x.q)
        a = acc - x.ba
        w = gyro - x.bg
        n = self.dim
        F = np.eye(n)
        RA = R @ skew(a)
        F[IP:IP + 3, IV:IV + 3] = np.eye(3) * dt
        F[IP:IP + 3, ITH:ITH + 3] = -0.5 * RA * dt * dt
        F[IP:IP + 3, IBA:IBA + 3] = -0.5 * R * dt * dt
        F[IV:IV + 3, ITH:ITH + 3] = -RA * dt
        F[IV:IV + 3, IBA:IBA + 3] = -R * dt
        F[ITH:ITH + 3, ITH:ITH + 3] = exp_so3(-w * dt)
        F[ITH:ITH + 3, IBG:IBG + 3] = -right_jacobian(w * dt) * dt
        return F

    def process_noise(self, dt):
        prm = self.prm
        Q = np.zeros((self.dim, self.dim))
        # Velocity noise from accel white noise: R diag(N_a^2) R^T dt = N_a^2 dt I (isotropic)
        Q[IV:IV + 3, IV:IV + 3] = np.eye(3) * prm.acc_noise ** 2 * dt
        Q[ITH:ITH + 3, ITH:ITH + 3] = np.eye(3) * prm.gyro_noise ** 2 * dt
        Q[IBA:IBA + 3, IBA:IBA + 3] = np.eye(3) * prm.acc_rw ** 2 * dt
        Q[IBG:IBG + 3, IBG:IBG + 3] = np.eye(3) * prm.gyro_rw ** 2 * dt
        for j in range(len(self.x.order)):
            i0 = NB + 3 * j
            Q[i0:i0 + 3, i0:i0 + 3] = np.eye(3) * prm.foot_noise ** 2 * dt
        return Q

    def predict(self, acc, gyro, dt):
        F = self.error_transition(acc, gyro, dt)
        Q = self.process_noise(dt)
        self.propagate_nominal(acc, gyro, dt)
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    # ------------------------------------------------------------------ contacts
    def kin_cov(self, leg, qenc):
        """Body-frame covariance of FK_i(q): encoder quantization (uniform, var lsb^2/12) + model error."""
        Jq = K.foot_jacobian(leg, qenc[3 * leg:3 * leg + 3])
        s2 = self.prm.enc_lsb ** 2 / 12.0
        return s2 * Jq @ Jq.T + np.eye(3) * self.prm.kin_noise ** 2

    def augment(self, leg, qenc):
        x = self.x
        R = quat_to_rot(x.q)
        f = self.fk(leg, qenc)
        d = x.p + R @ f
        n = self.dim
        J = np.zeros((3, n))
        J[:, IP:IP + 3] = np.eye(3)
        J[:, ITH:ITH + 3] = -R @ skew(f)
        Rk = R @ self.kin_cov(leg, qenc) @ R.T
        P = self.P
        PJt = P @ J.T
        Pn = np.zeros((n + 3, n + 3))
        Pn[:n, :n] = P
        Pn[:n, n:] = PJt
        Pn[n:, :n] = PJt.T
        Pn[n:, n:] = J @ PJt + Rk
        self.P = 0.5 * (Pn + Pn.T)
        x.feet[leg] = d
        x.order.append(leg)
        self.stats["augment"] += 1

    def marginalize(self, leg):
        i0 = self.foot_index(leg)
        keep = np.r_[0:i0, i0 + 3:self.dim]
        self.P = self.P[np.ix_(keep, keep)]
        self.x.order.remove(leg)
        del self.x.feet[leg]
        self.stats["marginalize"] += 1

    def handle_contacts(self, contact, qenc):
        """Liftoffs first (marginalize), then touchdowns (augment). Returns legs augmented this step."""
        new = []
        for leg in range(4):
            self.contact_count[leg] = self.contact_count[leg] + 1 if contact[leg] else 0
            if not contact[leg] and leg in self.x.feet:
                self.marginalize(leg)
        for leg in range(4):
            if contact[leg] and leg not in self.x.feet and self.contact_count[leg] > self.prm.settle_steps:
                self.augment(leg, qenc)
                new.append(leg)
        return new

    # ------------------------------------------------------------------ update
    def measurement(self, legs, qenc):
        """Stacked residual r, Jacobian H, noise Rn for the given stance legs."""
        x = self.x
        R = quat_to_rot(x.q)
        m = len(legs)
        r = np.zeros(3 * m)
        H = np.zeros((3 * m, self.dim))
        Rn = np.zeros((3 * m, 3 * m))
        for j, leg in enumerate(legs):
            z = self.fk(leg, qenc)
            h = R.T @ (x.feet[leg] - x.p)
            rows = slice(3 * j, 3 * j + 3)
            r[rows] = z - h
            H[rows, IP:IP + 3] = -R.T
            H[rows, ITH:ITH + 3] = skew(h)
            i0 = self.foot_index(leg)
            H[rows, i0:i0 + 3] = R.T
            Rn[rows, rows] = self.kin_cov(leg, qenc)
        return r, H, Rn

    def update(self, qenc, skip=()):
        legs = [l for l in self.x.order if l not in skip]
        self.last_nis[:] = np.nan
        if not legs:
            return
        if self.prm.gate_chi2 > 0:
            ok = []
            for leg in legs:
                r, H, Rn = self.measurement([leg], qenc)
                S = H @ self.P @ H.T + Rn
                nis = float(r @ np.linalg.solve(S, r))
                self.last_nis[leg] = nis
                if nis <= self.prm.gate_chi2:
                    ok.append(leg)
                else:
                    self.stats["gated"] += 1
            legs = ok
            if not legs:
                return
        r, H, Rn = self.measurement(legs, qenc)
        P = self.P
        S = H @ P @ H.T + Rn
        Kg = np.linalg.solve(S, H @ P).T          # P H^T S^-1, S symmetric
        dx = Kg @ r
        IKH = np.eye(self.dim) - Kg @ H
        self.P = IKH @ P @ IKH.T + Kg @ Rn @ Kg.T  # Joseph form
        self.inject(dx)
        self.stats["updates"] += 1

    def inject(self, dx):
        x = self.x
        x.p = x.p + dx[IP:IP + 3]
        x.v = x.v + dx[IV:IV + 3]
        dth = dx[ITH:ITH + 3]
        x.q = quat_mul(x.q, quat_exp(dth))
        x.q /= np.linalg.norm(x.q)
        x.ba = x.ba + dx[IBA:IBA + 3]
        x.bg = x.bg + dx[IBG:IBG + 3]
        for j, leg in enumerate(x.order):
            x.feet[leg] = x.feet[leg] + dx[NB + 3 * j:NB + 3 * j + 3]
        # ESKF reset: the error is now re-expressed about the corrected nominal attitude.
        G = np.eye(self.dim)
        G[ITH:ITH + 3, ITH:ITH + 3] = np.eye(3) - 0.5 * skew(dth)
        self.P = G @ self.P @ G.T
        self.P = 0.5 * (self.P + self.P.T)

    # ------------------------------------------------------------------ one sample
    def step(self, acc, gyro, qenc, contact, dt, use_kinematics=True, fk_offset=None):
        """Process log row k: contact events + kinematic update at t_k, then IMU propagation to t_{k+1}.

        Returns a snapshot of the posterior at t_k (before propagation).
        """
        self.fk_offset = fk_offset
        if use_kinematics:
            new = self.handle_contacts(contact, qenc)
            # A foot augmented from z_k must not also be updated with z_k (same data twice).
            self.update(qenc, skip=new)
        snap = (self.x.p.copy(), self.x.v.copy(), self.x.q.copy(), self.x.ba.copy(), self.x.bg.copy(),
                np.diag(self.P)[:NB].copy())
        self.predict(acc, gyro, dt)
        return snap


# ---------------------------------------------------------------------- initialization
def static_alignment(acc, gyro, yaw0=0.0):
    """Level + gyro-bias calibration from a stationary window.

    Roll/pitch from the mean specific force; gyro bias = mean rate; accel z-bias
    from the norm (|f| should equal g). Horizontal accel bias is *not*
    separable from tilt at rest; that ambiguity is encoded in P0 below.
    """
    f = acc.mean(0)
    bg = gyro.mean(0)
    fn = f / np.linalg.norm(f)
    roll = np.arctan2(fn[1], fn[2])
    pitch = np.arctan2(-fn[0], np.sqrt(fn[1] ** 2 + fn[2] ** 2))
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw0), np.sin(yaw0)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R = Rz @ Ry @ Rx
    ba = np.zeros(3)
    ba[2] = np.linalg.norm(f) - 9.80665   # |f| = g + b_z when level (first order)
    return R, ba, bg


def initial_covariance(spec, n_cal, p_sigma=1e-3, v_sigma=0.01, yaw_sigma=1e-3):
    """P0 with the tilt / horizontal-accel-bias correlation made explicit.

    At rest the filter sees f = R^T g + b_a. Leveling absorbs b_a,xy into tilt:
    dtheta_x = -db_y / g, dtheta_y = db_x / g. So (tilt, b_a,xy) are drawn from one
    underlying variable, not two independent ones. Independent diagonal P0 here
    would claim tilt knowledge the data never contained.
    """
    g = 9.80665
    P = np.zeros((NB, NB))
    P[IP:IP + 3, IP:IP + 3] = np.eye(3) * p_sigma ** 2
    P[IV:IV + 3, IV:IV + 3] = np.eye(3) * v_sigma ** 2
    sb = spec.acc_turn_on_bias
    A = np.array([[0.0, -1.0 / g, 0.0], [1.0 / g, 0.0, 0.0], [0.0, 0.0, 0.0]])  # db_a -> dtheta
    Sb = np.diag([sb ** 2, sb ** 2, 0.0])
    # residual z-bias after the norm calibration: accel white noise averaged over the window
    s_avg_a = spec.acc_noise_density * np.sqrt(spec.rate_hz) / np.sqrt(n_cal)
    s_avg_g = spec.gyro_noise_density * np.sqrt(spec.rate_hz) / np.sqrt(n_cal)
    Sb[2, 2] = s_avg_a ** 2 + (spec.acc_rrw ** 2) * 10.0
    P[IBA:IBA + 3, IBA:IBA + 3] = Sb
    P[ITH:ITH + 3, ITH:ITH + 3] = A @ Sb @ A.T + np.diag([(s_avg_a / g) ** 2] * 2 + [yaw_sigma ** 2])
    P[ITH:ITH + 3, IBA:IBA + 3] = A @ Sb
    P[IBA:IBA + 3, ITH:ITH + 3] = (A @ Sb).T
    P[IBG:IBG + 3, IBG:IBG + 3] = np.eye(3) * s_avg_g ** 2
    return P
