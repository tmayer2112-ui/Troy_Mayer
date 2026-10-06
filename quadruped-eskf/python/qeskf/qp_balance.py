"""QP force-balance controller for the stance legs (MIT Cheetah 3 style, simplified).

The scripted trot (gait.py) moves every foot along a position trajectory and lets stiff joint servos
track it; nothing in it reasons about forces. Here the stance legs are force controlled instead:

  1. PD on the body -> desired body acceleration and angular acceleration
       a_des    = [kv (vx_cmd - vx), kv (vy_cmd - vy), kpz (h_des - h) - kdz vz]
       wdot_des = [-kr roll - kdr wx, -kr pitch - kdr wy, kdy (wz_cmd - wz)]
  2. Single-rigid-body model: the stance-foot forces f_i must produce that motion
       sum f_i            = m (a_des - g)
       sum r_i x f_i      = I wdot_des          (r_i = foot relative to the centre of mass)
     i.e. A f = b. Solve min ||W (A f - b)||^2 + alpha ||f||^2 subject to friction and f_z >= 0.
  3. Joint torques: tau_i = J_i^T (-f_i in the body frame)  (the foot pushes on the ground with -f_i)

Friction is a linearized cone: each foot force is a non-negative combination of four edge vectors
(+-mu, 0, 1) and (0, +-mu, 1), which allows exactly |f_x| + |f_y| <= mu f_z (a pyramid inside the
true cone, so conservative). With that change of variables the only constraint is beta >= 0, so the
QP is a non-negative least-squares problem, solved exactly by scipy.optimize.nnls (Lawson-Hanson).

Everything is expressed in a level "heading" frame: z up, x along the body's heading. So the
controller needs roll, pitch, body-frame velocity and rates, and trunk height, never yaw or position,
which the estimator can't observe. Swing legs keep the scripted trajectory (including Raibert foot
placement) tracked by a joint PD with the same stiffness as Menagerie's position servos (kp = 100).
"""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.optimize import nnls

from . import kinematics as K
from .gait import GaitParams, TrotController
from .so3 import quat_to_rot, skew

G_W = np.array([0.0, 0.0, -9.80665])


@dataclass
class QPGains:
    kv: float = 4.0          # 1/s, horizontal velocity
    kpz: float = 100.0       # 1/s^2, height
    kdz: float = 20.0        # 1/s
    kr: float = 150.0        # 1/s^2, roll/pitch
    kdr: float = 20.0        # 1/s
    kdy: float = 10.0        # 1/s, yaw rate
    mu: float = 0.6          # friction assumed by the controller; the foot geoms have 0.8, so this leaves margin
    weights: tuple = (1.0, 1.0, 5.0, 10.0, 10.0, 2.0)   # force xyz, torque xyz in the least-squares fit
    alpha: float = 1e-4      # force regularization
    kp_swing: float = 100.0  # N m/rad, = Menagerie position-servo stiffness
    kd_stance: float = 0.5   # N m s/rad, small joint damping on stance legs


def _rp(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Ry @ Rx


def rigid_body_params(m):
    """Mass, body-frame inertia about the CoM, and CoM offset in the trunk frame, at the standing keyframe."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    mass = M[0, 0]
    I_o = M[3:6, 3:6]                               # rotational inertia about the trunk origin, body frame
    R = quat_to_rot(d.qpos[3:7])
    c = R.T @ (d.subtree_com[m.body("trunk").id] - d.qpos[:3])
    I_c = I_o - mass * (c @ c * np.eye(3) - np.outer(c, c))   # parallel-axis theorem back to the CoM
    return mass, I_c, c


class QPBalanceController:
    def __init__(self, m, gait=GaitParams(), gains=QPGains()):
        self.g = gains
        self.gait = TrotController(gait)
        self.h_des = gait.height
        self.mass, self.I_b, self.c_b = rigid_body_params(m)
        mu = gains.mu
        self.V = np.array([[mu, -mu, 0, 0], [0, 0, mu, -mu], [1, 1, 1, 1.0]])   # cone edges, 3x4
        self.W = np.sqrt(np.array(gains.weights))
        self.q_des = np.zeros(12)
        self.tau_ff = np.zeros(12)
        self.stance = np.ones(4, bool)
        self.f = np.zeros((4, 3))        # last solved forces, heading frame (for logging)

    def plan(self, t, cmd, fb, qenc):
        """500 Hz: swing targets from the trot planner, stance torques from the force QP."""
        v_b, rpy, w_b, h = fb
        g = self.g
        self.q_des = self.gait.joint_targets(t, cmd, v_b, rpy, w_b, h)
        self.stance = self.gait.scheduled_stance(t)
        RH = _rp(rpy[0], rpy[1])                 # body -> level heading frame
        v, w = RH @ v_b, RH @ w_b
        a_des = np.array([g.kv * (cmd[0] - v[0]), g.kv * (cmd[1] - v[1]),
                          g.kpz * (self.h_des - h) - g.kdz * v[2]])
        wd_des = np.array([-g.kr * rpy[0] - g.kdr * w[0], -g.kr * rpy[1] - g.kdr * w[1],
                           g.kdy * (cmd[2] - w[2])])
        legs = np.flatnonzero(self.stance)
        b = np.concatenate([self.mass * (a_des - G_W), RH @ self.I_b @ RH.T @ wd_des])
        A = np.zeros((6, 3 * len(legs)))
        r = []
        for j, leg in enumerate(legs):
            ri = RH @ (K.foot_position(leg, qenc[3 * leg:3 * leg + 3]) - self.c_b)
            r.append(ri)
            A[0:3, 3 * j:3 * j + 3] = np.eye(3)
            A[3:6, 3 * j:3 * j + 3] = skew(ri)
        Vb = np.kron(np.eye(len(legs)), self.V)          # stacked cone edges
        AV = self.W[:, None] * (A @ Vb)
        lhs = np.vstack([AV, np.sqrt(g.alpha) * np.eye(Vb.shape[1])])
        rhs = np.concatenate([self.W * b, np.zeros(Vb.shape[1])])
        beta, _ = nnls(lhs, rhs)
        fH = (Vb @ beta).reshape(-1, 3)
        self.f[:] = 0
        self.tau_ff[:] = 0
        for j, leg in enumerate(legs):
            self.f[leg] = fH[j]
            J = K.foot_jacobian(leg, qenc[3 * leg:3 * leg + 3])
            self.tau_ff[3 * leg:3 * leg + 3] = J.T @ (-(RH.T @ fH[j]))

    def torque(self, q, dq):
        """1 kHz: stance legs get the QP torques, swing legs a joint PD toward the planner's targets."""
        g = self.g
        tau = np.empty(12)
        for leg in range(4):
            s = slice(3 * leg, 3 * leg + 3)
            if self.stance[leg]:
                tau[s] = self.tau_ff[s] - g.kd_stance * dq[s]
            else:
                tau[s] = g.kp_swing * (self.q_des[s] - q[s])
        return tau
