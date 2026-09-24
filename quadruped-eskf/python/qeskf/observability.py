"""Numerical check of the analytic observability result.

Claim (derived in README "Observability"): for the contact-aided system with a fixed set of
stance feet, the unobservable subspace of the linearized error system is spanned by
  * 3 global translations:  dp = dd_i = e_j,  everything else 0
  * 1 rotation about gravity (yaw): dp = [e_z]x p, dv = [e_z]x v, dtheta = R^T e_z, dd_i = [e_z]x d_i
and everything else (velocity, roll, pitch, both biases) is observable given enough motion.

Test: linearize along a trajectory that satisfies the filter's own discrete dynamics, stack the
local observability matrix O = [H_0; H_1 F_0; H_2 F_1 F_0; ...] over a stance window, and check
  (a) O N = 0 to machine precision for the 4 analytic directions N, and
  (b) the SVD of O (column-scaled) has exactly 4 singular values at numerical zero.
"""
import numpy as np

from . import kinematics as K
from .eskf import IBA, IBG, IP, ITH, IV, NB, Eskf, EskfParams
from .so3 import quat_to_rot, skew


def stance_window(contact, k_start, min_len):
    """First index >= k_start where the same set of >= 2 feet stays in contact for min_len samples."""
    n = len(contact)
    k = k_start
    while k + min_len < n:
        s = contact[k]
        if s.sum() >= 2 and np.all(contact[k:k + min_len] == s):
            return k, [int(i) for i in np.where(s)[0]]
        k += 1
    raise ValueError("no stance window found")


def analytic_nullspace(f: Eskf):
    n = f.dim
    R = quat_to_rot(f.x.q)
    N = np.zeros((n, 4))
    for j in range(3):
        N[IP + j, j] = 1.0
        for i in range(len(f.x.order)):
            N[NB + 3 * i + j, j] = 1.0
    ez = np.array([0.0, 0.0, 1.0])
    N[IP:IP + 3, 3] = skew(ez) @ f.x.p
    N[IV:IV + 3, 3] = skew(ez) @ f.x.v
    N[ITH:ITH + 3, 3] = R.T @ ez
    for i, leg in enumerate(f.x.order):
        N[NB + 3 * i:NB + 3 * i + 3, 3] = skew(ez) @ f.x.feet[leg]
    return N


def observability_matrix(log, k0, legs, n_steps, use_true_imu=True):
    """Stacked local observability matrix over [k0, k0+n_steps) and the analytic N at k0.

    The nominal trajectory is produced by the filter's own propagation from the true state at k0
    driven by the ideal IMU, so every F_k and H_k is evaluated on one self-consistent trajectory.
    Foot positions are initialised from forward kinematics at k0 (exactly consistent at k0; the
    kinematic *residual* doesn't enter O, only the Jacobians do).
    """
    acc = log["acc_true"] if use_true_imu else log["acc"]
    gyr = log["gyro_true"] if use_true_imu else log["gyro"]
    f = Eskf(EskfParams(), p=log["p"][k0], v=log["v"][k0], q=log["quat"][k0], ba=np.zeros(3), bg=np.zeros(3),
             P0=np.eye(NB))
    for leg in legs:
        f.augment(leg, log["qenc"][k0])
    N0 = analytic_nullspace(f)
    Phi = np.eye(f.dim)
    blocks = []
    for j in range(n_steps):
        k = k0 + j
        _, H, _ = f.measurement(legs, log["qenc"][k])
        blocks.append(H @ Phi)
        F = f.error_transition(acc[k], gyr[k], log["dt"])
        f.propagate_nominal(acc[k], gyr[k], log["dt"])
        Phi = F @ Phi
    return np.vstack(blocks), N0


def analyze(log, contact, k_start, n_steps=150):
    k0, legs = stance_window(contact, k_start, n_steps)
    O, N = observability_matrix(log, k0, legs, n_steps)
    # scale columns so position/velocity/angle/bias units don't decide the rank
    scale = np.linalg.norm(O, axis=0)
    scale[scale == 0] = 1.0
    Os = O / scale
    sv = np.linalg.svd(Os, compute_uv=False)
    rel_null = np.linalg.norm(O @ N, axis=0) / (np.linalg.norm(O) * np.linalg.norm(N, axis=0))
    return dict(k0=int(k0), legs=legs, dim=int(O.shape[1]), rows=int(O.shape[0]),
                singular_values=(sv / sv[0]).tolist(), rel_ON=rel_null.tolist(),
                n_numerically_zero=int(np.sum(sv / sv[0] < 1e-9)))
