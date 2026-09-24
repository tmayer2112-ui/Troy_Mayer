"""SO(3) / quaternion helpers.

Conventions (identical in cpp/include/qeskf/so3.hpp):
  * Hamilton quaternions stored (w, x, y, z), same as MuJoCo qpos.
  * q_wb rotates body-frame vectors into the world frame: v_w = R(q_wb) v_b.
  * Attitude error is a *local* (right) perturbation: R = R_hat @ Exp(dtheta),
    so dtheta lives in the body frame.
"""
import numpy as np


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(phi):
    """Rodrigues. Exact for all angles, Taylor-expanded near zero."""
    th = np.linalg.norm(phi)
    K = skew(phi)
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K


def log_so3(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-8:
        return 0.5 * w
    if np.pi - th < 1e-6:
        # near pi: use the symmetric part
        B = 0.5 * (R + np.eye(3))
        axis = np.sqrt(np.clip(np.diag(B), 0, None))
        k = int(np.argmax(axis))
        axis = B[k] / axis[k]
        return th * axis / np.linalg.norm(axis)
    return th / (2 * np.sin(th)) * w


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_exp(phi):
    """Unit quaternion for rotation vector phi."""
    th = np.linalg.norm(phi)
    if th < 1e-8:
        q = np.array([1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]])
        return q / np.linalg.norm(q)
    s = np.sin(0.5 * th) / th
    return np.array([np.cos(0.5 * th), s * phi[0], s * phi[1], s * phi[2]])


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def rpy_from_rot(R):
    """ZYX Euler (roll, pitch, yaw) for plotting/reporting only."""
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
