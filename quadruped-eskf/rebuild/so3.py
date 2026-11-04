"""Rotation helpers (provided, you don't write these).

Same conventions as the main project (python/qeskf/so3.py):
  * quaternions are (w, x, y, z), Hamilton convention
  * R = quat_to_rot(q) turns a body-frame vector into a world-frame vector: v_world = R @ v_body
  * the attitude error is a small rotation in the body frame: R_true = R_est @ Exp(dtheta)
"""
import numpy as np


def skew(v):
    """[v]x, the matrix that does a cross product: skew(a) @ b == np.cross(a, b)."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(phi):
    """Rotation vector (axis * angle, radians) -> 3x3 rotation matrix (Rodrigues' formula)."""
    th = np.linalg.norm(phi)
    K = skew(phi)
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K


def log_so3(R):
    """3x3 rotation matrix -> rotation vector. The inverse of exp_so3 (for angles below pi)."""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-8:
        return 0.5 * w
    return th / (2 * np.sin(th)) * w


def right_jacobian(phi):
    """J_r(phi). How a small change in a rotation vector shows up as a rotation in the body frame.
    For small phi it is close to the identity: J_r ~ I - skew(phi) / 2."""
    th = np.linalg.norm(phi)
    K = skew(phi)
    if th < 1e-6:
        return np.eye(3) - 0.5 * K + K @ K / 6.0
    return np.eye(3) - (1 - np.cos(th)) / th ** 2 * K + (th - np.sin(th)) / th ** 3 * K @ K


def quat_mul(a, b):
    """Quaternion product a (x) b: first rotate by a, then by b in a's frame."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_exp(phi):
    """Rotation vector -> unit quaternion."""
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


def rot_from_rpy(roll, pitch, yaw):
    """Rotation matrix from roll (about x), pitch (about y), yaw (about z): R = Rz @ Ry @ Rx."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rpy_from_rot(R):
    """(roll, pitch, yaw) in radians, for reporting only."""
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi
