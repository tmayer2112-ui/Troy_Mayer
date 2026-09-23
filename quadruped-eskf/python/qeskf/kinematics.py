"""Go1 leg kinematics in the trunk (= IMU) frame.

Geometry is read straight off models/go1/go1.xml (Menagerie). The trunk "imu"
site sits at the trunk origin with identity orientation, so trunk frame, IMU
frame and estimator body frame are the same frame. If the IMU were offset you
would need a lever-arm term in the accelerometer model and an extra fixed
transform here; see README "Assumptions".

Leg order everywhere in this repo: FR, FL, RR, RL (MuJoCo actuator order).
Joint order per leg: abduction (about x), hip (about y), knee (about y).
"""
import numpy as np

LEGS = ("FR", "FL", "RR", "RL")
HIP_OFFSETS = np.array([
    [0.1881, -0.04675, 0.0],
    [0.1881, 0.04675, 0.0],
    [-0.1881, -0.04675, 0.0],
    [-0.1881, 0.04675, 0.0],
])
# Lateral thigh offset: right legs -0.08, left legs +0.08
SIDE = np.array([-1.0, 1.0, -1.0, 1.0])
THIGH_Y = 0.08
L_THIGH = 0.213
L_CALF = 0.213  # hip-to-knee and knee-to-foot-sphere-centre
FOOT_RADIUS = 0.023


def foot_position(leg: int, q: np.ndarray) -> np.ndarray:
    """Foot sphere centre in the trunk frame for one leg, q = (q_abd, q_hip, q_knee)."""
    q1, q2, q3 = q
    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)
    d = SIDE[leg] * THIGH_Y
    x = -L_THIGH * s2 - L_CALF * s23
    z = -L_THIGH * c2 - L_CALF * c23
    return HIP_OFFSETS[leg] + np.array([x, c1 * d - s1 * z, s1 * d + c1 * z])


def foot_jacobian(leg: int, q: np.ndarray) -> np.ndarray:
    """d(foot_position)/dq, 3x3."""
    q1, q2, q3 = q
    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)
    d = SIDE[leg] * THIGH_Y
    z = -L_THIGH * c2 - L_CALF * c23
    dx_dq2 = -L_THIGH * c2 - L_CALF * c23
    dx_dq3 = -L_CALF * c23
    dz_dq2 = L_THIGH * s2 + L_CALF * s23
    dz_dq3 = L_CALF * s23
    J = np.zeros((3, 3))
    J[0] = [0.0, dx_dq2, dx_dq3]
    J[1] = [-s1 * d - c1 * z, -s1 * dz_dq2, -s1 * dz_dq3]
    J[2] = [c1 * d - s1 * z, c1 * dz_dq2, c1 * dz_dq3]
    return J


def all_feet(q12: np.ndarray) -> np.ndarray:
    """(4,3) foot positions from the 12 joint angles."""
    return np.stack([foot_position(i, q12[3 * i:3 * i + 3]) for i in range(4)])


def leg_ik(leg: int, p: np.ndarray, knee_sign: float = -1.0) -> np.ndarray:
    """Analytic IK: trunk-frame foot target -> (q_abd, q_hip, q_knee). Used by the gait controller only."""
    v = p - HIP_OFFSETS[leg]
    d = SIDE[leg] * THIGH_Y
    r_yz2 = v[1] ** 2 + v[2] ** 2
    z = -np.sqrt(max(r_yz2 - d * d, 1e-9))
    q1 = np.arctan2(v[2], v[1]) - np.arctan2(z, d)
    q1 = (q1 + np.pi) % (2 * np.pi) - np.pi
    x = v[0]
    r2 = x * x + z * z
    c3 = np.clip((r2 - L_THIGH ** 2 - L_CALF ** 2) / (2 * L_THIGH * L_CALF), -1.0, 1.0)
    q3 = knee_sign * np.arccos(c3)
    q2 = np.arctan2(-x, -z) - np.arctan2(L_CALF * np.sin(q3), L_THIGH + L_CALF * np.cos(q3))
    return np.array([q1, q2, q3])
