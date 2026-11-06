"""Go1 leg geometry and inverse kinematics (provided, you don't write these).

Copied from the main project (python/qeskf/kinematics.py), which reads them off the MuJoCo Go1 model.
Leg order FR, FL, RR, RL. Joint order per leg: abduction (about x), hip (about y), knee (about y).
You write the forward kinematics, its Jacobian and the encoder-noise covariance in m6_legs.py.
"""
import numpy as np

HIP_OFFSETS = np.array([[0.1881, -0.04675, 0.0],
                        [0.1881, 0.04675, 0.0],
                        [-0.1881, -0.04675, 0.0],
                        [-0.1881, 0.04675, 0.0]])   # hip joint positions in the body frame, m
SIDE = np.array([-1.0, 1.0, -1.0, 1.0])            # right legs -1, left legs +1
THIGH_Y = 0.08                                      # sideways offset from the hip to the thigh, m
L_THIGH = 0.213                                     # hip to knee, m
L_CALF = 0.213                                      # knee to foot centre, m
ENCODER_BITS = 14                                   # the real project's joint encoders


def leg_ik(leg, p, knee_sign=-1.0):
    """Body-frame foot position -> joint angles (abduction, hip, knee). The inverse of your foot_position."""
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


def encoder_step(bits=ENCODER_BITS):
    """Size of one encoder count, in radians."""
    return 2 * np.pi / 2 ** bits


def quantize(q, bits=ENCODER_BITS):
    """What an encoder reports: the angle rounded to whole counts."""
    step = encoder_step(bits)
    return np.round(np.asarray(q) / step) * step


def joint_angles(legs):
    """(n, 4, 3) body-frame foot vectors -> (n, 4, 3) joint angles, by inverse kinematics."""
    q = np.zeros_like(legs)
    for k in range(legs.shape[0]):
        for i in range(4):
            q[k, i] = leg_ik(i, legs[k, i])
    return q
