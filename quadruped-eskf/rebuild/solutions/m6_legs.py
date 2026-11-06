"""Module 6: leg kinematics, the leg Jacobian, and encoder noise. REFERENCE SOLUTION.

The real filter doesn't get the foot position for free: it reads 12 joint encoders and runs forward
kinematics (FK). This module is that step. Same maths as python/qeskf/kinematics.py and kin_cov in
python/qeskf/eskf.py.
"""
import numpy as np

from legs import HIP_OFFSETS, L_CALF, L_THIGH, SIDE, THIGH_Y, encoder_step


def foot_position(leg, q):
    """Body-frame foot position for joint angles q = (abduction, hip, knee)."""
    q1, q2, q3 = q
    d = SIDE[leg] * THIGH_Y
    # 1. the thigh and calf swing in the leg's own plane (hip and knee both turn about y)
    x = -L_THIGH * np.sin(q2) - L_CALF * np.sin(q2 + q3)
    z = -L_THIGH * np.cos(q2) - L_CALF * np.cos(q2 + q3)
    # 2. abduction turns that plane about x; the thigh sits d to the side of the hip
    y_out = np.cos(q1) * d - np.sin(q1) * z
    z_out = np.sin(q1) * d + np.cos(q1) * z
    return HIP_OFFSETS[leg] + np.array([x, y_out, z_out])


def foot_jacobian(leg, q):
    """d(foot_position)/dq, 3 x 3: column j is how the foot moves per radian of joint j."""
    q1, q2, q3 = q
    s1, c1 = np.sin(q1), np.cos(q1)
    d = SIDE[leg] * THIGH_Y
    z = -L_THIGH * np.cos(q2) - L_CALF * np.cos(q2 + q3)
    dx_dq2 = -L_THIGH * np.cos(q2) - L_CALF * np.cos(q2 + q3)
    dx_dq3 = -L_CALF * np.cos(q2 + q3)
    dz_dq2 = L_THIGH * np.sin(q2) + L_CALF * np.sin(q2 + q3)
    dz_dq3 = L_CALF * np.sin(q2 + q3)
    return np.array([[0.0, dx_dq2, dx_dq3],
                     [-s1 * d - c1 * z, -s1 * dz_dq2, -s1 * dz_dq3],
                     [c1 * d - s1 * z, c1 * dz_dq2, c1 * dz_dq3]])


def leg_noise_cov(leg, q, bits=14, model_noise=0.003):
    """Covariance of the FK foot position: encoder rounding pushed through the Jacobian, plus model error.

    Rounding to a step D is uniform on +-D/2, so its variance is D^2 / 12 per joint.
    """
    J = foot_jacobian(leg, q)
    return J @ (np.eye(3) * encoder_step(bits) ** 2 / 12.0) @ J.T + np.eye(3) * model_noise ** 2


def demo():
    from legs import leg_ik, quantize
    q0 = np.zeros(3)
    print("1) Straight leg (all joints 0), FR:", np.round(foot_position(0, q0), 4),
          " = hip + (0, -0.08, -0.426)")
    stand = np.array([0.0, 0.8, -1.6])
    p = foot_position(0, stand)
    print("2) Standing pose (0, 0.8, -1.6), FR foot:", np.round(p, 4), "  IK back:", np.round(leg_ik(0, p), 4))
    print("3) Jacobian at the standing pose (columns: abduction, hip, knee), m/rad:\n", np.round(foot_jacobian(0, stand), 4))
    for bits in (14, 8, 6):
        C = leg_noise_cov(0, stand, bits, model_noise=0.0)
        print(f"4) {bits:2d}-bit encoders: one count = {np.degrees(encoder_step(bits)):.4f} deg,"
              f" foot sigma from rounding up to {1000 * np.sqrt(np.linalg.eigvalsh(C).max()):.3f} mm")
    qq = quantize(stand, 14)
    print("5) Rounding the standing pose to 14-bit counts moves the foot by",
          f"{1000 * np.linalg.norm(foot_position(0, qq) - p):.4f} mm. The 3 mm model error dominates.")


if __name__ == "__main__":
    demo()
