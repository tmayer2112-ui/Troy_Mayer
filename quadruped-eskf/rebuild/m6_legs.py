"""Module 6: leg kinematics, the leg Jacobian, and encoder noise.

  1. Do WORKSHEET.md, Module 6 on paper first.
  2. Fill in the three TODOs below (about 18 lines).
  3. Run:  python check.py 6   then:  python m6_legs.py   then:  python run.py legs

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
    # TODO (about 5 lines), worksheet 6.1-6.2:
    #   1. x and z of the two-link leg in its own plane (hip angle q2, knee angle q3; links L_THIGH, L_CALF)
    #   2. turn that plane about x by the abduction angle q1, with the thigh d to the side of the hip
    #   3. return HIP_OFFSETS[leg] + np.array([x, y, z])
    raise NotImplementedError


def foot_jacobian(leg, q):
    """d(foot_position)/dq, 3 x 3: column j is how the foot moves per radian of joint j."""
    # TODO (about 10 lines), worksheet 6.3: differentiate your foot_position by q1, q2 and q3.
    # Column j is d(foot)/d(q_j). Test yourself: fd_jacobian from Module 3 must give the same matrix.
    raise NotImplementedError


def leg_noise_cov(leg, q, bits=14, model_noise=0.003):
    """Covariance of the FK foot position: encoder rounding pushed through the Jacobian, plus model error.

    Rounding to a step D is uniform on +-D/2, so its variance is D^2 / 12 per joint.
    """
    # TODO (2 lines), worksheet 6.4: J (step^2 / 12) J^T + model_noise^2 I,  with step = encoder_step(bits)
    raise NotImplementedError


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
