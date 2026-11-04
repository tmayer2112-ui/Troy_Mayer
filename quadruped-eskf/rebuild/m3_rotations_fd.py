"""Module 3: rotations, the gravity leak, and finite differences.

  1. Do WORKSHEET.md, Module 3 on paper first.
  2. Fill in the four TODOs below (about 10 lines).
  3. Run:  python check.py 3      then:  python m3_rotations_fd.py

You'll reuse fd_jacobian in Modules 4 and 5: the checker tests YOUR filter Jacobians with YOUR
finite-difference function, exactly like the GoogleTests in the main project.
"""
import numpy as np

from so3 import exp_so3, quat_exp, quat_mul, rot_from_rpy, skew

G = 9.80665
GRAVITY = np.array([0.0, 0.0, -G])


def fd_jacobian(f, x0, h=1e-6):
    """Measure the Jacobian of f at x0 by central differences.

    f takes a 1-D array and returns a 1-D array. Return J with shape (len(f(x0)), len(x0)),
    where column j = (f(x0 + h e_j) - f(x0 - h e_j)) / (2h) and e_j is all zeros except a 1 at j.
    """
    # TODO (about 7 lines), worksheet question 3.4:
    #   x0 = np.asarray(x0, float);  y0 = np.atleast_1d(f(x0));  J = np.zeros((y0.size, x0.size))
    #   loop over j: build e, nudge up and down, fill column j
    raise NotImplementedError


def correct_by_adding(q, dtheta):
    """What a normal EKF would do: add the small change (0, dtheta/2) to the quaternion. (Wrong!)"""
    # TODO (1 line): q + [0, dtheta_x/2, dtheta_y/2, dtheta_z/2]   (hint: np.concatenate)
    raise NotImplementedError


def correct_by_multiplying(q, dtheta):
    """What the error-state filter does: rotate by Exp(dtheta). Use quat_mul and quat_exp from so3."""
    # TODO (1 line)
    raise NotImplementedError


def gravity_leak_block(R, f, dt):
    """How a small tilt error dtheta (body frame) turns into a velocity error after one step of dt.
    f is the accelerometer reading (body frame). Return the 3x3 block of F. Worksheet question 3.2."""
    # TODO (1 line): uses R, skew(f) and dt
    raise NotImplementedError


def velocity_after_one_step(dtheta, R, f, dt):
    """Velocity after one step from rest, if the orientation estimate is R @ Exp(dtheta). (Provided.)"""
    return ((R @ exp_so3(dtheta)) @ f + GRAVITY) * dt


def demo():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    dth = np.array([0.2, 0.0, 0.0])
    qa, qm = correct_by_adding(q, dth), correct_by_multiplying(q, dth)
    print("1) Correcting a quaternion by 0.2 rad about x")
    print(f"   adding:      {np.round(qa, 4)}   length {np.linalg.norm(qa):.4f}   <- not a rotation any more")
    print(f"   multiplying: {np.round(qm, 4)}   length {np.linalg.norm(qm):.4f}")

    print("\n2) Gravity leak on a level robot standing still: formula vs nudging")
    R, f, dt = np.eye(3), np.array([0.0, 0.0, G]), 1e-3
    A = gravity_leak_block(R, f, dt)
    J = fd_jacobian(lambda th: velocity_after_one_step(th, R, f, dt), np.zeros(3))
    print("   formula block:\n", np.round(A, 6), "\n   finite-difference block:\n", np.round(J, 6))
    print(f"   entry (v_x, tilt_y): formula {A[0, 1]:.8f}, nudged {J[0, 1]:.8f}  (= g * dt)")

    print("\n3) Why tilt and accelerometer bias look the same until the robot turns")
    tilt = np.radians(1.0)
    leak_world = np.array([G * np.sin(tilt), 0.0, 0.0])        # a world-fixed tilt error leaks gravity north
    bias_body = np.array([G * np.sin(tilt), 0.0, 0.0])          # a bias of the same size, glued to the body
    print("   facing      fake accel from 1 deg tilt     fake accel from 0.171 m/s^2 bias")
    for name, yaw in (("north", 0.0), ("east", np.pi / 2)):
        Rz = rot_from_rpy(0.0, 0.0, yaw)
        print(f"   {name:6s}      {np.round(leak_world[:2], 3)}               {np.round((Rz @ bias_body)[:2], 3)}")
    print("   Facing north they are identical. After the turn the bias has turned with the robot; the tilt hasn't.")


if __name__ == "__main__":
    demo()
