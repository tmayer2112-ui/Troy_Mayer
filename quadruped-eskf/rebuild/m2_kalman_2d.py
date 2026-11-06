"""Module 2: position and velocity, with matrices.

  1. Do WORKSHEET.md, Module 2 on paper first (you'll compute one predict and one update by hand).
  2. Fill in the three TODOs below (about 7 lines).
  3. Run:  python check.py 2      then:  python m2_kalman_2d.py

These same few lines run inside the full filter in Module 5. Only F, H, Q and R get bigger.
Numpy reminders: A @ B is a matrix product, A.T is a transpose, np.linalg.inv(A) is an inverse,
np.eye(n) is the n x n identity.
"""
import numpy as np


def make_F(dt):
    """State x = [position, velocity]. Constant velocity over one step of dt. Return a 2x2 array."""
    # TODO (1 line), worksheet question 2.1
    raise NotImplementedError


def predict(x, P, F, Q):
    """Return the predicted (x, P)."""
    # TODO (1-2 lines), worksheet question 2.2
    raise NotImplementedError


def update(x, P, z, H, R):
    """Return the updated (x, P) and the gain K.
    z: measurement vector, H: which part of the state it measures, R: its noise covariance."""
    # TODO (4-5 lines), worksheet question 2.3:
    #   S = how big the residual should be
    #   K = the gain
    #   x = move toward the measurement by K times the residual (z - H x)
    #   P = shrink it with (I - K H)
    raise NotImplementedError


def demo(seed=0):
    """A cart rolls at about 1 m/s. We only ever measure its POSITION (sigma = 0.5 m, every 0.1 s).
    Velocity is never measured, yet the filter works it out, through the p-v correlation in P."""
    rng = np.random.default_rng(seed)
    dt = 0.1
    F = make_F(dt)
    Q = np.diag([0.0, 0.05 ** 2 * dt])       # the cart's speed wanders a little
    H = np.array([[1.0, 0.0]])
    R = np.array([[0.5 ** 2]])
    x_true = np.array([0.0, 1.0])
    x, P = np.array([0.0, 0.0]), np.diag([1.0, 4.0])   # we start not knowing the speed at all
    print("    t   true v   est v   sigma_v   corr(p, v)")
    for k in range(1, 101):
        x_true = F @ x_true + np.array([0.0, rng.normal(0, 0.05 * np.sqrt(dt))])
        x, P = predict(x, P, F, Q)
        z = np.array([x_true[0] + rng.normal(0, 0.5)])
        x, P, K = update(x, P, z, H, R)
        if k % 10 == 0:
            corr = P[0, 1] / np.sqrt(P[0, 0] * P[1, 1])
            print(f"{k * dt:5.1f} {x_true[1]:8.3f} {x[1]:7.3f} {np.sqrt(P[1, 1]):9.3f} {corr:11.3f}")
    return x_true, x


if __name__ == "__main__":
    demo()
