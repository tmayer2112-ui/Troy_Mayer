"""Module 2: position and velocity, with matrices. REFERENCE SOLUTION.

Worksheet: WORKSHEET.md, Module 2. The same four lines of predict/update run inside the full
filter in Module 5; only F, H, Q and R get bigger.
"""
import numpy as np


def make_F(dt):
    """State x = [position, velocity]. Constant velocity over one step of dt."""
    return np.array([[1.0, dt], [0.0, 1.0]])


def predict(x, P, F, Q):
    """Returns the predicted (x, P)."""
    return F @ x, F @ P @ F.T + Q


def update(x, P, z, H, R):
    """Returns the updated (x, P) and the gain K."""
    S = H @ P @ H.T + R                      # how big the residual should be
    K = P @ H.T @ np.linalg.inv(S)           # the gain
    x = x + K @ (z - H @ x)                  # move toward the measurement
    P = (np.eye(len(x)) - K @ H) @ P         # the uncertainty shrinks
    return x, P, K


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
