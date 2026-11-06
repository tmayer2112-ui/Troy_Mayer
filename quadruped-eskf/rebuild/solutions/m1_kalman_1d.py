"""Module 1: the Kalman filter in one dimension. REFERENCE SOLUTION.

Plain Python, no libraries. Worksheet: WORKSHEET.md, Module 1.
"""
import random


def kalman_gain(P, R):
    """How far to move toward the measurement. P = prediction variance, R = measurement variance."""
    return P / (P + R)


def predict_1d(x, P, u, q):
    """Move the estimate by u (e.g. odometry: velocity * dt). The uncertainty grows by q."""
    return x + u, P + q


def update_1d(x, P, z, R):
    """Blend in a measurement z that has variance R. Returns the new (x, P)."""
    K = kalman_gain(P, R)
    x = x + K * (z - x)
    P = (1 - K) * P
    return x, P


def demo():
    """A robot walks along a line at 0.5 m/s. Odometry says how far it moved each 0.1 s step
    (a little wrong every time). Every 5th step a position sensor reports where it is (noisy).
    Watch P: it grows on every predict and drops on every update."""
    random.seed(1)
    q, R = 0.03 ** 2, 0.1 ** 2          # odometry error variance per step, sensor variance
    true_x, x, P = 0.0, 0.0, 0.0
    print(" step   true x   estimate      P      note")
    for k in range(1, 21):
        true_x += 0.05
        x, P = predict_1d(x, P, 0.05 + random.gauss(0, 0.03), q)
        note = ""
        if k % 5 == 0:
            z = true_x + random.gauss(0, 0.1)
            P_before = P
            x, P = update_1d(x, P, z, R)
            note = f"measured z = {z:.3f}, K = {kalman_gain(P_before, R):.3f}"
        print(f"{k:5d} {true_x:8.3f} {x:10.3f} {P:9.5f}   {note}")


if __name__ == "__main__":
    demo()
