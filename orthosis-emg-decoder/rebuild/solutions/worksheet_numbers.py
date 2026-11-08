"""Every number in ANSWERS.md that isn't a run.py output, recomputed (maintainers: run this after any change).

    python solutions/worksheet_numbers.py
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (str(HERE), str(HERE.parent)):
    if p not in sys.path:
        sys.path.insert(0 if p == str(HERE) else len(sys.path), p)

import m1_features as F  # noqa: E402
import m2_filter as M2  # noqa: E402
import m3_frontend as M3  # noqa: E402
import m5_polarity as M5  # noqa: E402
import m6_speed_loop as M6  # noqa: E402
import m7_bayes as M7  # noqa: E402
import m11_mujoco as M11  # noqa: E402


def show(label, v):
    print(f"  {label:58s} {v}")


print("Module 1")
show("window / step at 2048 Hz, ms", (round(400 / 2048 * 1e3, 1), round(100 / 2048 * 1e3, 1)))
w = np.array([[2.0], [-1.0], [-3.0], [1.0], [0.5], [-0.5]])[None]
show("1.2 MAV, WL, ZC, SSC (dead band 1.5)", (round(F.mav(w)[0, 0], 4), F.wl(w)[0, 0], F.zc(w, 1.5)[0, 0], F.ssc(w, 1.5)[0, 0]))
show("1.2 ZC without the dead band", F.zc(w, 0.0)[0, 0])
show("1.3 ln 2", round(math.log(2), 4))
show("1.5 windows in a 3 s hold (6144 samples)", (6144 - 400) // 100 + 1)

print("Module 2")
K = M2.K_SK
show("2.1 K, Q, Butterworth K, R for it (kOhm)", (round(K, 4), round(1 / (3 - K), 4), round(3 - math.sqrt(2), 4),
                                                  round(10 / (3 - math.sqrt(2) - 1), 2)))
show("2.1 RC (ms) and f0 (Hz), HPF / LPF", ((15e3 * 470e-9 * 1e3, round(M2.stages()[0][0], 2)),
                                            (1e3 * 330e-9 * 1e3, round(M2.stages()[1][0], 1))))
Q = 1 / (3 - K)
show("2.2 K Q", round(K * Q, 4))
b = 2 - 1 / Q ** 2
u = (-b + math.sqrt(b * b + 4)) / 2
show("2.3 2 - 1/Q^2, Omega^2, Omega, HPF corner, LPF corner", (round(b, 4), round(u, 4), round(math.sqrt(u), 4),
                                                              round(M2.stages()[0][0] * math.sqrt(u), 2),
                                                              round(M2.stages()[1][0] / math.sqrt(u), 2)))
fs, f0 = 2048.0, M2.stages()[1][0]
w0p = 2 * fs * math.tan(math.pi * f0 / fs)
c = 2 * fs
A, B, C = c * c, w0p / Q * c, w0p * w0p
show("2.4 LPF w0, pre-warped w0, ratio", (round(2 * math.pi * f0), round(w0p), round(w0p / (2 * math.pi * f0), 3)))
show("2.4 A, B, C (millions)", (round(A / 1e6, 2), round(B / 1e6, 2), round(C / 1e6, 2)))
bb, aa = M2.bilinear_biquad(f0, Q, K, "low", fs)
show("2.4 b0 b1 b2, a1 a2", (np.round(bb, 4).tolist(), np.round(aa[1:], 4).tolist()))
show("2.5 LSB mV, noise SD mV", (round(3.1 / 4096 * 1e3, 3), round(3.1 / 1024 / math.sqrt(12) * 1e3, 3)))
f = np.array([500.0, 700.0, 1000.0])
show("2.6 band-pass at 500/700/1000 Hz, dB from the peak",
     np.round(20 * np.log10(np.abs(M2.cascade_analog(f)) / 2.4048), 1).tolist())

print("Module 3")
a = 47 / (1 + 22.5 + 47) * (1 + 47 / (1 + 17.5))
bm = 47 / 18.5
show("3.1 a, b, G_diff, G_cm, CMRR ratio, dB", (round(a, 3), round(bm, 3), round((a + bm) / 2, 3), round(a - bm, 3),
                                                round((a + bm) / 2 / abs(a - bm), 1),
                                                round(20 * math.log10((a + bm) / 2 / abs(a - bm)), 1)))
r = abs(a - bm) / ((a + bm) / 2) + 10 ** (-62 / 20)
show("3.2 1/CMRR with the 62 dB term, dB", (round(r, 4), round(-20 * math.log10(r), 1)))
show("3.3 decay per sample at 2048 Hz, bandwidth Hz", (round(math.exp(-1 / (2048 * 0.18)), 5), round(1 / (2 * math.pi * 0.18), 2)))
g = (a + bm) / 2
show("3.4 total v1 gain, input to clear the diode (uV)", (round(g * K * K * 148, 1), round(0.4 / (g * K * K * 148) * 1e6, 1)))
show("3.4 5k / Z_cm, dB; + INA; / RLD", (f"{5e3 / M3.Z_CM:.2e}", round(M3.db(5e3 / M3.Z_CM), 1),
                                        round(M3.db(1e-5 + 5e3 / M3.Z_CM), 1), round(M3.db(M3.v2_cm_to_diff(5e3)), 1)))
show("3.5 10 mV leak (mV), at the diode (V)", (round(0.01 * 10 ** (-22.6 / 20) * 1e3, 2), round(M3.hum_at_rectifier(0.01, 22.6, 877.5), 2)))

print("Module 4")
show("4.2 chance with 11 classes, %", round(100 / 11, 1))
show("4.4 P(no training neighbour), leaky share %", (round(0.25 ** 6, 6), round(100 * (1 - 0.25 ** 6), 2)))

print("Module 5")
show("5.1 skewness [0,0,3], RMS", (round(float(M5.skewness(np.array([[0.0], [0.0], [3.0]]))[0]), 4), round(math.sqrt(3), 3)))
show("5.2 beta", round(0.52 / 0.51, 4))

print("Module 6")
kp, ki = M6.imc_gains(187, 0.02, 0.05)
show("6.1 speed at d = 0.1 (deg/s); ff at 15 deg/s; ff at 10 deg/s + 150 deg/s^2",
     (18.7, round(M6.feedforward(15, 0), 4), round(M6.feedforward(10, 150), 4)))
show("6.2 Kp, Ki (conservative); Kp, Ki (lambda 30 ms)", (round(kp, 6), round(ki, 4),
                                                          tuple(round(v, 5) for v in M6.imc_gains(187, 0.02, 0.03))))
c6 = M6.VelocityPI()
c6.prev_ref = 10.0
out = c6.step(10.0, 8.0, 0.001)
show("6.3 ff, P term, duty, integral", (round(10 / 187 + 0.05, 6), round(kp * 2, 6), round(out, 6), round(c6.integ, 6)))
hold = 0.136 * math.sin(math.radians(85))
show("6.4 duty to hold at 40 deg; sag before the integral holds (deg); toy arm's settled angle",
     (round(hold, 3), round((hold - 0.027) / ki, 2), round(float(M6.hold(True)[0]["theta"][500]), 2)))
cps = 25 / 0.06698
cnt = cps * 0.01
p = cnt - math.floor(cnt)
show("6.5 counts/s and per 10 ms at 25 deg/s; RMS error", (round(cps), round(cnt, 2), round(math.sqrt(p * (1 - p)) / cnt, 3)))
cnt3 = 3 / 0.06698 * 0.01
show("6.5 at 3 deg/s: per 10 ms, RMS error", (round(cnt3, 3), round(math.sqrt(cnt3 * (1 - cnt3)) / cnt3, 2)))
show("6.6 ff at 15 deg/s minus gravity at 50 deg", round(M6.feedforward(15, 0) - 0.136 * math.sin(math.radians(50)), 4))

print("Module 7")
lv = M7.levels()
show("7.1 ratio between levels", round(lv[1] / lv[0], 4))
q, est = M7.bayes_step(np.array([0.2, 0.6, 0.2]), 2.0, np.array([1.0, 2.0, 4.0]), 0.1, 0.0)
like = np.exp(-math.sqrt(2) * 2 / np.array([1.0, 2.0, 4.0])) / np.array([1.0, 2.0, 4.0])
show("7.2 likelihoods, posterior, estimate", (np.round(like, 4).tolist(), np.round(q, 4).tolist(), round(est, 3)))
T = 1 / (2 * math.pi * 4)
show("7.3 two-pole T (ms), 50 % of a step (ms)", (round(1e3 * T, 1), round(1.678 * 1e3 * T, 1)))

print("Module 11")
show("11.1 torque at 50 % duty, stall (N m)", round(M11.motor_torque(0.5, 0.0), 2))
show("11.1 back-EMF damping, N m per rad/s", round(M11.ETA * M11.N ** 2 * M11.KE ** 2 / M11.R_OHM, 2))
show("11.1 armature (reflected rotor inertia), kg m^2", round(M11.J_MOTOR * M11.N ** 2, 3))
