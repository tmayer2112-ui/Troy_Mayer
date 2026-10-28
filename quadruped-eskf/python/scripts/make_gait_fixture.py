"""Write the Python<->C++ fixture for the trot controller, used by cpp/tests/test_gait.cpp.

Feeds python/qeskf/gait.py a deterministic 5 s input sequence at 500 Hz and stores every input and every
output (12 joint targets, scheduled stance). The inputs are synthetic, not a simulator run, so the fixture
needs no dataset: 0.5 s standing, then 4.5 s of trotting with forward, sideways and turning commands
changing all the time, measured velocity off the command (exercises Raibert placement), tilt and rates
(attitude correction), and a height signal. The last 0.5 s pass no height (the height=None path).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf.gait import GaitParams, TrotController  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "cpp" / "tests" / "data" / "gait_python.csv"
T0, T1, DT = 1.5, 6.5, 0.002       # stand_time is 2.0, so 0.5 s standing first
NO_HEIGHT_FROM = 6.0


def inputs(t):
    w = 2 * np.pi
    cmd = np.array([0.25 + 0.2 * np.sin(w * t / 3.1), 0.15 * np.sin(w * t / 2.3), 0.5 * np.sin(w * t / 1.7)])
    v_b = cmd + np.array([0.05 * np.sin(w * t / 0.7), -0.04 * np.cos(w * t / 0.9), 0.01 * np.sin(w * t / 0.4)])
    rpy = np.array([0.04 * np.sin(w * t / 1.3), -0.03 * np.cos(w * t / 1.1), 0.3 * t])
    om = np.array([0.3 * np.cos(w * t / 0.5), -0.25 * np.sin(w * t / 0.6), cmd[2]])
    h = 0.27 + 0.01 * np.sin(w * t / 0.8) - 0.005
    return cmd, v_b, rpy, om, (None if t >= NO_HEIGHT_FROM else h)


def main():
    ctrl = TrotController(GaitParams())
    rows = []
    for k in range(int(round((T1 - T0) / DT))):
        t = T0 + k * DT
        cmd, v_b, rpy, om, h = inputs(t)
        q = ctrl.joint_targets(t, cmd, v_b, rpy, om, h)
        rows.append(np.r_[t, cmd, v_b, rpy, om, np.nan if h is None else h, q, ctrl.scheduled_stance(t)])
    hdr = ("t,vx_cmd,vy_cmd,wz_cmd,vx,vy,vz,roll,pitch,yaw,wx,wy,wz,height,"
           + ",".join(f"q{i}" for i in range(12)) + ",st0,st1,st2,st3")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(OUT, np.array(rows), delimiter=",", header=hdr, comments="", fmt="%.17g")
    st = np.array(rows)[:, -4:]
    print(f"wrote {OUT.name}: {len(rows)} rows; stance fraction per leg while walking "
          f"{st[np.array(rows)[:, 0] >= 2.0].mean(0).round(3)}")


if __name__ == "__main__":
    main()
