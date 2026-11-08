"""Write data/pi_fixture.txt: the Python reference loop (solutions/m6_speed_loop.py) run on the toy arm in
three situations, with every tick's inputs and outputs. The C++ parity test feeds the same inputs to your
C++ VelocityPI and compares the outputs.
Run from anywhere:  python cpp/make_fixture.py   (it uses solutions/, not your code)

  scenario 0: lift the forearm at 15 deg/s from 5 deg, 1.5 s (the ramp, feed-forward and integral)
  scenario 1: a 20 deg/s command with the joint held for 0.5 s, anti-windup on (saturation)
  scenario 2: the same with anti-windup off
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (str(HERE.parent / "solutions"), str(HERE.parent)):
    if p not in sys.path:
        sys.path.insert(0 if p.endswith("solutions") else len(sys.path), p)
import m6_speed_loop as M  # noqa: E402

DT = 1e-3
rows = []
for scenario, (antiwindup, theta0, speed, block_s, seconds) in enumerate(
        ((True, 5.0, 15.0, None, 1.5), (True, 20.0, 20.0, 0.5, 1.5), (False, 20.0, 20.0, 0.5, 1.5))):
    ctrl, arm, ramp = M.VelocityPI(antiwindup=antiwindup), M.ToyArm(theta0), M.Ramp(150.0)
    arm.blocked = block_s is not None
    for k in range(int(round(seconds / DT))):
        if block_s is not None and k * DT >= block_s:
            arm.blocked = False
        ref, ramping = ramp.step(speed, DT)
        meas = arm.omega
        duty = ctrl.step(ref, meas, DT, ramping)
        rows.append(f"{scenario} {int(antiwindup)} {ref:.17g} {meas:.17g} {int(ramping)} {duty:.17g} {ctrl.integ:.17g}")
        arm.step(duty, DT)

(HERE / "data").mkdir(exist_ok=True)
out = HERE / "data" / "pi_fixture.txt"
out.write_text(f"{len(rows)} {DT:.17g}\n" + "\n".join(rows) + "\n")
print(f"wrote {out}: {len(rows)} ticks")
