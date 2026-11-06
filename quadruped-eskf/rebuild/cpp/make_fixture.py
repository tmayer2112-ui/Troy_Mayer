"""Write data/predict_fixture.txt: 1000 IMU samples from the synthetic walk (during the turn) and the
Python reference filter's state every 200 predicts. The C++ parity test replays the same samples.
Run from anywhere:  python cpp/make_fixture.py   (it uses solutions/eskf.py, not your code)"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "solutions"), str(HERE.parent)]
import eskf  # noqa: E402
import simdata  # noqa: E402

d = simdata.make_walk(1)
k0, n, every = 12000, 1000, 200
prm = eskf.Params()
P0 = eskf.initial_covariance(d["spec"], 1500)
f = eskf.Eskf(prm, d["p"][k0], d["v"][k0], d["q"][k0], d["ba"][k0], d["bg"][k0], P0)


def state(f):
    vals = list(f.p) + list(f.v) + list(f.q) + list(f.ba) + list(f.bg) + list(f.P[:15, :15].ravel())
    return " ".join(f"{x:.17g}" for x in vals)


lines = [f"{n} {d['dt']:.17g} {prm.acc_noise:.17g} {prm.gyro_noise:.17g} {prm.acc_rw:.17g} {prm.gyro_rw:.17g}",
         state(f)]
lines += [" ".join(f"{x:.17g}" for x in list(d["acc"][k0 + k]) + list(d["gyro"][k0 + k])) for k in range(n)]
checks = []
for k in range(n):
    f.predict(d["acc"][k0 + k], d["gyro"][k0 + k], d["dt"])
    if (k + 1) % every == 0:
        checks.append(f"{k + 1} {state(f)}")
lines += [str(len(checks))] + checks
(HERE / "data").mkdir(exist_ok=True)
(HERE / "data" / "predict_fixture.txt").write_text("\n".join(lines) + "\n")
print(f"wrote {HERE / 'data' / 'predict_fixture.txt'}")
