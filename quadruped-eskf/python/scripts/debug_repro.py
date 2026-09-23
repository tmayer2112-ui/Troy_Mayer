"""Re-create the failures in DEBUG_LOG.md on demand.

  PYTHONPATH=python python3 python/scripts/debug_repro.py 5      # one entry
  PYTHONPATH=python python3 python/scripts/debug_repro.py all    # every scripted entry

Each function puts the bug back (or re-runs the measurement that exposed it) and prints what
the log says you should see. Entries 4 and 7 need `make data` first.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf import sim  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402
from qeskf.eskf import EskfParams  # noqa: E402
from qeskf.so3 import exp_so3, log_so3, quat_to_rot  # noqa: E402

G = np.array([0.0, 0.0, -9.80665])


def _accel_residual(log):
    ks = np.arange(2500, len(log["t"]) - 10)
    R = np.array([quat_to_rot(q) for q in log["quat"][ks]])
    a_fd = (log["v"][ks + 1] - log["v"][ks]) / log["dt"]
    a_imu = np.einsum("nij,nj->ni", R, log["acc_true"][ks]) + G
    return np.sqrt(np.mean((a_fd - a_imu) ** 2, axis=0))


def _with_model_patch(patch, duration=4.0):
    orig = sim.load_model

    def patched(ff=None):
        m = orig(ff)
        patch(m)
        return m
    sim.load_model = patched
    try:
        return sim.simulate(duration)
    finally:
        sim.load_model = orig


def e1():
    print("#1 body-frame v_z sampled at whole seconds vs at arbitrary times (gait period 0.40 s)")
    log = sim.simulate(12.0)
    R = np.array([quat_to_rot(q) for q in log["quat"]])
    vb = np.einsum("nji,nj->ni", R, log["v"])
    whole = [int(k * 1000) for k in range(5, 12)]
    print("  at t = 5,6,..,11 s:", np.round(vb[whole, 2], 3), "  <- same gait phase every time")
    print("  mean over 5-12 s: %.4f m/s" % vb[5000:, 2].mean(), "  <- the trunk is not falling")


def e4():
    print("#4 vertical drift vs how early the stance window ends (untuned contact model, headline walk)")
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    prm = EskfParams.from_imu_spec(pl.ImuSpec(), foot_noise=0.01, kin_noise=0.003, settle_steps=0)
    for adv in (0, 10, 20, 30):
        est = pl.run_cpp(log, params=prm, contact=pl.estimator_contacts(log, liftoff_advance_ms=adv))
        dz = pl.errors(log, est)["dp"][-1, 2]
        print(f"  liftoff advanced {adv:2d} ms -> final z error {dz:+.3f} m")
    print("  sign flips as the window moves: a contact-model bias, not noise. (Numbers in the log were taken"
          " before fixes #5/#6, so they differ; the sign flip is the point.)")


def e5():
    print("#5 accelerometer vs d(v)/dt with MuJoCo's implicit joint damping switched back on")
    on = _with_model_patch(lambda m: setattr(m.opt, "disableflags", m.opt.disableflags & ~int(mujoco.mjtDisableBit.mjDSBL_EULERDAMP)))
    off = sim.simulate(4.0)
    print("  eulerdamp ON  (MuJoCo default): rms residual", np.round(_accel_residual(on), 4), "m/s^2")
    print("  eulerdamp OFF (this repo)     : rms residual", _accel_residual(off), "m/s^2")


def e6():
    print("#6 MuJoCo default gravity (9.81) vs the filter's 9.80665")
    lg = _with_model_patch(lambda m: m.opt.__setattr__("gravity", np.array([0.0, 0.0, -9.81])))
    print("  z residual:", np.round(_accel_residual(lg)[2], 5), "m/s^2  (9.81 - 9.80665 = 0.00335)")


def e7():
    print("#7 pitch error before vs after the first turn (tuned filter, headline walk)")
    prm, pol = load_tuned()
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    est = pl.run_cpp(log, params=prm, contact=pl.estimator_contacts(log, liftoff_advance_ms=pol.liftoff_advance_ms))
    e = pl.errors(log, est)
    for a, b in ((0, 5), (5, 12), (12, 20), (20, 60)):
        m = (e["t"] >= a) & (e["t"] < b)
        print(f"  {a:2d}-{b:2d} s: pitch RMSE {np.degrees(np.sqrt(np.mean(e['drpy'][m, 1] ** 2))):.3f} deg, "
              f"mean accel-x-bias error {e['dba'][m, 0].mean():+.4f} m/s^2")
    print("  first commanded turn starts at t = 10 s on this clock")


def e8():
    print("#8 NumPy 2 repr in a text config file")
    x = np.float64(1.6794438557724239e-06)
    print(f"  f'{{x!r}}'        -> {x!r}     <- what the C++ parser choked on")
    print(f"  f'{{float(x)!r}}' -> {float(x)!r}")


def e10():
    print("#10 SO(3) log(exp(phi)) for |phi| > pi")
    phi = np.array([2.0, 2.9, 0.41])
    back = log_so3(exp_so3(phi))
    print("  |phi| =", round(float(np.linalg.norm(phi)), 3), " log(exp(phi)) =", np.round(back, 3),
          " |.| =", round(float(np.linalg.norm(back)), 3), "= 2*pi - |phi|: same rotation, shorter vector")


ENTRIES = {"1": e1, "4": e4, "5": e5, "6": e6, "7": e7, "8": e8, "10": e10}

if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    for k in (ENTRIES if which == "all" else [which]):
        ENTRIES[k]()
        print()
