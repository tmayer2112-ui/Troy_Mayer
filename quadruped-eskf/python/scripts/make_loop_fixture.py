"""Write the Python<->C++ fixture for the estimator-in-the-loop controller, used by cpp/tests/test_loop.cpp.

Runs python/qeskf/closed_loop.py with feedback="estimate" (tuned filter, sensor seed 0, headline command
profile) for 2 s of standing calibration plus 2.5 s of walking, and records, for every 1 kHz tick k, what
the C++ EstimatorController::step() receives and what the Python loop produced:

  loop_python.csv  t, acc[k-1], gyro[k-1], qenc[k], force contact[k-1], cmd[k],
                   joint targets[k], estimated velocity after the update at t_k
  loop_config.txt  filter / IMU / gait / loop parameters and the navigation frame's start pose
"""
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import closed_loop as cl  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402
from qeskf.gait import GaitParams  # noqa: E402
from qeskf.imu_model import ImuSpec  # noqa: E402
from qeskf.pipeline import CAL_WINDOW, T_START  # noqa: E402
from qeskf.so3 import quat_to_rot, rpy_from_rot  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "cpp" / "tests" / "data"
WALK_S = 2.5
SEED = 0


def main():
    prm, pol = load_tuned()
    r = cl.run(WALK_S, "estimate", SEED, params=prm, policy=pol)
    assert not r.fell
    L = r.log
    n, k0 = len(L["t"]), L["k0"]
    prev = lambda a: np.vstack([np.zeros((1,) + a.shape[1:]), a[:-1]])  # noqa: E731  sample from tick k-1
    rows = np.column_stack([L["t"], prev(L["acc"]), prev(L["gyro"]), L["qenc"], prev(L["force"]), L["cmd"],
                            L["q_cmd"], L["est_v"]])
    assert np.all(np.isfinite(rows[:, :-3])) and np.all(np.isfinite(rows[k0:]))
    hdr = ("t,ax,ay,az,gx,gy,gz," + ",".join(f"qenc{i}" for i in range(12)) + ",c0,c1,c2,c3,vx_cmd,vy_cmd,wz_cmd,"
           + ",".join(f"q{i}" for i in range(12)) + ",est_vx,est_vy,est_vz")
    OUT.mkdir(parents=True, exist_ok=True)
    np.savetxt(OUT / "loop_python.csv", rows, delimiter=",", header=hdr, comments="", fmt="%.17g")

    imu = ImuSpec()
    cfg = {k: float(getattr(prm, k)) for k in ("acc_noise", "gyro_noise", "acc_rw", "gyro_rw", "foot_noise",
                                               "kin_noise", "enc_lsb", "gate_chi2", "settle_steps")}
    cfg.update(dt=float(L["dt"]), rate_hz=imu.rate_hz, acc_noise_density=imu.acc_noise_density,
               gyro_noise_density=imu.gyro_noise_density, acc_turn_on_bias=imu.acc_turn_on_bias,
               acc_rrw=float(imu.acc_rrw), cal_start=CAL_WINDOW[0], t_start=T_START,
               liftoff_advance=pol.liftoff_advance_ms / 1000.0, use_schedule=float(pol.use_schedule),
               yaw0=float(rpy_from_rot(quat_to_rot(L["quat"][k0]))[2]))
    cfg.update({f"gait_{k}": float(v) for k, v in asdict(GaitParams()).items()})
    lines = [f"{k} {v!r}" for k, v in cfg.items()]
    lines.append("p0 " + " ".join(repr(float(x)) for x in L["p"][k0]))
    (OUT / "loop_config.txt").write_text("\n".join(lines) + "\n")
    walking = L["contact_est"][k0:]
    print(f"wrote loop_python.csv ({n} ticks, filter from tick {k0}) and loop_config.txt; "
          f"estimated-contact fraction per leg while walking {np.nanmean(walking, 0).round(3)}")


if __name__ == "__main__":
    main()
