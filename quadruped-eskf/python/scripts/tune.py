"""Choose the contact-model parameters on the TUNING trajectory only.

What is tuned (and why it has to be): the leg-kinematics error model. There is no datasheet
for "how much does a rubber foot on a soft floor move during stance".
  foot_noise          world-frame random walk of a planted foot (m/sqrt(s)): rolling + compression
  kin_noise           per-sample FK error (m)
  liftoff_advance_ms  end the stance window early, to skip the unloading phase (DEBUG_LOG #4)
  settle_ms           wait after touchdown before adding the foot (DEBUG_LOG #3)
What is NOT tuned: every IMU term (datasheet, imu_model.py), gravity, encoder resolution.

Selection rule, fixed before running: lowest world-frame 3D velocity RMSE on data/tuning.npz.
The headline trajectory is never loaded here.
"""
import itertools
import json
import os
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf.config import ContactPolicy, save_tuned  # noqa: E402
from qeskf.eskf import EskfParams  # noqa: E402
from qeskf.imu_model import ImuSpec  # noqa: E402

GRID = dict(
    foot_noise=[0.001, 0.003, 0.01, 0.03, 0.1],
    kin_noise=[0.001, 0.003, 0.01],
    liftoff_advance_ms=[0, 5, 10, 15, 20],
    settle_ms=[0, 10],
)
_LOG = None


def _init():
    global _LOG
    _LOG = pl.load_dataset(pl.ROOT / "data" / "tuning.npz")


def evaluate(combo):
    foot, kin, adv, settle = combo
    prm = EskfParams.from_imu_spec(ImuSpec(), foot_noise=foot, kin_noise=kin, settle_steps=int(settle))
    c = pl.estimator_contacts(_LOG, liftoff_advance_ms=adv)
    with tempfile.TemporaryDirectory() as wd:
        est = pl.run_cpp(_LOG, params=prm, contact=c, workdir=wd)
    s = pl.summarize(_LOG, est)
    nv = pl.nees(_LOG, est, "v")
    return dict(foot_noise=foot, kin_noise=kin, liftoff_advance_ms=adv, settle_ms=settle,
                vel_rmse=s["vel_rmse_world_3d"], vel_rmse_z=s["vel_rmse_world_xyz"][2],
                pos_final_err_m=s["pos_final_err_m"], roll_rmse_deg=s["roll_rmse_deg"],
                pitch_rmse_deg=s["pitch_rmse_deg"], nees_v_median=float(np.median(nv)))


def main():
    combos = list(itertools.product(*GRID.values()))
    with ProcessPoolExecutor(max_workers=os.cpu_count(), initializer=_init) as ex:
        rows = list(ex.map(evaluate, combos))
    rows.sort(key=lambda r: r["vel_rmse"])
    best = rows[0]
    out = pl.ROOT / "results" / "tuning_grid.json"
    out.write_text(json.dumps(dict(grid=GRID, rule="min world 3D velocity RMSE on data/tuning.npz", rows=rows),
                              indent=1))
    save_tuned(dict(foot_noise=best["foot_noise"], kin_noise=best["kin_noise"], settle_steps=int(best["settle_ms"])),
               ContactPolicy(use_schedule=True, liftoff_advance_ms=best["liftoff_advance_ms"]),
               dict(dataset="data/tuning.npz (long_walk_profile, 120 s, sensor seed 100)",
                    rule="min world 3D velocity RMSE", n_candidates=len(rows), best=best,
                    runner_up=rows[1], provenance=pl.provenance()))
    print(f"{len(rows)} candidates. Top 5 by velocity RMSE on the tuning walk:")
    for r in rows[:5]:
        print("  " + "  ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))
    print("worst:", {k: round(v, 4) if isinstance(v, float) else v for k, v in rows[-1].items()})


if __name__ == "__main__":
    main()
