"""Generate every dataset the other scripts use. Deterministic: same code + seeds -> same files.

  data/headline_seed{S}.npz   60 s evaluation walk (command_profile), sensor seeds 0..9
                              (the sim is deterministic, so only IMU/encoder noise changes between seeds)
  data/tuning.npz             120 s different walk (long_walk_profile), sensor seed 100 -- tuning only
  data/longwalk.npz           300 s walk for the observability / covariance plot, sensor seed 1
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf.gait import command_profile, long_walk_profile  # noqa: E402

DATA = pl.ROOT / "data"
N_SEEDS = 10


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=["headline", "tuning", "longwalk"], default=None)
    a = ap.parse_args()
    t0 = time.time()
    if a.only in (None, "headline"):
        base = pl.make_dataset(60.0, seed=0, profile=command_profile)
        pl.save_dataset(base, DATA / "headline_seed0.npz")
        for s in range(1, N_SEEDS):
            pl.save_dataset(pl.make_dataset(60.0, seed=s, profile=command_profile, sim_log=base),
                            DATA / f"headline_seed{s}.npz")
        print(f"headline: {N_SEEDS} seeds")
    if a.only in (None, "tuning"):
        pl.save_dataset(pl.make_dataset(120.0, seed=100, profile=long_walk_profile), DATA / "tuning.npz")
        print("tuning: 120 s")
    if a.only in (None, "longwalk"):
        pl.save_dataset(pl.make_dataset(300.0, seed=1, profile=long_walk_profile), DATA / "longwalk.npz")
        print("longwalk: 300 s")
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
