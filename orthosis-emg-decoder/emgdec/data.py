"""Load the Kaufmann/Englehart/Platzner 120-day dataset (LibEMG `MultiDay`).

One subject, 4 surface-EMG channels at 2048 Hz, 11 wrist and hand classes, one
~6 s sustained contraction per class per day, 121 recording days. It is the only
4-channel, multi-day raw-EMG set reachable without a registration wall, and the
channel count matches the orthosis board.

Files are `S0_D{day}_C{class}.csv`, space-separated, one row per sample.
Values are taken to be microvolts (the NeXus amplifier's native unit).

Two properties of the published files matter (see scripts/audit_data.py):
- Days 5/7, 63/68 and 110/112 are byte-identical copies of each other. All six
  days are dropped rather than guessing which copy is real.
- Channel polarity changes between sessions, around days 26, 68 and 105: a
  channel's skewness flips sign while its RMS doesn't, as a reversed lead would.
  Nothing corrects this. A decoder on real electrodes has to survive a swapped
  lead too.
"""
import os
from pathlib import Path

import numpy as np

FS = 2048
ALL_DAYS = list(range(1, 122))
DUPLICATE_DAYS = {5, 7, 63, 68, 110, 112}
DAYS = [d for d in ALL_DAYS if d not in DUPLICATE_DAYS]
CLASSES = ["no motion", "wrist extension", "wrist flexion", "wrist adduction",
           "wrist abduction", "supination", "pronation", "hand open",
           "hand closed", "key grip", "index point"]
UV = 1e-6

DATA_DIR = Path(os.environ.get("MULTIDAY_DIR", Path(__file__).resolve().parents[1] / "data" / "multiday"))
CACHE = DATA_DIR.parent / "multiday_cache.npz"


def load_all(days=DAYS):
    """{(day, cls): (n, 4) float32 microvolts}, cached to one .npz after the first read."""
    if CACHE.exists():
        z = np.load(CACHE)
        recs = {tuple(int(v) for v in k.split("_")): z[k] for k in z.files}
    else:
        if not DATA_DIR.exists():
            raise FileNotFoundError(f"{DATA_DIR} missing: run `make data` first")
        recs = {(d, c): np.loadtxt(DATA_DIR / f"S0_D{d}_C{c}.csv", dtype=np.float32)
                for d in ALL_DAYS for c in range(len(CLASSES))}
        np.savez(CACHE, **{f"{d}_{c}": x for (d, c), x in recs.items()})
    return {k: v for k, v in recs.items() if k[0] in days}
