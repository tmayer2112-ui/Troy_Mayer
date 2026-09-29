"""Two things about the published MultiDay files, found while debugging the CNN.

1. Byte-identical recordings under different day numbers.
2. Per-channel polarity changes between sessions: the sign of each channel's
   skewness flips while its RMS doesn't, which is what a reversed electrode lead
   looks like. The pattern changes around days 26, 68 and 105.

Writes results/data_audit.json and results/figures/polarity.png.
"""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import skew

from emgdec.data import ALL_DAYS, CLASSES, DATA_DIR, load_all


def main():
    groups = {}
    for d in ALL_DAYS:
        h = hashlib.md5(b"".join((DATA_DIR / f"S0_D{d}_C{c}.csv").read_bytes()
                                 for c in range(len(CLASSES)))).hexdigest()
        groups.setdefault(h, []).append(d)
    dupes = [g for g in groups.values() if len(g) > 1]

    recs = load_all(days=ALL_DAYS)
    sk = np.array([np.mean([skew(recs[(d, c)], axis=0) for c in range(len(CLASSES))], axis=0)
                   for d in ALL_DAYS], dtype=float)
    days = np.array(ALL_DAYS)
    clean = ~np.isin(days, sum(dupes, []))
    pattern = {int(d): "".join("+" if v > 0 else "-" for v in s) for d, s, k in zip(days, sk, clean) if k}
    runs = []                                  # run-length segments of the sign pattern
    for d, p in pattern.items():
        if runs and runs[-1]["pattern"] == p:
            runs[-1]["last_day"] = d
        else:
            runs.append({"first_day": d, "last_day": d, "pattern": p})
    train = {p for d, p in pattern.items() if d <= 60}
    res = {"identical_day_groups": dupes,
           "skewness_sign_segments_ch1234": runs,
           "patterns_in_training_days_1_60": sorted(train),
           "test_days_with_unseen_pattern": [d for d, p in pattern.items() if d > 60 and p not in train]}
    Path("results/data_audit.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))

    fig, ax = plt.subplots(figsize=(8, 3.2))
    for ch in range(4):
        ax.plot(days[clean], sk[clean, ch], ".-", lw=0.8, label=f"channel {ch + 1}")
    for b in (25.5, 67.5, 105.5):
        ax.axvline(b, color="k", lw=0.8, ls="--")
    ax.axhline(0, color="0.5", lw=0.5)
    ax.set(xlabel="day", ylabel="skewness (mean over classes)",
           title="Electrode polarity changes between sessions (duplicate days removed)")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("results/figures/polarity.png", dpi=150)


if __name__ == "__main__":
    main()
