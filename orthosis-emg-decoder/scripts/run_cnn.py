"""1D CNN on raw board output, cross-day. Writes results/cnn.json.

Train on days 1-50, pick the epoch on days 51-60, test on each of days 61-121.
Validating on held-out *days* (not held-out windows) makes model selection
optimise for the shift we actually care about.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from sklearn.metrics import balanced_accuracy_score

from emgdec.data import CLASSES, DAYS, load_all
from emgdec.models import cnn_predict, train_cnn
from emgdec.pipeline import build_table, run_chain

# "+gain" / "+flip" train with gain or polarity augmentation (see models.train_cnn)
CHAINS = ["reference", "v2", "v1_envelope", "v1_raw",
          "reference+gain", "v2+gain", "reference+flip", "v2+flip"]
TEST_DAYS = [d for d in DAYS if d > 60]
SEEDS = [0, 1, 2]
OUT = Path("results/cnn.json")


def main(chains=CHAINS, seeds=SEEDS, epochs=8):
    torch.set_num_threads(int(sys.argv[1]) if len(sys.argv) > 1 else 2)
    recs = load_all()
    res = json.loads(OUT.read_text()) if OUT.exists() else {}
    for key in chains:
        name, _, aug = key.partition("+")
        gain_aug = 1.5 if aug == "gain" else None
        sig, info = run_chain(recs, name)
        T = build_table(sig, keep_windows=True)
        del sig
        tr = np.flatnonzero(T.day <= 50)[::4]          # non-overlapping training windows
        va = np.flatnonzero((T.day > 50) & (T.day <= 60))
        scale = T.W[tr].std(axis=(0, 1)) + 1e-12
        X = lambda idx: (T.W[idx] / scale).astype(np.float32)
        runs = []
        for seed in seeds:
            t = time.time()
            print(f"{key} seed {seed}", flush=True)
            model, hist = train_cnn(X(tr), T.y[tr], X(va), T.y[va], len(CLASSES), seed,
                                    epochs=epochs, gain_aug=gain_aug, flip_aug=aug == "flip",
                                    log=lambda s: print(s, flush=True))
            per_day = []
            for d in TEST_DAYS:
                te = np.flatnonzero(T.day == d)
                per_day.append(balanced_accuracy_score(T.y[te], cnn_predict(model, X(te))))
            runs.append({"seed": seed, "val_history": [round(v, 4) for v in hist],
                         "test_per_day_mean": float(np.mean(per_day)),
                         "test_series": [round(float(a), 4) for a in per_day],
                         "train_seconds": round(time.time() - t, 1)})
            print(f"  -> test {np.mean(per_day):.3f} ({time.time() - t:.0f} s)", flush=True)
        m = [r["test_per_day_mean"] for r in runs]
        res[key] = {"info": info, "augmentation": aug or None, "runs": runs,
                     "test_mean_over_seeds": float(np.mean(m)), "test_sd_over_seeds": float(np.std(m))}
        OUT.write_text(json.dumps(res, indent=1))
        del T


if __name__ == "__main__":
    main(sys.argv[2:] or CHAINS)
