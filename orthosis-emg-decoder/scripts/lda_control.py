"""LDA trained on exactly the CNN's training windows (days 1-50, every 4th window).

Separates "CNN vs LDA" from "less training data". Writes results/lda_control.json.
"""
import json
from pathlib import Path

import numpy as np
from sklearn.metrics import balanced_accuracy_score

from emgdec.data import DAYS, load_all
from emgdec.models import lda
from emgdec.pipeline import build_table, run_chain


def main():
    recs = load_all()
    out = {}
    for name in ("reference", "v2", "v1_envelope", "v1_raw"):
        T = build_table(run_chain(recs, name)[0])
        tr = np.flatnonzero(T.day <= 50)[::4]
        model = lda().fit(T.X[tr], T.y[tr])
        per_day = [balanced_accuracy_score(T.y[T.day == d], model.predict(T.X[T.day == d]))
                   for d in DAYS if d > 60]
        va = (T.day > 50) & (T.day <= 60)
        out[name] = {"val_days_51_60": float(balanced_accuracy_score(T.y[va], model.predict(T.X[va]))),
                     "test_per_day": {"mean": float(np.mean(per_day)), "sd": float(np.std(per_day))}}
        print(name, out[name], flush=True)
    Path("results/lda_control.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
