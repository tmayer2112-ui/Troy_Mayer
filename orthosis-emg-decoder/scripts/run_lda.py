"""Every LDA experiment. Writes results/lda.json.

1. Main table: reference / v2 / v1 (envelope) / v1 (raw), with and without 100 mV
   of 60 Hz common-mode, through every protocol.
2. Hum sweep: common-mode amplitude 0-1 V.
3. Electrode-impedance sweep: 0 (buffered) to 50 kOhm.
"""
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from sklearn.metrics import confusion_matrix

from emgdec import protocols as P
from emgdec.data import load_all
from emgdec.models import lda
from emgdec.pipeline import build_table, run_chain

OUT = Path("results/lda.json")
REALISTIC_VCM = 0.1
BOARDS = ["v2", "v1_envelope", "v1_raw"]

RECS = None


def _init():
    global RECS
    RECS = load_all()


def full(job):
    name, v_cm = job
    sig, info = run_chain(RECS, name, v_cm=v_cm)
    T = build_table(sig)
    del sig
    tr, te = np.isin(T.day, P.TRAIN_DAYS), np.isin(T.day, P.TEST_DAYS)
    cm = confusion_matrix(T.y[te], lda().fit(T.X[tr], T.y[tr]).predict(T.X[te]), normalize="true")
    out = {"info": info,
           "random_split": P.random_split(T),
           "within_day": P.within_day(T),
           "cross_day": P.cross_day(T),
           "day_gap": P.day_gap_curve(T),
           "recalibration": P.recalibration(T),
           "reference_normalised": P.reference_normalised(T),
           "cross_day_confusion": np.round(cm, 3).tolist()}
    print(f"full  {name:12s} v_cm={v_cm:<5} cross-day {out['cross_day']['per_day']['mean']:.3f}", flush=True)
    return f"{name}@{v_cm}", out


def quick(job):
    name, v_cm, z = job
    sig, info = run_chain(RECS, name, v_cm=v_cm, z_electrode=z)
    T = build_table(sig)
    del sig
    out = {"info": info, "cross_day": P.cross_day(T)["per_day"], "within_day": P.within_day(T)["per_day"]}
    print(f"sweep {name:12s} v_cm={v_cm:<5} z={z:<7} cross-day {out['cross_day']['mean']:.3f}", flush=True)
    return f"{name}@{v_cm}@{z}", out


def main(workers):
    main_jobs = [("reference", 0.0)] + [(b, v) for b in BOARDS for v in (0.0, REALISTIC_VCM)]
    hum_jobs = [(b, v, 20e3) for b in BOARDS for v in (0.003, 0.01, 0.03, 0.3, 1.0)]
    z_jobs = [(b, v, z) for b in BOARDS for v in (0.0, REALISTIC_VCM) for z in (0.0, 5e3, 10e3, 50e3)]
    with Pool(workers, initializer=_init) as pool:
        res = {"main": dict(pool.map(full, main_jobs, chunksize=1))}
        res["sweeps"] = dict(pool.map(quick, hum_jobs + z_jobs, chunksize=1))
    OUT.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 2)
