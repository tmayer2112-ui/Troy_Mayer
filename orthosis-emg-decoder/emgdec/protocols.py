"""Evaluation protocols, from most optimistic (leaky) to most honest.

Each recording is one sustained ~6 s contraction, so "within-day" means splitting a
recording in time: the first 60 % trains, the last 40 % tests, with a 0.25 s gap
so no window straddles the boundary.
"""
import numpy as np
from sklearn.metrics import balanced_accuracy_score
from sklearn.model_selection import train_test_split

from .data import DAYS
from .features import SCALE_FEATURES
from .models import lda, lda_score

TRAIN_DAYS = np.array([d for d in DAYS if d <= 60])
TEST_DAYS = np.array([d for d in DAYS if d > 60])
SPLIT = 0.6
GAP_S = 0.25


def _early(T):
    return T.t1 <= SPLIT * T.dur


def _late(T):
    return T.t0 >= SPLIT * T.dur + GAP_S


def _stats(v):
    v = np.asarray(v, float)
    return {"mean": float(v.mean()), "sd": float(v.std()), "n": int(len(v))}


def random_split(T, seed=0):
    """Pool days 1-60 and split windows at random. Overlapping neighbours of every
    test window sit in the training set, so this is the number not to report."""
    idx = np.flatnonzero(np.isin(T.day, TRAIN_DAYS))
    tr, te = train_test_split(idx, test_size=0.25, random_state=seed, stratify=T.y[idx])
    return {"bal_acc": lda_score(T.X[tr], T.y[tr], T.X[te], T.y[te])[0]}


def within_day(T):
    """Calibrate every day: train on the first 60 % of that day, test on the rest."""
    accs = []
    for d in np.unique(T.day):
        m = T.day == d
        tr, te = m & _early(T), m & _late(T)
        accs.append(lda_score(T.X[tr], T.y[tr], T.X[te], T.y[te])[0])
    return {"per_day": _stats(accs)}


def cross_day(T, classes=None, X=None):
    """Train once on days 1-60, test on each of days 61-121 without recalibrating."""
    X = T.X if X is None else X
    keep = np.ones(len(T.y), bool) if classes is None else np.isin(T.y, classes)
    tr = keep & np.isin(T.day, TRAIN_DAYS)
    model = lda().fit(X[tr], T.y[tr])
    per_day = []
    for d in TEST_DAYS:
        te = keep & (T.day == d)
        per_day.append(balanced_accuracy_score(T.y[te], model.predict(X[te])))
    return {"per_day": _stats(per_day), "series": [round(float(a), 4) for a in per_day]}


GAP_BINS = [(1, 1), (2, 5), (6, 10), (11, 20), (21, 40), (41, 80), (81, 120)]


def day_gap_curve(T, block=5):
    """Train on 5 consecutive recording days, test on every later day; accuracy vs
    calendar days elapsed since the last training day."""
    by_gap = {}
    present = np.array(DAYS)
    for i in range(0, len(present) - block, block):
        days = present[i:i + block]
        tr = np.isin(T.day, days)
        model = lda().fit(T.X[tr], T.y[tr])
        for d in present[i + block:]:
            te = T.day == d
            by_gap.setdefault(int(d - days[-1]), []).append(
                balanced_accuracy_score(T.y[te], model.predict(T.X[te])))
    return {f"{lo}-{hi}": _stats(sum((by_gap.get(g, []) for g in range(lo, hi + 1)), []))
            for lo, hi in GAP_BINS}


def recalibration(T, seconds=(0.25, 0.5, 1.0, 2.0)):
    """On each test day, record k seconds per class, then test on that day's last 40 %.

    `old+fresh` adds those windows to days 1-60; `fresh` trains on them alone.
    k = 0 for `old+fresh` is cross-day evaluated on the same late windows.
    """
    old = np.isin(T.day, TRAIN_DAYS)
    out = {"old+fresh": {}, "fresh": {}}
    for k in (0.0,) + tuple(seconds):
        acc = {"old+fresh": [], "fresh": []}
        for d in TEST_DAYS:
            day = T.day == d
            te = day & _late(T)
            fresh = day & _early(T) & (T.t1 <= k)
            tr = old | fresh
            acc["old+fresh"].append(lda_score(T.X[tr], T.y[tr], T.X[te], T.y[te])[0])
            if k > 0:
                acc["fresh"].append(lda_score(T.X[fresh], T.y[fresh], T.X[te], T.y[te])[0])
        for mode, v in acc.items():
            if v:
                out[mode][str(k)] = _stats(v)
    return out


def reference_normalised(T, ref_class=0):
    """Divide each day's amplitude features by that day's reference recording.

    In log space that is a subtraction. The reference class is then left out of
    scoring, so both arms are compared on the same 10 classes.
    """
    X = T.X.copy()
    for d in np.unique(T.day):
        m = T.day == d
        ref = X[m & (T.y == ref_class), SCALE_FEATURES].mean(axis=0)
        X[m, SCALE_FEATURES] -= ref
    classes = [c for c in np.unique(T.y) if c != ref_class]
    return {"raw": cross_day(T, classes)["per_day"],
            "normalised": cross_day(T, classes, X)["per_day"]}
