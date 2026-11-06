"""Module 1: EMG and its features. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.
"""
from types import SimpleNamespace

import numpy as np

WIN = 400       # samples: 195 ms at 2048 Hz
STEP = 100      # samples: 49 ms, so neighbouring windows overlap by 75 %


def windows(x, win=WIN, step=STEP):
    """Cut one recording x, shape (n_samples, n_channels), into overlapping windows.

    Returns (w, starts): w has shape (n_windows, win, n_channels) and starts[i] is the index of
    window i's first sample. A window must fit completely: the last one ends at or before len(x).
    """
    # TODO (2-3 lines): the starts are 0, step, 2*step, ... while start + win <= len(x)
    # --- solution ---
    starts = np.arange(0, len(x) - win + 1, step)
    w = np.stack([x[s:s + win] for s in starts])
    return w, starts
    # --- end ---


def mav(w):
    """Mean absolute value of each window and channel: shape (n_windows, n_channels)."""
    # TODO (1 line). w has shape (n_windows, win, n_channels): average over axis 1.
    # --- solution ---
    return np.abs(w).mean(axis=1)
    # --- end ---


def wl(w):
    """Waveform length: the sum of |x[i+1] - x[i]| over each window."""
    # TODO (1 line). np.diff(w, axis=1) gives the differences along time.
    # --- solution ---
    return np.abs(np.diff(w, axis=1)).sum(axis=1)
    # --- end ---


def zc(w, thr):
    """Zero crossings: neighbours with opposite signs AND a step of at least thr (the dead band,
    one value per channel). The dead band stops noise wiggling around zero from counting."""
    # TODO (2 lines): x[i] * x[i+1] < 0  and  |x[i+1] - x[i]| >= thr, counted over each window
    # --- solution ---
    d = np.diff(w, axis=1)
    return ((w[:, :-1] * w[:, 1:] < 0) & (np.abs(d) >= thr)).sum(axis=1)
    # --- end ---


def ssc(w, thr):
    """Slope sign changes: the slope flips sign (a local peak or trough), and at least one of the
    two slopes is at least thr."""
    # TODO (3 lines): with d = np.diff(w, axis=1), compare d[:, :-1] with d[:, 1:]
    # --- solution ---
    d = np.diff(w, axis=1)
    d1, d2 = d[:, :-1], d[:, 1:]
    return ((d1 * d2 < 0) & ((np.abs(d1) >= thr) | (np.abs(d2) >= thr))).sum(axis=1)
    # --- end ---


# ------------------------------------------------------------------ provided from here down
FEATURE_NAMES = [f"{f}_ch{c + 1}" for f in ("logMAV", "logWL", "ZC", "SSC") for c in range(4)]
AMPLITUDE_COLUMNS = slice(0, 8)     # logMAV and logWL move with electrode gain; ZC and SSC don't


def hudgins(w, thr):
    """The 16 Hudgins features of each window: logMAV, logWL, ZC, SSC for each of 4 channels.
    MAV and WL are logged so that a gain change becomes an offset (Worksheet 1.3)."""
    eps = 1e-12
    return np.concatenate([np.log(mav(w) + eps), np.log(wl(w) + eps), zc(w, thr), ssc(w, thr)], axis=1)


def dead_band(signals, frac=0.05):
    """Per-channel ZC/SSC threshold: 5 % of the channel's RMS on calibration data, then frozen.
    The real project uses days 1-5 (emgdec/features.py, dead_band)."""
    return frac * np.sqrt(np.mean(np.concatenate(signals) ** 2, axis=0))


def feature_table(recs, featurize, fs=2048):
    """Window every recording in recs {(day, class): (n, ch)} and stack the features.

    featurize(w) maps windows (n_windows, win, ch) to a feature matrix. Returns a table with
    X (features), y (class), day, rec (recording number), start (sample), t0/t1 (window start/end, s)
    and dur (recording length, s): what Module 4's protocols need.
    """
    parts = {k: [] for k in ("X", "y", "day", "rec", "start", "t0", "t1", "dur")}
    for i, ((day, cls), x) in enumerate(sorted(recs.items())):
        w, starts = windows(x)
        n = len(starts)
        parts["X"].append(featurize(w))
        parts["y"].append(np.full(n, cls))
        parts["day"].append(np.full(n, day))
        parts["rec"].append(np.full(n, i))
        parts["start"].append(starts)
        parts["t0"].append(starts / fs)
        parts["t1"].append((starts + w.shape[1]) / fs)
        parts["dur"].append(np.full(n, len(x) / fs))
    return SimpleNamespace(**{k: np.concatenate(v) for k, v in parts.items()})


def hudgins_table(recs, cal_days=(1, 2, 3, 4, 5)):
    """The decoder's feature table: dead band from the calibration days, then Hudgins features."""
    thr = dead_band([x for (d, _), x in recs.items() if d in cal_days])
    return feature_table(recs, lambda w: hudgins(w, thr))
