"""Windowing and the Hudgins time-domain feature set."""
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

WIN = 400       # 195 ms at 2048 Hz
STEP = 100      # 49 ms
TRIM_S = 0.5    # drop the first 0.5 s of every recording: filter and envelope settling


def windows(x, win=WIN, step=STEP):
    """(n, ch) -> (n_windows, win, ch) view, plus window start indices."""
    w = sliding_window_view(x, win, axis=0)[::step]          # (n_win, ch, win)
    starts = np.arange(w.shape[0]) * step
    return np.swapaxes(w, 1, 2), starts


def hudgins(w, thr):
    """MAV, WL, ZC, SSC per channel. `thr` is a per-channel dead band (same units as w).

    MAV and WL are returned as logs so they combine linearly with LDA.
    """
    d = np.diff(w, axis=1)
    mav = np.abs(w).mean(axis=1)
    wl = np.abs(d).sum(axis=1)
    zc = ((w[:, :-1] * w[:, 1:] < 0) & (np.abs(d) >= thr)).sum(axis=1)
    d1, d2 = d[:, :-1], d[:, 1:]
    ssc = ((d1 * d2 < 0) & ((np.abs(d1) >= thr) | (np.abs(d2) >= thr))).sum(axis=1)
    eps = 1e-12
    return np.concatenate([np.log(mav + eps), np.log(wl + eps), zc, ssc], axis=1)


FEATURE_NAMES = [f"{f}_ch{c + 1}" for f in ("logMAV", "logWL", "ZC", "SSC") for c in range(4)]
SCALE_FEATURES = slice(0, 8)   # log MAV and log WL move with electrode gain; ZC/SSC do not


def dead_band(signals, frac=0.05):
    """Per-channel ZC/SSC threshold: a fraction of the channel's RMS on calibration data."""
    return frac * np.sqrt(np.mean(np.concatenate(signals) ** 2, axis=0))
