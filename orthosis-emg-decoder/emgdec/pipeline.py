"""Run recordings through a signal chain and turn them into window tables."""
import zlib
from dataclasses import dataclass

import numpy as np

from . import board as B
from .data import FS, UV
from .features import STEP, TRIM_S, WIN, dead_band, hudgins, windows

CAL_DAYS = range(1, 6)      # pots and dead bands are set once, on the first five days


def make_chain(name, z_electrode=20e3, mismatch=0.25):
    e = B.Electrodes(z_electrode, mismatch)
    return {
        "reference": None,
        "v2": B.BoardV2(electrodes=e),
        "v1_envelope": B.BoardV1(electrodes=e, envelope=True),
        "v1_raw": B.BoardV1(electrodes=e, envelope=False),
    }[name]


def _rng(day, cls, salt):
    return np.random.default_rng(zlib.crc32(f"{day}-{cls}-{salt}".encode()))


def run_chain(recs, name, v_cm=0.0, salt="", adc_fs=None, anti_alias=False, **chain_kw):
    """Returns ({(day, cls): processed signal}, info). Signals are trimmed of TRIM_S.

    `adc_fs` samples the v2 board's output at another rate (the firmware uses 1 kHz);
    `anti_alias` puts a sharp FIR low-pass at adc_fs/2 in front of it. See board.sample_at.
    """
    chain = make_chain(name, **chain_kw)
    if adc_fs and not isinstance(chain, B.BoardV2):
        raise ValueError("adc_fs is only modelled for the v2 board")
    fs_out = adc_fs or FS
    trim = int(TRIM_S * fs_out)
    adc_kw = dict(adc_fs=adc_fs, anti_alias=anti_alias) if adc_fs else {}

    def inputs(day, cls):
        x = recs[(day, cls)].astype(np.float64) * UV
        if chain is not None and v_cm > 0:
            # Hum level is a property of the day (room, cables, skin), not of the
            # gesture: draw it per day, or it becomes a per-class fingerprint.
            level = v_cm * _rng(day, 0, "hum-level" + salt).uniform(0.5, 1.5)
            x = x + B.hum(len(x), FS, level, chain.cm_to_diff, _rng(day, cls, "hum-phase" + salt))
        return x

    info = {"chain": name, "v_cm": v_cm}
    if chain is None:
        out = {k: B.reference_chain(x.astype(np.float64), FS)[trim:].astype(np.float32)
               for k, x in recs.items()}
        return out, info

    target = 1.6 if isinstance(chain, B.BoardV1) else 1.2
    cal = [inputs(d, c) for (d, c) in recs if d in CAL_DAYS]
    B.calibrate_pots(chain, cal, FS, target)
    gain = chain.front_gain * B.K_SK ** 2 * chain.out_gain(4)
    info.update(pot_ohms=chain.pot_ohms.round().tolist(), total_gain=gain.round(1).tolist(),
                front_gain=round(float(chain.front_gain), 3),
                cmrr_db=round(float(-20 * np.log10(chain.cm_to_diff)), 1))

    out, clipped = {}, []
    for (d, c) in recs:
        x = inputs(d, c)
        y = chain(x, FS, _rng(d, c, "adc" + salt), **adc_kw)
        if isinstance(chain, B.BoardV1):
            pre = chain.pre_rectifier(x, FS)
            clipped.append(np.mean((pre >= chain.v_high - 1e-9) | (pre <= chain.v_low + 1e-9)))
            info.setdefault("_active", []).append(np.mean(pre > chain.diode_vf))
        else:
            pre = chain.pre_adc(x, FS)
            clipped.append(np.mean(np.abs(pre) >= chain.swing - 1e-9))
        out[(d, c)] = y[trim:].astype(np.float32)
    info["clipped_fraction"] = float(np.mean(clipped))
    if "_active" in info:
        info["above_diode_fraction"] = float(np.mean(info.pop("_active")))
    return out, info


@dataclass
class Table:
    X: np.ndarray        # (n, 16) Hudgins features
    y: np.ndarray        # class
    day: np.ndarray
    t0: np.ndarray       # window start, s from trimmed recording start
    t1: np.ndarray       # window end
    dur: np.ndarray      # trimmed recording length, s
    W: np.ndarray = None  # raw windows (n, WIN, 4), only when asked for


def build_table(sig, keep_windows=False, fs=FS):
    """Window length and step are fixed in seconds, so at fs = 2048 they are WIN and STEP."""
    win, step = round(WIN * fs / FS), round(STEP * fs / FS)
    thr = dead_band([x for (d, _), x in sig.items() if d in CAL_DAYS])
    parts = {k: [] for k in ("X", "y", "day", "t0", "t1", "dur", "W")}
    for (d, c), x in sorted(sig.items()):
        w, starts = windows(x, win, step)
        parts["X"].append(hudgins(w, thr).astype(np.float32))
        n = len(starts)
        parts["y"].append(np.full(n, c))
        parts["day"].append(np.full(n, d))
        parts["t0"].append(starts / fs)
        parts["t1"].append((starts + win) / fs)
        parts["dur"].append(np.full(n, len(x) / fs))
        if keep_windows:
            parts["W"].append(np.array(w, dtype=np.float32))
    cat = {k: np.concatenate(v) for k, v in parts.items() if v}
    return Table(**cat)
