"""Two things about the published MultiDay files, found while debugging the CNN.

1. Byte-identical recordings under different day numbers.
2. Electrode-lead reversals between sessions.

A reversed lead negates the signal, x -> -x. That flips the sign of every recording's
skewness and leaves its RMS unchanged. Both halves of that are tested:

- Class-profile regression. For each day and channel, the 11 per-class skewness values
  are regressed (no intercept) on the same channel's mean profile over a stable
  reference stretch (REF_DAYS). beta near +1 means the same polarity as the reference,
  near -1 reversed. A reversal predicts exactly this: every class flips together and
  keeps its size. When |beta| < MIN_BETA the day looks like neither, so no sign is
  called for that channel and it doesn't start a new segment.
  The earlier test (the sign of the class-averaged skewness) is kept for comparison.
  It produced one-day "changes" on days 1 and 51 that were channels near zero, not
  reversals.
- RMS across every change. For each channel that flips, the mean RMS over the SPAN
  kept days after the change divided by the SPAN days before, compared with the same
  ratio at every point where nothing changes.

Writes results/data_audit.json, results/figures/polarity.png and polarity.csv.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import skew

from emgdec.data import ALL_DAYS, CLASSES, DATA_DIR, DUPLICATE_DAYS, load_all

REF_DAYS = (26, 50)     # longest stable stretch inside the training days
MIN_BETA = 0.4          # |beta| below this: too weak to call a sign
SPAN = 5                # kept days either side of a change for the RMS ratio
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]   # colour-blind checked; markers repeat it
MARKERS = ["o", "s", "^", "D"]


def sign(v):
    return "+" if v > 0 else "-"


def runs(pattern):
    """Run-length segments of {day: pattern}. '?' matches anything and is filled in by
    the first confident call in the run, so a weak channel never starts a segment."""
    out = []
    for d, p in pattern.items():
        last = out[-1]["pattern"] if out else None
        if last is not None and all(a == b or "?" in (a, b) for a, b in zip(last, p)):
            out[-1]["pattern"] = "".join(b if a == "?" else a for a, b in zip(last, p))
            out[-1]["last_day"] = d
        else:
            out.append({"first_day": d, "last_day": d, "pattern": p})
    return out


def day_patterns(segments):
    return {d: s["pattern"] for s in segments for d in range(s["first_day"], s["last_day"] + 1)}


def main():
    groups = {}
    for d in ALL_DAYS:
        h = hashlib.md5(b"".join((DATA_DIR / f"S0_D{d}_C{c}.csv").read_bytes()
                                 for c in range(len(CLASSES)))).hexdigest()
        groups.setdefault(h, []).append(d)
    dupes = [g for g in groups.values() if len(g) > 1]
    dropped = set(sum(dupes, []))

    recs = load_all(days=ALL_DAYS)
    days = np.array([d for d in ALL_DAYS if d not in dropped])
    n_cls = len(CLASSES)
    sk = np.array([[skew(recs[(d, c)], axis=0) for c in range(n_cls)] for d in days])   # (day, class, ch)
    rms = np.array([[np.sqrt(np.mean(np.square(recs[(d, c)], dtype=np.float64), axis=0))
                     for c in range(n_cls)] for d in days]).mean(axis=1)               # (day, ch)
    mean_sk = sk.mean(axis=1)

    ref = sk[(days >= REF_DAYS[0]) & (days <= REF_DAYS[1])].mean(axis=0)               # (class, ch)
    beta = np.einsum("dck,ck->dk", sk, ref) / np.einsum("ck,ck->k", ref, ref)
    called = {int(d): "".join("?" if abs(b) < MIN_BETA else sign(b) for b in row) for d, row in zip(days, beta)}
    segments = runs(called)
    per_day = day_patterns(segments)
    train = sorted({per_day[int(d)] for d in days if d <= 60})
    low = [{"day": int(days[i]), "channel": int(c) + 1, "beta": round(float(beta[i, c]), 3),
            "segment_sign": per_day[int(days[i])][c]}
           for i, c in np.argwhere(np.abs(beta) < MIN_BETA)]

    # RMS across each change, against the same ratio wherever nothing changes
    first = {s["first_day"] for s in segments[1:]}
    idx = {int(d): i for i, d in enumerate(days)}
    ratio = lambda i: rms[i:i + SPAN].mean(axis=0) / rms[max(0, i - SPAN):i].mean(axis=0)
    at_changes = []
    for prev, seg in zip(segments, segments[1:]):
        i = idx[seg["first_day"]]
        flipped = [c for c in range(4) if prev["pattern"][c] != seg["pattern"][c]]
        r = ratio(i)
        at_changes.append({"after_day": prev["last_day"], "first_day": seg["first_day"],
                           "flipped_channels": [c + 1 for c in flipped],
                           "rms_ratio_flipped": [round(float(r[c]), 3) for c in flipped],
                           "rms_ratio_all_channels": [round(float(v), 3) for v in r]})
    elsewhere = np.concatenate([ratio(i) for i in range(SPAN, len(days) - SPAN + 1)
                                if not any(abs(int(days[i]) - f) < SPAN for f in first)])

    res = {
        "identical_day_groups": dupes,
        "duplicate_days_match_data_py": dropped == DUPLICATE_DAYS,
        "days_kept": int(len(days)),
        "train_days": int(np.sum(days <= 60)),
        "test_days": int(np.sum(days > 60)),
        "polarity": {
            "method": f"per-class skewness profile regressed on the mean profile of days "
                      f"{REF_DAYS[0]}-{REF_DAYS[1]}; '+' same polarity, '-' reversed, "
                      f"'?' |beta| < {MIN_BETA}",
            "segments_ch1234": segments,
            "patterns_in_training_days_1_60": train,
            "test_days_with_unseen_pattern": [int(d) for d in days if d > 60 and per_day[int(d)] not in train],
            "abs_beta_quantiles_5_50_95": np.quantile(np.abs(beta), [0.05, 0.5, 0.95]).round(3).tolist(),
            "low_confidence_calls": low,
            "low_confidence_calls_against_their_segment": sum(
                1 for e in low if sign(beta[idx[e["day"]], e["channel"] - 1]) != e["segment_sign"]),
            "rms_at_changes": at_changes,
            "rms_ratio_elsewhere_p5_p95": np.quantile(elsewhere, [0.05, 0.95]).round(3).tolist(),
        },
        "mean_skew_sign_segments_ch1234": runs({int(d): "".join(map(sign, s)) for d, s in zip(days, mean_sk)}),
    }
    Path("results/data_audit.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))

    figs = Path("results/figures")
    with open(figs / "polarity.csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["day"] + [f"{q}_ch{c + 1}" for q in ("mean_skew", "beta", "rms") for c in range(4)])
        for i, d in enumerate(days):
            w.writerow([int(d)] + [f"{v:.4f}" for v in (*mean_sk[i], *beta[i], *rms[i])])

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    for ch in range(4):
        kw = dict(color=COLORS[ch], marker=MARKERS[ch], ms=3.5, lw=0.8, label=f"channel {ch + 1}")
        a1.plot(days, mean_sk[:, ch], **kw)
        a2.plot(days, beta[:, ch], **kw)
    for prev, seg in zip(segments, segments[1:]):
        for ax in (a1, a2):
            ax.axvline((prev["last_day"] + seg["first_day"]) / 2, color="k", lw=0.8, ls="--")
    a1.axhline(0, color="0.5", lw=0.5)
    a2.axhspan(-MIN_BETA, MIN_BETA, color="0.9", zorder=0)
    for y in (-1, 1):
        a2.axhline(y, color="0.5", lw=0.5, ls=":")
    a1.set(ylabel="skewness\n(mean over classes)",
           title="Electrode polarity between sessions (duplicate days removed)")
    a2.set(xlabel="day", ylabel=f"class profile vs\ndays {REF_DAYS[0]}–{REF_DAYS[1]} (β)",
           title=f"+1 = same polarity as days {REF_DAYS[0]}–{REF_DAYS[1]}, −1 = reversed, "
                 f"grey = |β| < {MIN_BETA}, no call")
    a2.title.set_fontsize(9)
    a1.legend(fontsize=8, ncol=4)
    for ax in (a1, a2):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "polarity.png", dpi=150)


if __name__ == "__main__":
    main()
