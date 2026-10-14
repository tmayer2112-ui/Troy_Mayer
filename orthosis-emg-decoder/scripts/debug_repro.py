"""Bring back the failures in DEBUG_LOG.md.

  PYTHONPATH=. python3 scripts/debug_repro.py N

1  Hum level drawn per recording: v1 within-day accuracy jumps to ~92 %.  Needs `make data`, ~1 min.
3  The CNN's cliff at day 69, and what each augmentation does to it.       From results/*.json.
5  One-day polarity "changes" from the old mean-skewness test.            From results/data_audit.json.
6  RMS across each polarity change.                                       From results/data_audit.json.
"""
import json
import sys
from pathlib import Path

import numpy as np

R = Path("results")


def repro_1():
    from emgdec import pipeline as pl
    from emgdec import protocols as P
    from emgdec.data import load_all

    fixed, bug = '_rng(day, 0, "hum-level" + salt)', '_rng(day, cls, "hum-level" + salt)'
    src = Path(pl.__file__).read_text()
    if fixed not in src:
        sys.exit("pipeline.run_chain no longer draws the hum level the way this repro expects")
    ns = {}
    exec(compile(src.replace(fixed, bug), "pipeline (per-recording hum)", "exec"), {**vars(pl)}, ns)
    recs = load_all()
    for label, run in (("per day (as fixed)", pl.run_chain), ("per recording (the bug)", ns["run_chain"])):
        T = pl.build_table(run(recs, "v1_envelope", v_cm=0.01)[0])
        print(f"v1 envelope, 10 mV hum, level drawn {label:24s} within-day "
              f"{100 * P.within_day(T)['per_day']['mean']:.1f} %   cross-day {100 * P.cross_day(T)['per_day']['mean']:.1f} %")


def repro_3():
    from emgdec.data import DAYS
    days = np.array([d for d in DAYS if d > 60])
    cnn = json.loads((R / "cnn.json").read_text())
    lda = json.loads((R / "lda.json").read_text())["main"]
    rows = [("LDA, days 1-60", np.array(lda["v2@0.0"]["cross_day"]["series"]))]
    rows += [(f"CNN{lab}", np.mean([r["test_series"] for r in cnn[k]["runs"]], axis=0))
             for k, lab in (("v2", ""), ("v2+gain", " + gain aug"), ("v2+flip", " + polarity aug"))]
    print(f"{'v2 board':24s} days 61-67  days 69-121")
    for name, s in rows:
        print(f"{name:24s} {100 * s[days < 69].mean():9.1f}  {100 * s[days >= 69].mean():10.1f}")


def repro_5():
    a = json.loads((R / "data_audit.json").read_text())
    print("old test, sign of the class-averaged skewness:")
    for s in a["mean_skew_sign_segments_ch1234"]:
        print(f"  days {s['first_day']:3d}-{s['last_day']:3d}  {s['pattern']}")
    print("class-profile test (+ = same polarity as days 26-50):")
    for s in a["polarity"]["segments_ch1234"]:
        print(f"  days {s['first_day']:3d}-{s['last_day']:3d}  {s['pattern']}")
    low = a["polarity"]["low_confidence_calls"]
    print(f"{len(low)} low-confidence calls, {a['polarity']['low_confidence_calls_against_their_segment']} "
          f"against their segment: " + ", ".join(f"day {e['day']} ch{e['channel']} β={e['beta']}" for e in low))


def repro_6():
    p = json.loads((R / "data_audit.json").read_text())["polarity"]
    for c in p["rms_at_changes"]:
        print(f"day {c['first_day']:3d}: flipped ch{c['flipped_channels']} RMS x{c['rms_ratio_flipped']}, "
              f"all channels x{c['rms_ratio_all_channels']}")
    print(f"same 5-day ratio where nothing changes, 5th-95th percentile: {p['rms_ratio_elsewhere_p5_p95']}")


if __name__ == "__main__":
    n = sys.argv[1] if len(sys.argv) > 1 else ""
    fn = globals().get(f"repro_{n}")
    if fn is None:
        sys.exit(__doc__)
    fn()
