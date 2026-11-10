"""Every number in DECODER_ANSWERS.md, recomputed: the arithmetic, the synthetic examples, and the real
values read from ../results/*.json (maintainers: run this after any change).

    python solutions/decoder_numbers.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (str(HERE), str(HERE.parent)):
    if p not in sys.path:
        sys.path.insert(0 if p == str(HERE) else len(sys.path), p)

import m1_features as F  # noqa: E402
import simdata  # noqa: E402

RESULTS = HERE.parent.parent / "results"


def real(name, path):
    v = json.loads((RESULTS / name).read_text())
    for k in path.split("/"):
        v = v[int(k)] if isinstance(v, list) else v[k]
    return v


def show(label, v):
    print(f"  {label:62s} {v}")


def windows_in(seconds, fs=2048, win=400, step=100):
    return (int(round(seconds * fs)) - win) // step + 1


print("D1 the pipeline")
n6 = windows_in(6.0 - 0.5)
show("windows in a 6.0 s hold after the 0.5 s trim", n6)
show("per training day (11 classes); 58 days", (11 * n6, 58 * 11 * n6))
per_class = 74000 / (58 * 11)
show("README's ~74,000 training windows -> windows per hold, hold length (s)",
     (round(per_class), round(((round(per_class) - 1) * 100 + 400) / 2048 + 0.5, 1)))

print("D2 the trim")
Q, f0 = 1 / (3 - (1 + 10 / 18)), 1 / (2 * math.pi * 15e3 * 470e-9)
show("HPF pole time constant 2Q/w0 (ms)", round(1e3 * 2 * Q / (2 * math.pi * f0), 1))
show("peak detector left after 0.5 s", round(math.exp(-0.5 / 0.18), 3))

print("D3 the pots")
p = 1.2 / 58
show("v2: 99.9th pct 20.69 mV -> gain, pot (ohm)", (round(1.2 / 0.02069, 1), round((1.2 / 0.02069 - 1) * 1e3 - 10e3)))
show("real channel 1: pot, total gain, clipped fraction",
     (real("lda.json", "main/v2@0.0/info/pot_ohms/0"), real("lda.json", "main/v2@0.0/info/total_gain/0"),
      round(real("lda.json", "main/v2@0.0/info/clipped_fraction"), 6)))
show("real total gain check: 10.107 x 1.5556^2 x (1 + (10k + 46962)/1k)",
     round(10.107 * (1 + 10 / 18) ** 2 * (1 + (10e3 + 46962) / 1e3), 1))
show("v1: max gain at 100k; wanted gain for 0.5 mV; pot it asks for (ohm)",
     (1 + (47e3 + 100e3) / 1e3, round(1.6 / 0.5e-3), round((1.6 / 0.5e-3 - 1) * 1e3 - 47e3)))
show("real v1 pots", real("lda.json", "main/v1_envelope@0.0/info/pot_ohms"))

print("D4 what the features measure (synthetic: day 6, class 2, first window, channel 1)")
recs = simdata.make_dataset(days=range(1, 7))
thr = F.dead_band([x for (d, _), x in recs.items() if d <= 5])
w, _ = F.windows(recs[(6, 2)])
X = F.hudgins(w[:1], thr)[0]
lmav, lwl, zc, ssc = X[0], X[4], X[8], X[12]
ratio = math.exp(lwl - lmav)
f_rms = ratio * 2048 / (399 * 2 * math.pi)
show("logMAV, logWL, WL/MAV", (round(lmav, 2), round(lwl, 2), round(ratio, 1)))
show("f_rms from WL/MAV = ratio fs / (399 2 pi) (Hz)", round(f_rms, 1))
show("Rice: predicted ZC = 2 f_rms T, T = 0.1953 s; measured ZC, SSC", (round(2 * f_rms * 400 / 2048, 1), int(zc), int(ssc)))
show("Rice at f_rms = 120 Hz", round(2 * 120 * 400 / 2048, 1))

print("D5 LDA's size")
show("multiply-adds per decision; parameters (W, b, scaler)", (11 * 16, 11 * 16 + 11 + 2 * 16))
show("0.25 s per class: windows per class, total; covariance numbers", (windows_in(0.25), 11 * windows_in(0.25), 16 * 17 // 2))

print("D6 the CNN")
layers = [("conv 4->32 k9", 4 * 32 * 9 + 32), ("bn 32", 64), ("conv 32->64 k7", 32 * 64 * 7 + 64), ("bn 64", 128),
          ("conv 64->64 k5", 64 * 64 * 5 + 64), ("bn 64", 128), ("linear 64->11", 64 * 11 + 11)]
show("parameters per layer", layers)
show("total", sum(n for _, n in layers))
rf, jump = 1, 1
for k, s in ((9, 1), (2, 2), (7, 1), (2, 2), (5, 1)):
    rf, jump = rf + (k - 1) * jump, jump * s
show("receptive field (samples, ms)", (rf, round(rf / 2048 * 1e3, 1)))
macs = 4 * 32 * 9 * 400 + 32 * 64 * 7 * 200 + 64 * 64 * 5 * 100 + 64 * 11
show("multiply-adds per window (M); ratio to LDA's 176", (round(macs / 1e6, 2), round(macs / 176)))
cnn = json.loads((RESULTS / "cnn.json").read_text())
best = np.mean([max(r["val_history"]) for r in cnn["v2"]["runs"]])
last = np.mean([r["val_history"][-1] for r in cnn["v2"]["runs"]])
show("v2 CNN val: best epoch, last epoch (mean of 3 seeds)", (round(100 * best, 1), round(100 * last, 1)))
show("gain augmentation range", (round(math.exp(-math.log(1.5)), 3), 1.5))
show("training windows: days 1-50, every 4th (approx, from ~74,000 over 58 days)", round(74000 * 50 / 58 / 4))

print("D7 time and recalibration (real)")
show("day gap curve", {k: round(100 * v["mean"], 1) for k, v in real("lda.json", "main/v2@0.0/day_gap").items()})
show("fresh only", {k: round(100 * v["mean"], 1) for k, v in real("lda.json", "main/v2@0.0/recalibration/fresh").items()})
show("old + fresh", {k: round(100 * v["mean"], 1) for k, v in real("lda.json", "main/v2@0.0/recalibration/old+fresh").items()})
n2 = windows_in(2.0)
show("2 s per class: fresh windows; old per class; fresh share %", (n2, round(74000 / 11), round(100 * n2 / (74000 / 11 + n2), 2)))
show("reference normalisation raw -> normalised",
     tuple(round(100 * real("lda.json", f"main/v2@0.0/reference_normalised/{k}/mean"), 1) for k in ("raw", "normalised")))

print("D9 claims")
claims = json.loads((HERE.parent.parent / "claims.json").read_text())
c = {x["id"]: x for x in claims["claims"]}
show("resume_v2 value, tolerance; v2_cross_day value, tolerance",
     ((c["resume_v2"]["value"], c["resume_v2"]["abs_tol"]), (c["v2_cross_day"]["value"], c["v2_cross_day"]["abs_tol"])))
bullet = "raising day-to-day gesture decoding from 31% to 87% in emulated tests."
show("check_claims.py regex 'gesture decoding.*?emulation' on the résumé bullet",
     re.search(r"gesture decoding.*?emulation", bullet))
show("a regex that would match: 'gesture decoding.*?emulat\\w*'",
     re.search(r"gesture decoding.*?emulat\w*", bullet).group(0))
