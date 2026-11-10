# Decoder worksheet: answers

Every number here is recomputed by `python solutions/decoder_numbers.py`. Real values come from
`orthosis-emg-decoder/results/`. The one synthetic example (D4) is labelled.

---

## D1

**D1.1** **115** days; **58** train, **57** test (`data_audit.json` → `days_kept`, `train_days`, `test_days`).

**D1.2**
- A 6.0 s hold trimmed by 0.5 s is 11,264 samples: (11,264 − 400)/100 + 1 = **109** windows.
- One day is 11 × 109 = **1,199** windows; 58 days give **69,542**.
- 74,000 over 58 × 11 holds is **116** windows per hold, i.e. a hold of about **6.3 s** (the README says
  "about 6 s").

**D1.3** board emulation → trim → windows → dead band → 16 Hudgins features → z-score → LDA fit → per-day
balanced accuracy → mean ± SD over test days.

**D1.4**
- Pot trim and dead band: **days 1-5**.
- Z-score and LDA fit: **training days** (z-scoring is inside the LDA pipeline, so it's fitted on training
  rows only).
- The board emulation: **every day**: it has no learned parameters, only the frozen pots.
- If a test day reached any of the first four, the model would have quietly calibrated on the day it's
  scored on, and "no recalibration" would be false.

---

## D2

- 2 × 0.692/(2π × 22.58) = **9.8 ms**, so the HPF settles in about 50 ms.
- e^(−0.5/0.18) = **0.062**, about 6 % left.
- v1's 180 ms peak detector sets the trim: the HPF alone would need a tenth of it.

---

## D3

- v2: G3 = 1.2/0.02069 = **58.0**, pot = (58 − 1) × 1k − 10k = **47 kΩ** (46,999 Ω).
- Real channel 1: 10.107 × 1.5556² × 57.96 = **1417.5**, against `total_gain` 1417.6 (rounding in K).
- v1: the maximum gain is 1 + (47k + 100k)/1k = **148**. 1.6 V/0.5 mV asks for a gain of **3,200**, i.e. a
  3.15 MΩ pot, so it clamps at 100k. The real v1 pots are [100k, 100k, 100k, 100k]
  (`main/v1_envelope@0.0/info/pot_ohms`).
- 1.2 V leaves 0.4 V of headroom for the peaks above the 99.9th percentile. Real: 0.0265 % of v2 samples
  clip (`clipped_fraction` 0.000265).

---

## D4 (the example window is synthetic)

**D4.1** 2 × 120 × 0.1953 = **46.9**.

**D4.2**
- The σ and the √(2/π) cancel in the ratio: WL/MAV = (N − 1)·2π·f_rms/fs.
- e^5.00 ≈ **147** (the unrounded logs give 147.2), so f_rms = 147.2 × 2048/(399 × 2π) = **120 Hz**.
- Rice predicts **47** zero crossings; the window has **48**. Two features, one spectrum.

**D4.3** Between two zero crossings the signal must turn around: at least one **peak or trough**. So
SSC ≥ ZC (56 vs 48 here).

---

## D5

- **176** multiply-adds per decision, and **219** numbers in the model (176 + 11 + 32).
- 0.25 s is 512 samples: (512 − 400)/100 + 1 = **2** windows per class, **22** in all, for **136**
  covariance numbers.
  - The plain estimate is singular: 22 points can't pin down 136 numbers.
  - Shrinkage pulls it toward a scaled identity, which is invertible. That's how 0.25 s alone still
    reaches 49.8 %.
- With ~74,000 windows the sample covariance is already well determined, so the best blend toward the
  identity is close to none. Ledoit-Wolf estimates exactly that trade-off.

---

## D6

**D6.1** conv1 4·32·9 + 32 = **1,184**; bn **64**; conv2 32·64·7 + 64 = **14,400**; bn **128**;
conv3 64·64·5 + 64 = **20,544**; bn **128**; linear 64·11 + 11 = **715**. Total **37,163**.

**D6.2** 1 → conv9: 9 → pool: 10 (jump 2) → conv7: 10 + 6×2 = 22 → pool: 24 (jump 4) → conv5: 24 + 4×4 =
**40 samples = 19.5 ms**.
- That's a tenth of the 195 ms window, so each of the 64 final channels is a local detector.
- The global average pool turns each one into "how much of this pattern is in the window": a learned
  MAV-like feature.

**D6.3** 460,800 + 2,867,200 + 2,048,000 + 704 = **5.38 million**, about **30,500 ×** LDA's 176.

**D6.4** The epoch with the highest validation score is chosen *because* it's highest, so the score is partly
luck on days 51-60. The last epoch involves no choice: **92.2 %** is the fair number to set beside LDA's
single fit (`results/summary.md`, "CNN vs LDA").

**D6.5**
- Gain: from **0.667** (⅔) to 1.5. Polarity: **16** patterns.
- Polarity augmentation rescued the CNN: 68.2 → 86.3 %.
- Gain augmentation made it worse: 60.6 %, worst on the days with a new lead pattern (DEBUG_LOG #4).

---

## D7

**D7.1** 89.0 (6-10 days) → 81.0 (81-120 days): about **8 points** for a tenfold longer gap. It's flat for
the first ~20 days and falls fastest after that.

**D7.2**
- 2 s is 4,096 samples: **37** windows per class, against about **6,727** old windows per class, so the
  fresh data is **0.55 %** of the pooled class. The fit barely notices it.
- What would help: up-weight the fresh windows, or shift the class means toward them (README, "Next
  steps" 1).
- Fresh data alone at 2 s (77.5 %) already matches the 60-day model on the same windows (77.5 %).

**D7.3** A gain change shifts every log-amplitude feature by the same ln g on a day. Subtracting that day's
class-0 features would remove it and raise the accuracy. Instead it moved 88.0 → 87.6 %, so there was no
day-level gain shift to remove: this subject's drift was electrode placement and lead polarity, not gain.
- Caveat: the method assumes class 0 is a stable reference. Class 0, "no motion", is the *loudest* class
  every day, so it may not be rest at all.

---

## D8 (a plausible reading, not a tested result)

- **Supination and pronation** use the same forearm muscles turning opposite ways. Hudgins features carry
  how loud and how fast each channel is, not which way the wrist turns.
- **Key grip → no motion:** key grip is a light, mostly thumb effort, and class 0 is unusually loud here.
- **Hand closed → hand open:** the same flexor/extensor pair, opposite balance.

Say "plausible" in an interview: none of these was tested.

---

## D9

**D9.1** `v2` is the chain and `@0.0` the 60 Hz common-mode on the body in volts: no hum. In
`v2@1.0@20000.0`, 1.0 is 1 V of hum and 20000.0 is the electrode impedance, 20 kΩ.

**D9.2** The résumé says "87%", so it only has to round correctly: 0.87 ± 0.005. The site says "87.3%",
which claims one more digit, so the check is ten times tighter.

**D9.3**
- **No.** The résumé says "emulat**ed** tests", and "emulated" doesn't contain "emulation".
- With no match, the résumé text scanned is empty, so the scan finds no numbers, and prints
  `[OK] no unbacked numbers in: index.html, Troy_Mayer_Resume.pdf`. That OK is about nothing. (Verified by
  running `scripts/check_claims.py`.)
- It also reads `Troy_Mayer_Resume.pdf`. Your current one is `Troy_Mayer_Resume_Oct2026.pdf`, though both
  have the same decoder bullet.
- **Fix:**
  - match `gesture decoding.*?emulat\w*`;
  - point `RESUME` at the current PDF;
  - make the scan *fail* when its regex finds nothing.
- Until then, say "the site's numbers are checked automatically; the résumé's I check by hand". The
  résumé's 31 % and 87 % are themselves checked (`claims.json` → `resume_v1`, `resume_v2`): only the scan
  for *unbacked* numbers misses the résumé.

---

## D10: model answers

**The decoder in 60 seconds.**

> "The orthosis board had a redesign on paper but not on the bench, so I asked what each board would let a
> decoder do. I took 115 days of real four-channel forearm EMG from one subject, ran it through a
> sample-level emulation of each board's analog chain and ADC, and trained an LDA on Hudgins features from
> days 1 to 60. Then I scored every later day without recalibrating. Through the old board it averaged 31 %
> over 11 gestures; through the redesign 87.3 %, within a point of no board at all. A CNN looked better in
> validation but dropped to 68 % on new days. I traced that to electrode leads reversed between sessions,
> using per-channel skewness, and random polarity flips in training brought it to 86.3 %. The limit: one
> subject, sustained holds only, and an emulation, not a recorded board."

1. **"Why LDA?"** "176 multiply-adds a decision and about 200 numbers, so it fits the microcontroller. It's
   immune to lead reversals because its features are even functions, and it matched the CNN once the CNN
   was fixed. The CNN costs about 30,000 times more per window."
2. **"How do you know it isn't leakage?"** "I train on days 1 to 60 and test each later day as a whole, so no
   test window shares a recording with training. The shuffled split, where neighbours overlap 75 %,
   scored 91.3 %, and I don't quote it. I also found and dropped three pairs of byte-identical days. None
   straddled the split, but they double-weighted two test days."
3. **"A new subject?"** "Unknown: it's one person. The days-elapsed curve and the recalibration experiment
   suggest a short calibration on the new user, two seconds per gesture, would be the starting point, with
   the fresh data weighted up, not pooled."
4. **"What did emulation tell you?"** "That v1's problem wasn't noise, it was loading: the electrodes cut its
   gain from 47 to 2.45, and its 0.4 V diode let 0.22 % of samples through. The impedance sweep showed that
   buffering alone would recover half to three-quarters of the loss, and the rectifier the rest. A
   datasheet gives you CMRR; the emulation gives you accuracy."
