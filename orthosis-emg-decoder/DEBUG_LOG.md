# Debug log

One entry per problem: **symptom**, **hypothesis**, **how it was tested**, **actual cause**, **fix**, and a
command that brings the failure back.

Entries 1–4 were reconstructed on 2026-10-05 from the README and the committed results. Their numbers are the
committed ones; where a step wasn't written down at the time, the entry says so. Entries 5–8 were recorded on
2026-10-05, and 9–12 on 2026-10-14 after an external review. Append new ones as they happen.

`repro N` below means `PYTHONPATH=. python3 scripts/debug_repro.py N` (entry 1 needs `make data`).

---

### 1. v1 within-day accuracy of 93–95 %, from a board that passes almost no signal
**Symptom:** A first version scored v1 at 93–95 % within-day, while its cross-day accuracy sat near 30 % and
only a fraction of a percent of its samples cleared the rectifier.
**Hypothesis:** Not recorded.
**Actual cause:** The hum level was drawn once per *recording*. Each recording holds exactly one class, and
the within-day protocol trains and tests on two halves of the same recording. So the hum amplitude
identified the recording, and with it the class. Cross-day trains and tests on different recordings, which
hides the leak.
**Fix:** The level is drawn per day (`_rng(day, 0, "hum-level")` in `pipeline.run_chain`), the phase per
recording.
**Confirmed on 2026-10-05:** Putting the per-recording draw back takes v1 (envelope) within-day accuracy at
10 mV from 46.7 % to 92.3 %. Cross-day barely moves (36.6 → 34.3 %).
**Rule:** Draw every random nuisance parameter at the level where it really varies (day, room, cables), never
at a level that coincides with the label.
**Repro:** `repro 1`.

### 2. The same recording under two day numbers
**Symptom:** Found while debugging entry 3; what first gave it away wasn't recorded.
**Test:** MD5 of each day's 11 files concatenated.
**Actual cause:** Days 5/7, 63/68 and 110/112 are byte-identical in the published files. Real EMG can't repeat
sample for sample, so these are copies.
**Fix:** All six days are dropped (`data.DUPLICATE_DAYS`), since which number is real can't be known. An early
run kept them. The audit now also checks the hashes against `DUPLICATE_DAYS`
(`duplicate_days_match_data_py`).
**What it had broken:** No pair crosses day 60, so no training data reached the test set. Two test days were
double-weighted, and the days-elapsed curve scored a model on its own training data: a block containing day
63 was "tested" on day 68 and filed under a 5-day gap.
**Repro:** `make audit` → `identical_day_groups`.

### 3. CNN validates at 92.5 % and tests at 68.2 %
**Symptom:** Trained on days 1–50, the CNN picked its epoch at 92.5 % on days 51–60, then scored 68.2 % on
days 61–121. LDA scored 87.3 % on the same board output.
**Hypothesis 1:** It gets less data than LDA (every 4th window, days 1–50 only).
**Test:** LDA trained on exactly the CNN's windows (`lda_control.py`): 86.9 %. **Falsified.**
**Hypothesis 2:** Slow drift in electrode gain, the textbook cause.
**Test:** Per-day accuracy shows a cliff, not a slope. Gain augmentation made it worse (entry 4).
Normalising each day by a reference recording moved v2's LDA by at most 0.4 points. **Falsified.**
**Actual cause:** Electrode lead reversals. A channel's skewness flips sign while its RMS barely moves, and
from day 69 on every test day has a polarity pattern absent from training. Hudgins features are even
functions of the signal, so LDA can't see a reversal. The CNN's first convolution is linear, so it sees one.
**Fix:** Random per-channel polarity flips during training: 86.3 %, level with LDA. Per day (computed on
2026-10-05): the plain CNN scores 91.5 % on days 61–67 and 65.5 % from day 69; with polarity flips, 89.0 %
and 86.0 %.
**Left over:** The invariance is learned only for the case it was shown. See README, Next steps.
**Repro:** `repro 3`.

### 4. The textbook fix for cross-day drift made the CNN worse
**Symptom:** Per-channel gain augmentation (log-uniform, ×⅔–×1.5) took the v2 CNN from 68.2 to 60.6 %, and
the no-board CNN from 68.5 to 58.9 %.
**Test (2026-10-05, from `cnn.json`):** Split by day, gain augmentation leaves days 61–67 unchanged (91.4 vs
91.5 %) and makes days 69–121 worse (57.0 vs 65.5 %). It hurts only where the polarity is new.
**Actual cause:** Not established. A plausible reading: independent per-channel gains scramble the amplitude
ratios between channels, one of the strongest class cues, so the network leans harder on waveform shape,
including the sign-sensitive parts that break when a lead reverses.
**Fix:** None; gain augmentation is reported, not used. It did show that this subject's day-to-day change
wasn't gain.
**Repro:** `repro 3`.

### 5. The audit reported one-day polarity changes on days 1 and 51
**Symptom:** The audit's segments included day 1 alone (`+--+`) and day 51 alone (`-++-`) between longer runs.
**Hypothesis:** Real one-day lead reversals, or channels whose class-averaged skewness happened to sit near zero.
**Test:** The day-51 channel-1 mean skewness is −0.074; day 1's channel 1 and channel 4 are +0.026 and +0.044.
Typical magnitudes are 0.2–1. A reversal negates the whole 11-class profile at roughly the same size, so
each day's profile was regressed on the mean profile of days 26–50 (β). Day 51 channel 1 gives β = +0.28, and
day 1 gives −0.37 and −0.10: weak, and on the same side as their neighbours.
**Actual cause:** The test called a sign from a class average with no magnitude floor.
**Fix:** `audit_data.py` now calls polarity from β and makes no call when |β| < 0.4. Nine of 460 day-channels
fall below that, none against its neighbours. That leaves five segments, and adds a channel-1 reversal on
day 116 that the README's list ("26, 68 and 105") had left out. The headline conclusion is unchanged: every
test day from 69 on has a pattern absent from training. The old segments stay in `data_audit.json` for
comparison.
**Repro:** `repro 5`.

### 6. The RMS half of the reversal argument had never been computed
**Symptom:** The README said a reversal is "skewness flips while RMS doesn't", but `audit_data.py` only
computed skewness.
**Test:** Mean RMS over the 5 kept days after each change, divided by the 5 before. The flipped channels give
0.90–1.18. Where nothing changes, the same ratio runs 0.94–1.11 (5th–95th percentile). At each change, the
channels that didn't flip moved by similar amounts (day 106: +7 to +18 % on all four).
**Actual cause:** Nothing was wrong with the conclusion, but "unchanged" was too strong. Amplitude steps a
little on every channel at each change. That looks like electrodes re-applied with leads swapped, not a
channel moving onto another muscle.
**Fix:** The audit reports the ratios (`polarity.rms_at_changes`) and the README says "barely moves".
**Repro:** `repro 6`.

### 7. The study samples at 2048 Hz; the firmware samples the board at 1 kHz
**Symptom:** `firmware/emg_orthosis/orthosis_core.h` has `fs_hz = 1000`, while every decoder number was
computed at the dataset's 2048 Hz. The board's only effective anti-alias filter is the 482 Hz low-pass, which
is just 3.5 dB down at 500 Hz.
**Hypothesis:** 500–1000 Hz content aliases into the band and shifts ZC and SSC.
**Test:** `board.sample_at` re-samples the emulated analog node at 1 kHz, with and without a sharp FIR
anti-alias filter at 500 Hz (`make rate`). `tests/` checks that a 700 Hz tone lands at 300 Hz without the
filter and vanishes with it.
**Result:** Cross-day 87.3 % at 2048 Hz, 86.9 % at 1 kHz, 86.9 % at 1 kHz behind the filter. Aliasing
costs nothing; the 0.4 points come from the band above 500 Hz. Hypothesis falsified for this decoder.
**Fix:** None needed. Documented as E4. The emulation can't contain anything above 1024 Hz, so aliasing from
higher up isn't covered.
**Repro:** `make rate`.

### 8. The docs had drifted from the code
**Symptom:** `train_cnn` defaulted to 15 epochs while `run_cnn.py` passed 8, and the README never gave the
number. The Makefile said LDA takes ~20 min and the README ~30; a full run takes 250 s on 4 cores. The
summary's "CNN val" was the best of 8 epochs, optimistic by construction, beside a single-fit LDA score.
The `hum()` docstring said "per channel" for one waveform shared by all four channels.
**Fix:** Default set to 8, the times measured, a last-epoch CNN validation column added (92.2 % vs 92.5 %
best-epoch for v2, so the conclusion stands), the docstring corrected. `make claims` now checks every
published number against `results/`.

### 9. Within-day (81.3 %) scored below cross-day (87.3 %), though calibrating on the day should help
**Symptom:** The headline table put a same-day model 6 points below a 60-day-old one. Raised in an external
review on 2026-10-14.
**Hypothesis:** The two columns score different windows. Within-day tests only the last 40 % of each
recording; cross-day tests every window.
**Test:** The same cross-day model (v2) scored on the 57 test days: 92.4 % on the first 60 % of each
recording, 77.5 % on the last 40 %, 87.3 % on everything. Against the first 60 %, the last 40 % has lower RMS
(median × 0.92) and lower mean frequency (median × 0.94, lower in 94 % of recordings).
**Actual cause:** A table artefact. The late part of each hold is harder for every model, and only the
within-day column was scored on it.
**Fix:** `cross_day` also reports its accuracy on the first 60 % and last 40 % of each recording, and
`within_day` reports its test-day mean. `summary.md` and the README compare them on identical windows: on
v2, 80.8 % (within-day) vs 77.5 % (cross-day), so calibrating on the day does beat the old model. The
headline's within-day column is labelled with the windows it scores.
**Repro:** `make lda report` → "Within-day vs cross-day on identical windows" in `summary.md`.

### 10. A section titled "same test windows" mixed day sets
**Symptom:** `summary.md` said "All three score the last 40 % of each of days 61-121", then quoted
within-day over all 115 days. Raised in the same review.
**Fix:** That bullet now uses the test-day mean (80.8 %, not 81.3 %).

### 11. Identical hum on every channel made v1 look better with hum than without
**Symptom:** v1 gained up to 18 points of cross-day accuracy at 10 mV of 60 Hz common-mode (rectified 22.3
→ 40.5 %). The review asked whether each channel shouldn't see its own share of the hum.
**Hypothesis:** The hum works as dither, lifting the EMG over the SS14's 0.4 V drop. The share of samples
above the diode went from 0.22 % to 23.9 %. But the model put the *same* waveform on all four channels at a
fixed size every day, so the dither was the same every day, which a real board wouldn't give.
**Test:** `hum()` now takes a signed coupling per channel (`cm_coupling`). Each channel's electrode imbalance
is redrawn per day (uniform on ±50 %, averaging the old 25 %), and the sign of each channel's own CMRR error
is drawn once per board.
**Result:** The dither still happens (20.3 % of samples above the diode at 10 mV), but it no longer helps
across days. v1 envelope at 10 mV: 36.6 → 20.7 % (below its no-hum 30.8 %). Rectified: 40.5 → 28.7 %. At
100 mV: 13.8 → 9.3 % and 12.7 → 11.6 %. v2 barely moves: 87.1 → 87.2 % at 100 mV, and 84.1 → 82.8 % at 1 V.
No-hum results are unchanged.
**Fix:** The per-channel model is the default; `tests/` checks its size against `cm_to_diff` and its sign.
**Repro:** `make lda report` → the hum sweep table.

### 12. The emulated INA333 gain didn't match the schematic, and a firmware comment named the wrong parts
**Symptom:** `board.py` set R_G = 11 kΩ (G = 10.091). The schematic has R21 + R22 = 2 × 5.49 kΩ = 10.98 kΩ
(G = 10.107). Separately, `emg_orthosis.ino` credited the driver pull-downs to R67–R70, which are channel-4
parts on page 5 of the PDF.
**Fix:** `ina_gain = 1 + 100e3 / (5.49e3 + 5.49e3)`. Calibration re-trims the pots, so no no-hum number
changed beyond 0.1 point (the LDA control: 86.8 → 86.9 %). The firmware comment now names R2 and R3 (PWM), R6
(MOT_EN) and R5 (EN_ISO), checked against page 1. `cnn.json` predates the gain fix (0.16 % on the INA333)
and wasn't re-run, because torch wasn't available here.
