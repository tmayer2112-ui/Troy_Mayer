# Decoder worksheet: the EMG decoder, end to end (optional, about 65 min)

Modules 1-5 of `WORKSHEET.md` rebuild the decoder's parts: features, filter, boards, LDA and polarity.
This worksheet goes over what they skip, so you can walk an interviewer through the whole
`orthosis-emg-decoder/` project and defend every number in it. It's paper only, like the main worksheet,
with tiny numbers, a "Say it" line and a "Connect it" line per section. Answers are in
`DECODER_ANSWERS.md`, and `python solutions/decoder_numbers.py` recomputes every one of them.

**When:** after Day 5, or the evening before an interview that will be about the decoder. It's outside the
6.5-hour core; skip it first if you're short on time.

Paths are relative to `orthosis-emg-decoder/`. "Background" marks general theory you can say but didn't derive.

---

## D1. The pipeline, by the numbers (10 min)

**The idea.** One sentence per stage, in order. If you can't say the stage, you can't defend the number at
the end.

**D1.1 Days.** MultiDay has 121 recording days. Six are dropped as byte-identical copies, which leaves ____
days. Training is days ≤ 60 (____ days), testing days > 60 (____ days).

**D1.2 Windows.** Each class is one sustained hold of about 6 s at 2048 Hz. The first 0.5 s is trimmed, then
windows of 400 samples start every 100.
- A 6.0 s hold gives ____ windows.
- One training day (11 classes) gives ____ windows, and 58 days give ____.
- The README says "about 74,000 training windows". How long must the average hold be for that? ____ s.

**D1.3 Put the stages in order:** z-score, LDA fit, board emulation (gain, filters, clipping, ADC), trim,
per-day balanced accuracy, windows, 16 Hudgins features, dead band, mean ± SD over test days.

**D1.4 Who sees which days?** For each step, write *days 1-5*, *training days*, or *every day*:
- the pot trim;
- the ZC/SSC dead band;
- the z-score's mean and SD;
- the LDA fit;
- the board emulation itself.

Why must the test days never reach the first four?

**Say it:** *"Each recording goes through an emulation of the board, then 195 ms windows, 16 Hudgins
features and an LDA trained on days 1-60, and I score every later day on its own. Anything tuned (pots, dead
band, scaler) sees calibration or training days only, like trimming a board on the bench."*

**Connect it:** `emgdec/pipeline.py` (`run_chain`, `build_table`), `emgdec/data.py` (`DUPLICATE_DAYS`),
`results/data_audit.json`.

---

## D2. Why the first 0.5 s is thrown away (5 min)

- The high-pass settles with its pole's time constant 2Q/ω0. With Q = 0.692 and f0 = 22.58 Hz, that is
  ____ ms. So the HPF alone needs about 50 ms.
- v1's peak detector holds anything it catches at start-up with τ = 180 ms. After 0.5 s, e^(−0.5/0.18) =
  ____ of it is left.
- Which of the two set the 0.5 s?

**Say it:** *"I trim half a second so neither filter's start-up transient lands in a window; the slowest is
v1's 180 ms peak detector, about 6 % left after 0.5 s."*

**Connect it:** `emgdec/features.py` `TRIM_S`; README, "The rest of the chain".

---

## D3. Trimming the pots like on the bench (5 min)

`calibrate_pots` picks, per channel, the smallest gain that puts the 99.9th percentile of |output| at the
target, using days 1-5 only.
- **v2:** the gain stage is G3 = 1 + (10k + pot)/1k, and the target is 1.2 V. If the 99.9th percentile of
  the signal entering the gain stage is 20.69 mV, then G3 = ____ and pot = ____ kΩ.
- **Check against the real channel 1:** pot 46,962 Ω. Total gain = 10.107 (INA333) × 1.5556² (the two
  Sallen-Key stages) × (1 + (10k + 46,962)/1k) = ____. Compare `lda.json` → `main/v2@0.0/info/total_gain`.
- **v1:** the target is 1.6 V, G3 = 1 + (47k + pot)/1k, and the pot stops at 100k, so the most gain v1 can
  have is ____. If the 99.9th percentile entering v1's gain stage is 0.5 mV, it wants a gain of ____.
  What happens? (README: "v1 hits the limit on every channel".)
- **Why 1.2 V for v2, when its swing is ±1.6 V?** What happens to the 0.1 % of samples above the 99.9th
  percentile? (Real: 0.03 % of v2 samples clip, `.../info/clipped_fraction`.)

**Say it:** *"Each pot is trimmed once, on days 1-5, to put the 99.9th percentile at 1.2 V, leaving headroom
below the 1.6 V swing. v1 couldn't reach its target on any channel: its pots all sat at the 100 k limit."*

**Connect it:** `emgdec/board.py` `calibrate_pots` (line 242); `lda.json` → `main/*/info/pot_ohms`.

---

## D4. What the features actually measure (10 min)

Background (Rice, for Gaussian noise): a signal whose power spectrum has moments m_n = ∫ fⁿ S(f) df crosses
zero 2·√(m2/m0) times per second. f_rms = √(m2/m0) is its RMS frequency.

**D4.1** A window lasts T = 400/2048 = 0.1953 s. If f_rms = 120 Hz, expect ZC ≈ 2·f_rms·T = ____.

**D4.2 WL/MAV is a frequency.** For a smooth signal, consecutive samples differ by about 2π·f_rms·σ/fs, so
WL ≈ (N − 1)·√(2/π)·2π·f_rms·σ/fs, while MAV ≈ √(2/π)·σ.
- Show that WL/MAV ≈ (N − 1)·2π·f_rms/fs: the amplitude cancels.
- `python run.py features` prints the first window of a synthetic hold (channel 1): logMAV 4.40 and
  logWL 9.40. So WL/MAV = e^(9.40 − 4.40) = ____, which gives f_rms = ____ Hz.
- Rice then predicts ZC ≈ ____. The same output measured ZC = 48. Close?

**D4.3** Background: SSC counts local peaks and troughs, about 2·√(m4/m2) per second. Why must
SSC ≥ ZC for a smooth signal? (Between two zero crossings there is at least one ____.) The synthetic
window has SSC 56.

**Say it:** *"The four features are two amplitudes and two frequencies. MAV and WL scale with amplitude, the
ratio WL/MAV is an RMS frequency, and ZC and SSC count crossings and peaks, which Rice's formula ties to the
spectrum. That's why logs work: the difference of two logs is gain-free."*

**Connect it:** decoder README, "Features" (the ≈ lines); `emgdec/features.py` `hudgins` (line 17).

---

## D5. LDA's size, and why shrinkage (5 min)

- One decision computes 11 scores of 16 multiply-adds each: ____ multiply-adds. The whole model is W
  (16 × 11), b (11) and the scaler's mean and SD (2 × 16): ____ numbers.
- The fresh-only recalibration trains on the first 0.25 s of each class on the new day. How many 400-sample
  windows fit in 0.25 s at 2048 Hz? ____ per class, or ____ in all. A 16 × 16 covariance has ____ different
  numbers to estimate. What does shrinkage do here?
- With ~74,000 training windows, Ledoit-Wolf's α comes out near zero for v2. Why is that the right answer?

**Say it:** *"An LDA decision is 176 multiply-adds, and the model is about 200 numbers, so it fits on the
ESP32 easily. Shrinkage only matters when data is scarce or a feature is dead: v1's zero crossings are
constant, which makes the covariance singular."*

**Connect it:** `emgdec/models.py` `lda` (line 11); README, "Decoders, LDA".

---

## D6. The CNN by the numbers (10 min)

The architecture (README, "1D CNN"): Conv1d 4→32 (kernel 9), BatchNorm, ReLU, MaxPool 2; Conv1d 32→64 (k 7),
BN, ReLU, MaxPool 2; Conv1d 64→64 (k 5), BN, ReLU; global average pool; dropout; Linear 64→11.

**D6.1 Parameters.** A Conv1d has in × out × k weights plus out biases; a BatchNorm has 2 per channel.
- conv1 ____, bn ____, conv2 ____, bn ____, conv3 ____, bn ____, linear ____.
- Total ____ (README: 37,163).

**D6.2 Receptive field.** Start at 1 sample with a jump of 1. Each layer adds (k − 1) × jump; a stride-2 pool
then doubles the jump. Go layer by layer: conv k9 → pool 2 → conv k7 → pool 2 → conv k5. The receptive field
is ____ samples, ____ ms. Compare it with the 195 ms window: what does the global average pool then
compute?

**D6.3 Cost.** Multiply-adds per window: 4·32·9·400 + 32·64·7·200 + 64·64·5·100 (+ the linear layer) =
____ million, which is about ____ times LDA's 176.

**D6.4 Validation.** The epoch is picked on days 51-60: the best epoch scores 92.5 % there and the last epoch
92.2 %. Why is the best-epoch number optimistic by construction? Which one should you compare with LDA's
single fit?

**D6.5 Augmentation.** Gain: × e^u with u uniform on ±ln 1.5, so between ____ and 1.5. Polarity: each
channel × ±1, which is ____ patterns. Which one rescued the CNN, and which one made it worse? (Module 5.)

**Say it:** *"The CNN is three 1-D convolutions and 37 thousand parameters, about 5 million multiply-adds a
window against LDA's 176. Its receptive field is 20 ms, so it's 64 learned detectors averaged over the
window, a learned MAV and WL that nothing forces to ignore sign."*

**Connect it:** `emgdec/models.py` `EMGNet` (line 22), `train_cnn` (45); `results/cnn.json`;
`results/summary.md`, "CNN vs LDA".

---

## D7. Time, recalibration and normalisation (8 min)

**D7.1 Decay.** LDA trained on 5 consecutive days, scored by days since training (`lda.json` →
`main/v2@0.0/day_gap`): 89.9 % (1 day), 90.0 (2-5), 89.0 (6-10), 88.1 (11-20), 85.5 (21-40), 82.0
(41-80), 81.0 (81-120). Roughly how many points are lost going from about 10 days to about 100 days? ____.
Is the loss fastest early or late?

**D7.2 Recalibration (E5).** Scored on the last 40 % of each test day:
- fresh data alone: 49.8 % (0.25 s per class), 65.0 (0.5), 71.7 (1), 77.5 (2 s);
- days 1-60 plus the fresh data: 77.5, 77.5, 77.5, 77.5, 77.6.

Questions:
- 2 s per class is ____ fresh windows per class. The old data has about 74,000/11 = ____ per class. So
  the fresh windows are ____ % of the pooled class.
- Why can't pooling help? What would? (README, "Next steps" 1.)

**D7.3 Reference normalisation.** Subtract each day's class-0 log-amplitude features: 88.0 → 87.6 % (classes
1-10). What would a real day-to-day gain change have done to that number? What does "it didn't help" tell
you about this subject's drift? And which caveat about class 0 matters here? (It is the loudest class every
day.)

**Say it:** *"Accuracy decays slowly, about 9 points from a day to a hundred days. Two seconds per class of
same-day data alone matches 58 old days, but pooled it's swamped, under 1 % of the class. Normalising by a
rest recording changed nothing, so this subject's drift wasn't a gain change."*

**Connect it:** `emgdec/protocols.py` `day_gap_curve` (82), `recalibration` (99), `reference_normalised`
(123); README E5.

---

## D8. The confusions (3 min)

The most common v2 confusions (README, "Results"):
- key grip → no motion, 18.9 % of key-grip windows;
- supination → pronation, 16.0 %;
- pronation → key grip, 10.3 %;
- hand closed → hand open, 10.2 %.

Pick two and say why those pairs might look alike to amplitude and frequency features. (This is a guess, so
say it as one: which muscles are shared, and what do the features *not* encode?)

---

## D9. Every published number has a key (5 min)

`claims.json` maps each résumé and site number to a file and key, and `make claims` checks them.

**D9.1 Read a key.** `results/lda.json` → `main/v2@0.0/cross_day/per_day/mean`. What do `v2` and `@0.0`
mean? In `sweeps/v2@1.0@20000.0`, what are 1.0 and 20000.0?

**D9.2 Tolerances.** "87.3 %" is checked against 0.873 ± 0.0005, and "87 %" against 0.87 ± 0.005. Why the
different tolerances?

**D9.3 A check that passes because it checked nothing.** `scripts/check_claims.py` finds the résumé's decoder
text with the regex `gesture decoding.*?emulation`. Your résumé says "…from 31% to 87% in emulated tests."
- Does the regex match?
- If not, what text does the scan check, and what does it print? (It prints "[OK] no unbacked numbers in:
  …, Troy_Mayer_Resume.pdf".)
- Which PDF does it read, and is that your current résumé?
- How would you fix it?

**Say it:** *"Every number I publish has an entry in claims.json pointing at the results file and key that
produced it, and make claims fails the build on a mismatch."* Only say the last part about the résumé once
D9.3 is fixed (DECODER_ANSWERS.md, D9.3).

---

## D10. The decoder in 60 seconds (5 min)

Close everything and say it out loud: the problem, the data, the boards, the features and model, the honest
protocol, the result, the CNN story, one limit. Record it once. Then answer these four out loud, without
notes:
1. "Why LDA and not a neural network?"
2. "How do you know 87 % isn't leakage?"
3. "What happens on a new subject?"
4. "What did emulating the boards tell you that a datasheet wouldn't?"

Model answers are in `DECODER_ANSWERS.md`.
