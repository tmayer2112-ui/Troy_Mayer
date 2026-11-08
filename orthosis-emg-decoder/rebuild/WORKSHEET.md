# Worksheet: derive it before you code it

Do these on paper. The numbers are small enough to work by hand, and the code checks the same numbers.
Answers are in `ANSWERS.md`: look only after you've written something down. "Background" marks general
theory you can say in an interview but didn't derive yourself.

---

## Module 1: EMG and Hudgins features (day 1, 30 min with setup)

**The idea.** A surface electrode hears thousands of muscle fibres firing. Each motor unit sends a small
spike-shaped wave, a MUAP, a few ms long. Their sum is a noisy, zero-mean signal whose *loudness* tracks
effort and whose *texture* (how fast it wiggles) depends on which muscle and where. Like a crowd: you can't
hear one voice, but you can hear how loud it is and whether it's a cheer or a murmur.

Background: surface EMG is band-limited (most power 20-450 Hz) and heavier-tailed than Gaussian.

**1.1 Windows.** The decoder cuts each channel into windows of 400 samples, starting every 100 samples, at
2048 Hz. One window lasts ____ ms, a new one starts every ____ ms, and two neighbours share ____ % of their
samples.

**1.2 The four features by hand.** One window of one channel, x = [2, −1, −3, 1, 0.5, −0.5], dead band ε = 1.5.
The differences are Δ = x[i+1] − x[i].
- MAV = mean of |x| = ____
- WL = sum of |Δ| = ____
- ZC: count neighbours with opposite signs AND |Δ| ≥ ε = ____. Which crossing does the dead band throw out,
  and why is that the point of a dead band?
- SSC: count places where the slope flips sign (Δ[i]·Δ[i+1] < 0) AND at least one of the two slopes has
  size ≥ ε = ____

**1.3 Why logs.** A loose electrode multiplies the signal by g. What happens to MAV? To log MAV?
For g = 2, how much does log MAV move? ____. Why is a *constant* shift easier for a linear classifier than a
*scaling*? MAV ∝ amplitude and WL ∝ amplitude × frequency, so what does log WL − log MAV measure?

**1.4 Why all four ignore x → −x.** Reverse the lead: x becomes −x. For each feature, say which piece makes
it blind to the sign: |x|, |Δ|, or a product of two neighbours. Compute the features of
−x = [−2, 1, 3, −1, −0.5, 0.5]: same or different?

**1.5 Counting.** A 3 s hold at 2048 Hz is 6144 samples. How many full windows? ____

**Code:** `m1_features.py` (5 functions, about 10 lines), `python check.py 1`, `python run.py features`.

**Say it:** *"Each channel is cut into 195 ms windows every 49 ms, and I compute the four Hudgins features:
mean absolute value, waveform length, zero crossings and slope sign changes, with a small dead band.
Amplitudes are logged, so a gain change is just an offset, and all four are even functions, so a reversed
lead changes nothing."*

**Connect it (5 min):** `emgdec/features.py`, `windows` (line 10) and `hudgins` (17); the dead band is
5 % of each channel's RMS on days 1-5 (`dead_band`, 36). 4 features × 4 channels = 16.

---

## Module 2: the front end: filter, ADC, aliasing (day 1, 40 min)

**The idea.** The board must keep the EMG band and reject the rest: below ~20 Hz is cable sway and electrode
drift, above ~500 Hz is mostly noise. Two op-amp filter stages do it, a high-pass and a low-pass, both
"Sallen-Key". Then the ADC turns volts into numbers, 2048 or 1000 times a second.

**2.1 One stage.** With equal parts (R1 = R2 = R, C1 = C2 = C) and an amplifier gain K, a Sallen-Key stage
has ω0 = 1/RC and Q = 1/(3 − K). K = 1 + 10k/18k.
- K = ____, Q = ____.
- HPF: R = 15 kΩ, C = 470 nF, so RC = ____ ms and f0 = ω0/2π = ____ Hz.
- LPF: R = 1 kΩ, C = 330 nF, so RC = ____ ms and f0 = ____ Hz.
- A Butterworth (maximally flat) stage has Q = 1/√2. What K does that need, and what resistor in place of
  the 18k?

**2.2 Gain at f0.** H_HP(s) = K s² / (s² + (ω0/Q) s + ω0²). Put s = jω0: s² = −ω0². Which two terms of the
denominator cancel? Show |H(jω0)| = K·Q = ____.

**2.3 One stage's −3 dB point.** Setting |H|² = K²/2 with Ω = f/f0 gives Ω⁴ + (2 − 1/Q²)Ω² − 1 = 0.
- Put u = Ω²: a quadratic, u² + b u − 1 = 0, with b = 2 − 1/Q² = ____.
- The positive root is u = (−b + √(b² + 4))/2 = ____, so Ω = ____.
- The HPF alone is −3 dB at ____ Hz.
- The two stages together peak at × 2.405, not K² = 2.42, and their corners sit at 23.0 and 473.9 Hz.
  Why is the peak lower than K²? (Hint: are the two f0 far enough apart that each stage is at full gain
  in the middle?)

**2.4 Going digital: the bilinear transform.** A computer can't run H(s). Substitute
s = c (1 − z⁻¹)/(1 + z⁻¹) with c = 2·fs, where z⁻¹ means "one sample ago".
- **Pre-warp:** the substitution squeezes frequencies near fs/2, so first move ω0 to ω0' = c·tan(π f0/fs).
  For the LPF at fs = 2048 Hz: ω0 = 2π·482.3 = ____ rad/s becomes ω0' = ____ rad/s. By how many %? For
  the HPF it barely moves: why?
- **Coefficients:** multiply top and bottom by (1 + z⁻¹)². With A = c², B = (ω0'/Q)·c and C = ω0'², the
  denominator is (A + B + C) + (2C − 2A) z⁻¹ + (A − B + C) z⁻². For the LPF:
  - A = ____ million, B = ____ million, C = ____ million;
  - a1 = (2C − 2A)/(A + B + C) = ____ and a2 = ____;
  - b0 = K·C/(A + B + C) = ____.

**2.5 The ADC.** The ESP32-S3 reads 0-3.1 V in 12 bits, but noise leaves about 10 useful ("effective") bits.
- One step (LSB) = 3.1 V / 2¹² = ____ mV.
- A uniform error one step wide has SD = step/√12. So the noise that leaves 10 bits has
  SD = (3.1/2¹⁰)/√12 = ____ mV.
- The board centres its signal on 1.65 V. Why there?

**2.6 Aliasing.** Sampling at fs can't tell f from f − fs.
- At the firmware's 1 kHz, where does 700 Hz show up? ____ Hz.
- The band-pass is already −3.5 dB at 500 Hz, −7.4 dB at 700 Hz and −12.9 dB at 1 kHz.
- The real decoder lost 0.4 points at 1 kHz, and lost the same 0.4 with a sharp anti-alias filter in
  front. So was the loss caused by folding, or by something else?

**Code:** `m2_filter.py` (6 functions, about 22 lines), `python check.py 2`, `python run.py filter`.

**Say it:** *"Two equal-component Sallen-Key stages with K = 1.56, so Q = 0.69, slightly overdamped of
Butterworth. Together they pass 23 to 474 Hz with a gain of 2.4. I emulated them with a pre-warped bilinear
transform, within 0.4 dB of the circuit below 400 Hz."*

**Connect it (5 min):**
- `emgdec/board.py`: the constants at lines 27-30, `sallen_key_analog` (40), `sallen_key` (48), `adc` (56),
  `sample_at` (64).
- The numbers are in `results/board.json` and `results/sample_rate.json`.

---

## Module 3: why v1 failed and v2 passes (day 2, 35 min)

**The idea.** v1's first stage was a difference amplifier with 1 kΩ inputs. Skin electrodes are tens of kΩ,
so they sat in series with those 1 kΩ, like trying to listen to someone through a wall. v1 then rectified
the signal through a diode that needs 0.4 V to conduct. v2 uses an instrumentation amp (INA333, 100 GΩ
inputs) and keeps the whole waveform.

**3.1 Loading.** For v1, with R1 = 1k and R2 = 47k:
- a = R2/(R1 + Z+ + R2) · (1 + R2/(R1 + Z−)) and b = R2/(R1 + Z−); G_diff = (a + b)/2 and G_cm = a − b.
- With perfect (zero-impedance) electrodes, a = ____, b = ____, so G_diff = 47 and G_cm = 0.
- With 20 kΩ ± 25 % (Z+ = 22.5k, Z− = 17.5k): a = ____, b = ____, G_diff = ____, G_cm = ____.
- CMRR = G_diff/|G_cm| = ____ = ____ dB. In words: why does the mismatch between the two electrodes,
  not just their size, set the CMRR?

**3.2 Adding the resistor limit.** 1 % resistors alone allow 62 dB. Leakages add: 1/CMRR = 1/13.6 +
10^(−62/20) = ____, or ____ dB.

**3.3 The peak detector.** v1's diode charges a capacitor instantly and a resistor bleeds it:
e[n] = max(e[n−1]·decay, v[n] − 0.4 V).
- By hand: v = [0.5, 0.3, 0.9, 0.1] V, decay = 0.5, starting from e = 0. e = [____, ____, ____, ____].
- τ = C9·R15 = 10 µF × 18 kΩ = ____ ms. At 2048 Hz, decay = e^(−1/(fs·τ)) = ____. The output can only
  change at about 1/(2πτ) = ____ Hz.
- Why are zero crossings always 0 at v1's output?

**3.4 Signal and hum budgets.**
- v1's total gain at max pot: 2.45 × K² (2.42) × 148 = ____. The smallest EMG that reaches the 0.4 V diode:
  ____ µV. Surface EMG is 30-300 µV RMS. What fraction gets through? (The real emulation says 0.22 % of
  samples.)
- v2: common-mode leaks in through the INA333 (100 dB, so 1e−5) plus the electrode imbalance against its
  884 MΩ input: 5 kΩ/884 MΩ = ____. Add the two, then divide by 10 for the right-leg drive (20 dB):
  ____ dB. How much of 1 V of body hum becomes signal? ____ µV.

**3.5 Hum as a dither pedestal (E1).** 10 mV of 60 Hz on the body, through v1's 22.6 dB, leaks in as ____ mV.
- Times the 877.5 gain, that's ____ V at the diode: more than 0.4 V. So on every hum crest, the EMG riding
  on top gets through. (Real: 0.22 % of samples → 20.3 %.)
- Then why did accuracy *fall* (30.8 → 20.7 %)? Hint: each channel's electrode imbalance, which sets how
  much hum it turns into signal and with what sign, changes every day.

**Code:** `m3_frontend.py` (6 functions, about 14 lines), `python check.py 3`, `python run.py boards`.

**Say it:** *"v1's unbuffered 1 kΩ inputs were loaded by 20 kΩ electrodes, so the first stage's gain fell
from 47 to 2.45 and its CMRR to 23 dB. Then a 0.4 V diode passed almost nothing. v2's INA333 has 100 GΩ
inputs, and with the right-leg drive the common-mode rejection is about 116 dB, so 1 V of hum becomes under
2 µV."*

**Connect it (5 min):**
- `emgdec/board.py`: `diff_amp_gains` (97), `_peak_detector` (81), `BoardV2.cm_to_diff` (207), `hum` (256).
- `results/board.json` → `v1_front_end_vs_electrode_impedance` and `v2`.
- Decoder README, "Why v1 fails" and E1.

---

## Module 4: LDA and honest evaluation (day 3, 45 min)

**The idea.** LDA pictures each class as a cloud of points (16 features each). All clouds have the same
shape, only different centres. A new window goes to the class whose centre is closest, measured in "cloud
units". The bigger idea is honesty: the score depends as much on *which windows you test on* as on the model.

**4.1 From Gaussians to a straight line.** Each class k is Gaussian, N(μ_k, Σ), with ONE shared Σ.
- log p(x | k) = −½ (x − μ_k)ᵀ Σ⁻¹ (x − μ_k) + const. Expand the product. One term, −½ xᵀΣ⁻¹x, is the
  same for every class. Why can you drop it, and why does that make the boundary a straight line?
- What's left is the score δ_k(x) = xᵀΣ⁻¹μ_k − ½ μ_kᵀΣ⁻¹μ_k + ln π_k.

**4.2 Tiny LDA.** One feature, two classes: A = {0, 2}, B = {4, 6}.
- The means are μ_A = ____ and μ_B = ____. Pooled variance: subtract each class's own mean, square, add,
  divide by (n − classes) = 4 − 2: σ² = ____.
- δ_A(x) = x μ_A/σ² − μ_A²/(2σ²) and the same for B (equal priors). Set them equal: the boundary is
  x = ____. Is that just the midpoint? When wouldn't it be?
- 2D: μ_A = (0, 0), μ_B = (2, 1), Σ = diag(2, 1). Then w = Σ⁻¹(μ_B − μ_A) = (____, ____), and the boundary
  is x1 + x2 = ____.
- Background: *shrinkage* (Ledoit-Wolf) blends Σ with a multiple of the identity when data is scarce, or
  when a column is constant, as v1's ZC is. Why would a constant column break Σ⁻¹?

**4.3 Balanced accuracy.** 10 windows: 9 of class 0, all predicted right; 1 of class 1, predicted wrong.
Plain accuracy = ____. Balanced accuracy (mean of each class's recall) = ____. With 11 classes, what is
chance? ____ %.

**4.4 Why the shuffled split leaks.** Windows are 400 long and start every 100.
- How many neighbours does a window share samples with? ____ (3 on each side).
- Shuffle and put 25 % in the test set. The chance that none of a test window's 6 neighbours is in
  training is 0.25⁶ = ____. So ____ % of test windows have a near-copy in training.
- By hand, starts 0, 100, …, 900 with windows 0-3 in test: which test windows share a sample with a
  training window? [____, ____, ____, ____]

**4.5 The data audit.** MultiDay had days 5/7, 63/68 and 110/112 byte-identical.
- Why is an identical *byte* copy proof of a copy, when two real recordings of the same gesture never are?
- All six days were dropped rather than one of each pair. Why?

**4.6 Same windows, or it doesn't count (DEBUG_LOG #9).** The headline table said within-day 81.3 % against
cross-day 87.3 %: calibrating on the day *lost*?
- Within-day tests only the last 40 % of each hold. Cross-day scores 92.4 % on the first 60 % and 77.5 % on
  the last 40 %.
- On identical windows the comparison is ____ % vs ____ %. Which wins, and why should it?

**Code:** `m4_lda.py` (6 functions, about 27 lines), `python check.py 4`, `python run.py lda` (~15 s).

**Say it:** *"LDA assumes Gaussian classes with one shared covariance, so the decision boundaries are linear:
16 features times 11 classes is a handful of multiply-adds. I score balanced accuracy, train on days 1-60
and test each later day without recalibrating. A shuffled split of 75 %-overlapping windows leaks, and it
overstated v2 by 4 points."*

**Connect it (5 min):**
- `emgdec/models.py`, `lda` (line 11): scikit-learn, `shrinkage="auto"`.
- `emgdec/protocols.py`: `random_split` (34), `within_day` (42), `cross_day` (58).
- `scripts/audit_data.py`, lines 67-73.
- `results/lda.json` → `main/v2@0.0`.

---

## Module 5: electrode polarity (day 5, 35 min)

**The idea.** Clip a lead on backwards and the channel is multiplied by −1. A decoder built only from
sign-blind features won't notice. A model that looks at the raw waveform will, because a MUAP has a tall
peak and a shallow trough, and reversing it swaps them.

**5.1 Odd and even.** Skewness γ = E[(x − μ)³]/σ³; RMS = √E[x²].
- For x = [0, 0, 3]: μ = 1, the deviations are −1, −1, 2, E[d²] = ____, E[d³] = ____, so γ = ____ and
  RMS = ____.
- For −x: γ = ____ and RMS = ____.
- For 2x: γ = ____ and RMS = ____.
- Fill in: a reversed lead → skewness ____, RMS ____. A gain change → skewness ____, RMS ____.

**5.2 The β test.** Each day gives 11 per-class skewness values per channel, its profile s. Regress it with
no intercept on the reference days' mean profile: β = (s·s_ref)/(s_ref·s_ref).
- s = [0.4, −0.2, 0.6] and s_ref = [0.5, −0.1, 0.5]: β = ____.
- If the lead is reversed, β = ____.
- |β| < 0.4 is "no call". Why test the whole profile rather than the sign of the average skewness?
  (DEBUG_LOG #5: day 51's average was −0.074.)

**5.3 Blind and not blind.**
- Why can't LDA on Hudgins features see a reversal at all? (Module 1.4.)
- Now take a detector f(x) = mean of ReLU(x − 1), where ReLU(v) = max(v, 0). On x = [0, 3]:
  f(x) = ____. On −x: f(−x) = ____.
- That thresholded ReLU after a learned filter is what a CNN's first layer is. What does that mean for a
  CNN trained only on days whose lead pattern it has seen?

**5.4 Augmentation.**
- Each of 4 leads reversed or not: ____ patterns.
- Training with random flips teaches the model that the sign doesn't matter. The real CNN went from 65.5 %
  to 86.0 % on days with an unseen pattern, and lost 2.5 points (91.5 → 89.0) on familiar ones.
- In this rebuild, a linear model on fixed random kernels pays far more. Why can a linear model only become
  sign-blind by averaging, and why would more kernels help?

**5.5 The RMS half (DEBUG_LOG #6).** At each reversal, RMS moved by up to 18 %, and on the channels that
didn't flip too. What does that suggest happened between sessions?

**Code:** `m5_polarity.py` (5 functions, about 7 lines), `python check.py 5`, `python run.py polarity` (~10 s).

**Say it:** *"A CNN validated at 92 % and tested at 68. Matched data and gain drift didn't explain it, but
per-channel skewness did: it flips sign with a reversed lead while RMS barely moves, and from day 69 on every
test day had a lead pattern training never saw. LDA's features are even functions, so it was immune.
Random polarity flips in training brought the CNN to 86.3 %."*

**Connect it (5 min):**
- `scripts/audit_data.py`, line 84 (β).
- `emgdec/models.py`, `train_cnn(flip_aug=True)`.
- Decoder README, E3; DEBUG_LOG #3-#6.
- `results/data_audit.json` → `polarity`.

---

## Module 6: the motor speed loop (day 4, 45 min)

**The idea.** EMG sets a joint *speed*. The motor gets a *duty* (the fraction of the 12 V supply it sees).
Feed-forward is the guess: "this speed needs this much duty". The PI loop fixes what the guess gets wrong,
like the arm's weight. Cruise control: you know roughly how much throttle 60 mph needs, and you press harder
on a hill.

**6.1 The plant and the guess.** A DC motor behind a stiff gearbox is first order: ω/d = K/(τs + 1), with
K = 187 °/s per unit duty and τ = 20 ms.
- At d = 0.1, the speed settles to ____ °/s, reaching 63 % of it after ____ ms.
- Feed-forward inverts this: d_ff = (ω_ref + τ·dω_ref/dt)/K + d_f·sign(ω_ref), with d_f = 0.05.
  - For 15 °/s steady: d_ff = ____.
  - At 10 °/s while ramping at 150 °/s²: d_ff = ____.
  - Which term pays for the back-EMF, which accelerates the inertia, and which breaks friction?

**6.2 IMC tuning.** PI is C(s) = Kp (1 + 1/(Ti s)). Choose Ti = τ, so the controller's zero cancels the
plant's pole.
- Show that the loop gain becomes L(s) = C·P = Kp K/(τ s). With unity feedback, the closed loop is then
  1/(λs + 1) when Kp K/τ = 1/λ.
- So Kp = τ/(Kλ) = ____ and Ki = Kp/Ti = 1/(Kλ) = ____, for λ = 50 ms. Units of each?
- The fast tier uses λ = 30 ms. Why does the firmware unlock it only after `j` has measured the motor?

**6.3 One tick by hand.** ref = 10 °/s, meas = 8 °/s, the previous ref was 10, dt = 1 ms, integral 0.
- e = ____.
- Feed-forward = ____ (no ramp).
- P term = Kp·e = ____.
- duty = ____.
- The integral becomes Ki·e·dt = ____.

**6.4 Anti-windup and the position hold.**
- Part 1: when the output is pinned at ±0.6 and e pushes the same way, why must the integral stop?
  (Run `run.py loop`: with the joint held for 0.5 s, the integral reaches 0.40 with anti-windup and 0.60
  without.)
- Part 2: while the setpoint ramps, the joint always lags a little. Why is that lag the feed-forward's job,
  not the integral's?
- The position hold: at zero command, e = −ω, so the integral is −Ki × (how far the joint has moved). Holding
  the forearm at 40° takes 0.136·sin(85°) = 0.135 duty, and friction gives 0.027 for free. How far does the
  arm sag before the integral holds it? ____ °.

**6.5 Speed from the encoder.** 0.06698 joint degrees per count.
- At 25 °/s that is ____ counts/s, or ____ counts per 10 ms tick.
- Counting gives 3 or 4 each tick: 4 with probability p = 0.73. The error is +0.27 or −0.73 counts, so its
  SD is √(p(1 − p)) = ____ counts, or ____ % of the speed.
- At 3 °/s (0.448 counts per tick): ____ %.
- The firmware instead times the last 4 edges, one full quadrature cycle. Why does that cancel the uneven
  edge spacing?

**6.6 Lifting the forearm.** At 15 °/s from 5°, the forearm's weight costs 0.136·sin(50°) = 0.104 duty.
- Feed-forward gives 0.130, so 0.130 − 0.104 = ____ duty is left to beat 0.027 of friction. Does the arm
  move?
- How does the integral fix it, and why is the closed-loop error not exactly zero while lifting? (Hint: the
  load changes as the angle changes.)

**Code:** `m6_speed_loop.py` (5 functions, about 18 lines), `python check.py 6`, `python run.py loop`.

**Say it:** *"I model the motor as first order, K over tau s plus one, and the feed-forward inverts that. The
PI is IMC-tuned, Kp = tau/(K lambda) and Ki = 1/(K lambda), so the closed loop is first order at lambda,
50 ms. Two anti-windup rules: no integrating into saturation, and no charging on a ramp. Lifting the forearm
at 15 °/s, feed-forward alone stalls; the loop holds the error under 1 % in simulation."*

**Connect it (5 min):**
- `firmware/emg_orthosis/orthosis_core.h`: `Config::kp()/ki()` (151-152), `VelocityPI::step` (432),
  `EdgeTracker` (183), `VelocityEstimator` (207).
- `firmware/results/metrics.json`: `kp_duty_per_dps`, `ki_duty_per_deg`, `lift_15dps_*`.

---

## Module 7: the Bayesian envelope and latency (day 6, 45 min)

**The idea.** To move the arm you need the EMG's *loudness*, smoothed. A plain low-pass filter must choose:
smooth and slow, or fast and jittery. The Bayesian estimator keeps a probability for each possible loudness
and updates it every sample. Like a weather forecaster: tomorrow is probably like today, but a storm can
arrive at any moment, so when the evidence is overwhelming, believe it at once.

**7.1 The levels.** 64 loudness levels from 1 to 2000 ADC counts, log-spaced. Each is ____ % above the last.
Why log-spaced?

**7.2 One sample by hand.** Three levels s = [1, 2, 4], prior p = [0.2, 0.6, 0.2], drift α = 0.1, jump β = 0.
- **Predict, drift:** each level keeps 1 − 2α of its own probability and takes α from each neighbour (an end
  level counts itself as its missing neighbour). q = [____, ____, ____].
- **Update** with the sample e = 2: multiply by the Laplace likelihood exp(−√2·|e|/s)/s, which is
  [____, ____, ____].
- **Normalise:** posterior = [____, ____, ____].
- **Estimate:** exp(Σ p·ln s) = ____.

**7.3 Fast but smooth.**
- What does a tiny jump probability β = 1e−12 buy at a contraction's onset?
- What does a tiny drift α = 1e−4 buy during a steady hold?
- The old envelope was two one-pole low-passes at 4 Hz. Each has T = 1/(2π·4) = ____ ms, and the pair
  reaches 50 % of a step at 1.678·T = ____ ms.

**7.4 Onset with hysteresis and a confirm.** Onset 4, release 3, confirm_n = 2.
- For envelope [0, 5, 5, 5, 0], which samples are active? ____. The channel turns on when the count of
  samples above onset in a row exceeds confirm_n.
- For [5, 5, 5, 5, 3.5, 3.5, 2.9] with confirm 0: ____.
- Why does a jump-capable estimator need a 20 ms confirm when a slow filter didn't? (An electrode pop lasts
  3 ms.)

**7.5 The 43 ms budget** (`firmware/README.md`, "Where the 43 ms goes").
- The muscle's ramp to a detectable level is about 16.5 ms, plus the 20 ms confirm, plus about 7 ms of motor
  spin-up: ≈ ____ ms.
- With the old conservative tuning, the estimator alone took 103 to 97 ms. What else had to change to reach
  43?

**Code:** `m7_bayes.py` (3 functions, about 23 lines), `python check.py 7`, `python run.py latency` (~5 s).

**Say it:** *"The Bayesian estimator keeps a probability over 64 amplitude levels and updates it every
sample: a slow drift prior, a rare jump, and a Laplacian likelihood, because EMG is heavy-tailed. It jumps at
an onset but averages hard in a steady hold. A pop is also a jump, so an onset must hold for 20 ms, which
took false starts to zero. With a faster control tier on top, the joint moves 43 ms after the muscle, down
from 103."*

**Connect it (5 min):**
- `orthosis_core.h`: `BayesEnvelope` (261), `step` (274); `EmgChannel::updateActivation` (359).
- `firmware/analysis/latency_study.py`: `env_bayes` (48), `to_command` (88).
- `firmware/results/latency_study.json` and `latency_sweep.txt`.

---

## Module 8: safety: latching faults and the watchdog (day 5, 25 min)

**The idea.** A motor strapped to an arm must stop when anything looks wrong, and stay stopped until a person
decides it's safe. Monitors watch for symptoms; a fault cuts the driver; a watchdog catches the code itself
hanging.

**8.1 Count them.** Open `orthosis_core.h`, `supervise()` (line 669) and `superviseMotion()` (685).
- List every `return "...";`. How many runtime monitors are there? ____. Does that match "10 latching fault
  monitors" on your résumé?
- `firmware/sim/test_closed_loop.cpp` has tests named `Safety.*` (lines 165, 187, 218). Which monitors do
  they exercise? ____ of the ____.

**8.2 Persistence.** A monitor trips only when its condition holds for `ms` in a row (`timer`, line 806).
- At 1 kHz, a 20 ms monitor trips on the ____th consecutive true tick.
- A 19-tick blip, then 1 false tick, then 20 true ticks: on which tick does it trip?
- Why hold times at all? Why 400 ms for stall but 60 ms for runaway?

**8.3 Why latch.** A stall fault cuts the motor, the motor stops pushing, and the stall condition goes away.
- What would happen if the fault cleared itself?
- Clearing (`d`) goes to IDLE, not ARMED. What does the separate arm step re-check?

**8.4 The watchdog.**
- If the 1 kHz loop hangs (a bad sensor read, an infinite loop), can a check *inside* that loop fire?
- The ESP32-S3's task watchdog is fed every tick and resets the chip after ____ ms without a feed
  (`emg_orthosis.ino`, line 335).
- After the reset, what keeps the motor driver off while the pins float? (Firmware README, "Watchdog".)

**8.5 The toy runs.** `run.py safety` blocks the joint, then reverses the encoder. Predict which monitor
trips in each case before you run it.

**Code:** `m8_safety.py` (4 functions, about 13 lines), `python check.py 8`, `python run.py safety`.

**Say it:** *"Ten runtime monitors, from driver fault and lead-off to stall and runaway, each with a hold time
so a blip can't trip it. Any of them latches a fault that coasts the motor, and only a deliberate clear and
re-arm brings it back. A task watchdog outside the control loop resets the chip in 200 ms if the loop hangs,
and pull-downs keep the driver off during the reset."*

**Connect it (5 min):**
- `orthosis_core.h`: `supervise` (669), `superviseMotion` (685), `fault` (793), `timer` (806).
- `firmware/sim/test_closed_loop.cpp`, `Safety.*` (165-244).
- `firmware/README.md`: "Encoder sign", "Watchdog".

---

## Module 9: the speed loop in C++17 (day 7, 45 min; set up WSL on day 2)

You port Module 6's `imc_gains`, `feedforward` and `VelocityPI.step` to a header-only C++17 file, in the
style of `orthosis_core.h`.

**9.1 Python → C++.** Fill in the C++ column:

| Python | C++17 |
|---|---|
| `return tau / (K * lam), 1.0 / (K * lam)` | `return {____, ____};` into `struct Gains { double kp, ki; }` |
| `math.copysign(friction, ref)` | `std::________(friction, ref)` |
| `float(np.clip(v, -m, m))` | `std::______(v, -m, m)` (in `<algorithm>`) |
| `abs(ref) > 0.5` | `std::______(ref) > 0.5` |
| `True and not x` | `true && !x` |

Why is every function in a header marked `inline`?

**9.2 The step.** Write it in the same order as the Python. Why does the order matter for the parity test?

**9.3 Parity.** The test feeds 4500 ticks of the Python reference's inputs into your C++ step and compares
the outputs. Both use IEEE double precision with the same operations in the same order, so how big should
the difference be? A second test compares your port with the firmware's own float `VelocityPI`. How big
should *that* difference be, and why? (float keeps about 7 significant digits.)

**9.4 Timing.** `./build/rebuild_bench` times your step, the firmware's PI, and the firmware's Bayesian
envelope on 4 channels.
- Which uses more of the 1 ms tick, and why? (64 `exp()` per channel per sample.)
- Why isn't your laptop's number the ESP32-S3's?
- Why does the firmware use float on that chip?

**Build and run** (README, "Setup"):
```
cd cpp
cmake -S . -B build
cmake --build build
./build/rebuild_tests_solution
./build/rebuild_tests
./build/rebuild_bench
```
`rebuild_tests_solution` checks your setup; `rebuild_tests` runs your `velocity_pi.hpp`.

**Say it:** *"The control core is a header-only C++17 file with no hardware dependencies, so the same code
compiles into the firmware and into GoogleTest on a laptop. I test numbers worked out by hand, behaviour on a
plant model, and parity against a reference implementation."*

**Connect it (5 min):**
- `orthosis_core.h` lines 151-152 and 432.
- `firmware/CMakeLists.txt` (it builds the same header against GoogleTest).
- `firmware/sim/test_core.cpp`, `TEST(VelocityPI, AntiWindupStopsIntegratingIntoSaturation)` (line 225).

---

## Module 10: the report, your real numbers, the retell (days 6-7, 35 min)

**10a (day 6, 15 min).**
- Run `python run.py report` (add `--plot` if matplotlib is installed). In each section, find the line
  where the synthetic number sits next to the real one. Name the one place the toy disagrees most, and
  say why (README, "What's simpler").
- Read `REAL_NUMBERS.md` out loud, start to finish.

**10b (day 7, 20 min).**
- Close everything. On one page, draw the chain: electrodes → INA333 → band-pass → ADC → DC track →
  Bayesian envelope → onset confirm → speed command → shaper → feed-forward + PI → motor → encoder edges
  → speed.
- Give the 90-second retell of the whole orthosis entry: problem, board, decoder, controller, latency,
  safety, one limit. Record it once.
- Then a 10-minute mock interview with Claude on the questions at the end of `REAL_NUMBERS.md`.

---

## Module 11 (optional): your loop on a MuJoCo elbow (+30 min, in WSL)

A hinge carries a 2 kg forearm with its centre of mass 17.8 cm out: 3.5 N·m of gravity at most, the
firmware's `limb_mgr`. Your Module 6 loop drives it at 1 kHz through a DC-motor model.

**11.1 The motor.** The current is i = (d·V − Ke·ω_m)/R with ω_m = N·ω_joint, and the joint torque is
η·N·Ke·i, with N = 191.95, Ke = 0.0186, R = 1.3 Ω, V = 12 V and η = 0.783.
- At d = 0.5 and stall, the torque is ____ N·m.
- The ω term is a damper: η·N²·Ke²/R = ____ N·m per rad/s. Which part of the first-order model is that?
- MuJoCo's `armature` = J_motor·N² = ____ kg·m². Why does a 3.5e−6 kg·m² rotor matter more than the forearm
  (0.085)?

**11.2 What the sensor reports.** `jointvel` is in ____ per second. Your loop wants °/s.

**11.3 Compare.** Lifting at 15 °/s and holding at 40°: MuJoCo against the firmware's simulation. The hold
differs (MuJoCo creeps). Look up MuJoCo's `frictionloss`: why can't a soft constraint hold the way stiction
does?

**Code:** `m11_mujoco.py` (2 functions, 3 lines), `~/venv/bin/python check.py 11`,
`~/venv/bin/python run.py mujoco`.
