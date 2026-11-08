# Answers

Check one module at a time, after you've written your own answer. Hand-calculation numbers are recomputed
by `python solutions/worksheet_numbers.py`. Run numbers come from `python run.py report` and are
**synthetic**: yours will match if your code is right. Real numbers name their file.

---

## Module 1

**1.1** 400/2048 = **195.3 ms**; 100/2048 = **48.8 ms**; neighbours share 300 of 400 samples, **75 %**.

**1.2** x = [2, −1, −3, 1, 0.5, −0.5], Δ = [−3, −2, 4, −0.5, −1].
- MAV = (2 + 1 + 3 + 1 + 0.5 + 0.5)/6 = 8/6 = **1.333**.
- WL = 3 + 2 + 4 + 0.5 + 1 = **10.5**.
- ZC: the sign changes are (2, −1) with |Δ| = 3, (−3, 1) with 4, and (0.5, −0.5) with 1.0. The last is
  under 1.5, so **ZC = 2**; without the dead band it would be 3. That tiny crossing is the kind of wiggle
  noise makes around zero, which is exactly what the dead band is for.
- SSC: the slope pairs are (−3, −2) no, (−2, 4) yes, (4, −0.5) yes, (−0.5, −1) no, so **SSC = 2**.

**1.3** MAV → g·MAV, so log MAV → log MAV + ln g: a constant shift, **+0.693** for g = 2. A linear model
handles an offset with one bias term, while a scaling stretches every class's cloud differently.
log WL − log MAV ≈ log(frequency): a gain-free measure of how fast the signal wiggles.

**1.4** MAV uses |x|, WL uses |Δ|, ZC uses x[i]·x[i+1] (and (−a)(−b) = ab) plus |Δ|, and SSC uses
Δ[i]·Δ[i+1] plus |Δ|. All even, so −x gives exactly the same four numbers. `run.py features` confirms it on
real-sized windows: the maximum change is 0.0.

**1.5** (6144 − 400)/100 + 1 = 58.44 → **58** windows (the 59th would run past the end).

---

## Module 2

**2.1** K = **1.5556**, Q = 1/(3 − 1.5556) = **0.6923**.
- HPF: RC = 15k × 470n = **7.05 ms**, so f0 = **22.58 Hz**.
- LPF: RC = **0.33 ms**, so f0 = **482.3 Hz**.
- Butterworth needs K = 3 − √2 = 1.586, i.e. 10k/0.586 = **17.07 kΩ** where the board has 18k, the nearest
  standard value. So Q is slightly under 1/√2: very slightly overdamped.

**2.2** At s = jω0, s² + ω0² = −ω0² + ω0² = 0, leaving (ω0/Q)·jω0. So H = K(−ω0²)/(jω0²/Q) = jKQ and
|H| = **K·Q = 1.0769**.

**2.3** b = 2 − 1/0.6923² = **−0.0864**; u = (0.0864 + √(0.0075 + 4))/2 = **1.0441**; Ω = **1.0218**.
- The HPF alone is −3 dB at 22.58 × 1.0218 = **23.07 Hz**, and the LPF at 482.3/1.0218 = **471.98 Hz**.
- The two f0 are only 4.4 octaves apart, so neither stage is quite at full gain mid-band. The cascade peaks
  at × 2.405 instead of 2.42, and measured from that lower peak its corners move out to **23.0 / 473.9 Hz**
  (`run.py filter`; real: `board.json` → `cascade_minus3db_hz_analog`, `cascade_passband_gain`).

**2.4**
- ω0 = **3030 rad/s** becomes ω0' = 4096·tan(π·482.3/2048) = **3739 rad/s**, **+23 %**. The HPF's
  22.6 Hz is far below fs/2, where tan(x) ≈ x, so it barely moves.
- A = **16.78** million, B = **22.12** million, C = **13.98** million.
- a1 = (2·13.98 − 2·16.78)/52.88 = **−0.1059**; a2 = (16.78 − 22.12 + 13.98)/52.88 = **0.1633**.
- b0 = 1.5556·13.98/52.88 = **0.4112** (b = [0.4112, 0.8224, 0.4112]).
- At 2048 Hz the digital corners are 23.0 / 476.4 Hz, within **0.384 dB** of the circuit below 400 Hz
  (`board.json` → `max_digital_vs_analog_error_db_below_400hz`).

**2.5**
- LSB = 3.1/4096 = **0.757 mV**.
- Noise SD = (3.1/1024)/√12 = **0.874 mV**. With rounding on top, the error SD is 0.90 mV
  (`run.py filter`: 0.903).
- 1.65 V is half the 3.3 V supply: the EMG swings both ways and needs room on both sides of the ADC's
  range.

**2.6** 700 Hz lands at |700 − 1000| = **300 Hz**. The real decoder scored 87.3 % at 2048 Hz, and 86.9 % at
1 kHz both with and without a sharp anti-alias FIR (`sample_rate.json`). If folding were the problem, the
filter would have fixed it. The 0.4 points are the information above 500 Hz, which 1 kHz simply can't hold.

---

## Module 3

**3.1**
- Buffered (Z = 0): a = 47/48 × (1 + 47) = **47** and b = **47**, so G_diff = 47 and G_cm = 0.
- 20 kΩ ± 25 %: a = 47/70.5 × (1 + 47/18.5) = **2.360**, b = 47/18.5 = **2.541**, so G_diff = **2.450**,
  G_cm = **−0.180**, CMRR = 13.6 = **22.7 dB**.
- A common-mode voltage cancels only if both inputs see the same gain. Unequal electrodes make a ≠ b, so a
  piece of the common mode becomes signal.

**3.2** 1/13.6 + 10^(−3.1) = 0.0736 + 0.0008 = **0.0743**, i.e. **22.6 dB** (`board.json`: 22.6).

**3.3** e = [max(0, 0.1), max(0.05, −0.1), max(0.025, 0.5), max(0.25, −0.3)] = **[0.1, 0.05, 0.5, 0.25]**.
- τ = **180 ms**, decay per sample = **0.99729**, bandwidth **0.88 Hz**.
- The diode only lets the output rise above 0 V, and the capacitor only bleeds back toward 0, so the output
  is never negative: zero crossings are always 0 (`run.py boards`: min 0.0000 V). Four of the 16 features
  are dead.

**3.4**
- v1: total gain **877.6**, so the EMG must be **455.8 µV** to reach the diode, above almost all surface
  EMG. (Synthetic: 0.28 % of samples clear it; real: 0.22 %.)
- v2: 5k/884M = **5.65e−6** (105.0 dB). Adding the INA's 1e−5 gives 1.57e−5 (**96.1 dB**), and ÷10 for the
  right-leg drive gives **116.1 dB**. So 1 V of hum → **1.57 µV**, against 30-300 µV of EMG (`board.json`
  → `v2/effective_cmrr_db_incl_rld`).

**3.5** 10 mV × 10^(−22.6/20) = **0.74 mV**, × 877.5 = **0.65 V**.
- Synthetic run: 0.28 % → 19.37 % of samples clear the diode; real: 0.22 % → 20.3 % (`lda.json` →
  `.../info/above_diode_fraction`).
- But each channel's imbalance, and so the size and sign of its hum, is redrawn every day. What gets
  through looks different from one day to the next, so cross-day accuracy falls.
  - Real (`lda.json`, `sweeps/v1_envelope@0.01@20000.0`): 30.8 → **20.7 %**.
  - Synthetic: 29.0 → 14.3 %.
- At 100 mV the calibration turns every pot to 0, and v1 is at chance: real 9.3 %, synthetic 8.8 %.

---

## Module 4

**4.1** −½(x − μ)ᵀΣ⁻¹(x − μ) = −½xᵀΣ⁻¹x + xᵀΣ⁻¹μ − ½μᵀΣ⁻¹μ. The first term doesn't depend on k, because Σ
is shared, so it can't change which class wins. What remains is linear in x, so every boundary
δ_A = δ_B is a straight line (a plane in 16-D).

**4.2** μ_A = **1**, μ_B = **5**. Each point is 1 from its mean: σ² = (1 + 1 + 1 + 1)/(4 − 2) = **2**.
- δ_A = x/2 − 1/4 and δ_B = 5x/2 − 25/4. Equal when 2x = 6: **x = 3**, the midpoint, because the priors and
  the variance are shared. Unequal priors would push it toward the rarer class.
- 2D: w = Σ⁻¹(μ_B − μ_A) = (2/2, 1/1) = **(1, 1)**. The boundary is at w·x = ½ μ_BᵀΣ⁻¹μ_B = ½(4/2 + 1) =
  **1.5**: x1 + x2 = 1.5.
- A constant column makes a row and column of Σ zero, so Σ has no inverse. Shrinkage adds a little of the
  identity, so it does. (Background: Ledoit-Wolf picks how much from the data; `ledoit_wolf` in
  `m4_lda.py` is that formula, about 0.002 here.)

**4.3** Plain accuracy **0.9**, balanced **0.5**, chance **9.1 %**.

**4.4**
- **6** neighbours. 0.25⁶ = **0.000244**, so **99.98 %** of test windows have a training neighbour.
  `run.py lda` measures 99.94 %: edge windows have fewer neighbours.
- By hand: **[False, True, True, True]**. Window 0 ends at 400, where window 4 starts: they touch but share
  no sample.
- On the real v2 data the shuffled split scores 91.3 % against 87.3 % cross-day (`lda.json`). Synthetic:
  80.6 % against 71.1 %.

**4.5** Real EMG is a random interference pattern of thousands of motor units, so two sessions can't match
sample for sample. Which number of a pair is real can't be known, so both go. The copies double-weighted two
test days and made one point of the days-elapsed curve a self-test (DEBUG_LOG #2). Six dropped: **115 days**
kept (`data_audit.json` → `days_kept`).

**4.6** On identical windows (the last 40 %, test days): within-day **80.8 %** vs cross-day **77.5 %**
(`lda.json` → `main/v2@0.0/within_day/test_days`, `.../cross_day/late_40pct`). Calibrating on the day wins,
as it should: it knows today's electrode placement.
- Synthetic: 90.3 % vs 71.3 %, the same direction.
- But the synthetic early and late windows score alike (70.4 vs 71.3 %), where the real ones don't (92.4 vs
  77.5). The README explains why.

---

## Module 5

**5.1**
- E[d²] = (1 + 1 + 4)/3 = **2**, E[d³] = (−1 − 1 + 8)/3 = **2**, so γ = 2/2^1.5 = **0.7071** and RMS = √3 =
  **1.732**.
- −x: γ = **−0.7071**, RMS = **1.732**. 2x: γ = **0.7071**, RMS = **3.464**.
- A reversed lead flips the skewness and leaves the RMS; a gain change leaves the skewness and scales the
  RMS.

**5.2** β = (0.2 + 0.02 + 0.3)/(0.25 + 0.01 + 0.25) = 0.52/0.51 = **1.0196**; reversed, **−1.0196**.
- The average can sit near zero by chance: day 51's channel 1 averaged −0.074 and looked like a one-day
  reversal. The profile test asks whether all 11 classes flipped together at the same size, which is what a
  reversal predicts. Real: across 460 day-channels, the median |β| is 0.99 (`data_audit.json` → `polarity`).

**5.3**
- Every Hudgins feature is an even function of x (1.4).
- f([0, 3]) = (0 + 2)/2 = **1**; f([0, −3]) = **0**. A threshold after a filter sees the sign.
- A CNN trained only on familiar lead patterns has never seen what a reversed lead does to its detectors.
  Synthetic (`run.py polarity`): the raw-waveform model scores **83.0 %** on days with a familiar pattern
  and **15.0 %** on unseen ones. LDA scores 69.6 / 72.5 %.

**5.4** **16** patterns.
- With random flips in training, the synthetic raw model scores 62.5 / 62.4 %: no cliff, but it pays about
  20 points on the familiar days.
- Its features are fixed, so the only way an LDA can become sign-blind is to average each kernel's 16 sign
  variants, which throws information away. More kernels leave more to average:
  - 4 kernels: 43.7 %
  - 8 kernels: 54.0 %
  - 16 kernels: 62.8 %
- The real CNN *learns* its detectors, so it found sign-blind ones that kept the information: 89.0 / 86.0 %,
  2.5 points lost on familiar days (decoder README, E3).

**5.5** The amplitude stepped a little on every channel at each change. That looks like electrodes taken off
and re-applied with some leads swapped, not one channel moving onto another muscle. It's an inference: the
dataset doesn't document it.

---

## Module 6

**6.1**
- 0.1 × 187 = **18.7 °/s**, 63 % of it after **20 ms**.
- Steady 15 °/s: 15/187 + 0.05 = **0.1302**. Ramping at 10 °/s: (10 + 0.02·150)/187 + 0.05 = **0.1195**.
- ω_ref/K pays for the back-EMF, τ·ω̇/K accelerates the inertia, and d_f breaks friction.

**6.2** C·P = Kp(τs + 1)/(τs) × K/(τs + 1) = Kp K/(τs). The closed loop L/(1 + L) = 1/(1 + τs/(Kp K)), which
is 1/(λs + 1) when τ/(Kp K) = λ.
- Kp = 0.02/(187 × 0.05) = **0.002139 duty per °/s** and Ki = **0.1070 duty per °** (`metrics.json`:
  0.00214 and 0.1070).
- At λ = 30 ms with the datasheet model: 0.00357 and 0.178.
- A fast loop leans on the model. With datasheet guesses the fast tier overshoots up to 38 % across 36
  perturbed motors; identified first, the worst is 20 % (firmware README).

**6.3** e = **2**; feed-forward = 10/187 + 0.05 = **0.103476**; P = 0.002139 × 2 = **0.004278**;
duty = **0.107754**; integral = 0.107 × 2 × 0.001 = **0.000214**.

**6.4**
- Part 1: while the output is pinned at 0.6 it can't grow, so the integral would keep charging with no
  effect. When the joint frees up, all that stored duty comes out as a surge. Toy run: with anti-windup the
  integral reaches 0.40 and the peak after release is 52 °/s; without it, 0.60 and 70 °/s.
- Part 2: during a ramp the lag is predictable, and the feed-forward's τ·ω̇ term is meant to cover it. An
  integral that charged on it would overshoot once the ramp ends. Real: 9 % overshoot without part 2, none
  with it (`make latency`, part 3). The toy arm has no stiction, so it can't show this.
- Sag before the hold: (0.135 − 0.027)/0.107 = **1.0°**. The toy settles at 38.99° from 40°.

**6.5**
- 25/0.06698 = **373 counts/s**, **3.73 counts per tick**.
- √(0.73 × 0.27) = **0.44 counts**, so 0.44/3.73 = **11.9 %**. At 3 °/s: √(0.448 × 0.552)/0.448 = **111 %**.
- Each edge's placement error repeats every 4 edges, so the time across 4 edges is always exactly one
  cycle.
- `run.py loop` reproduces the firmware test (`sim/test_core.cpp`, line 70): 110.9 / 33.5 / 11.9 / 2.2 %
  for count difference, ≤ 0.015 % for edge timing.

**6.6** 0.130 − 0.104 = **0.026 < 0.027**: the arm doesn't move.
- Toy: feed-forward alone has a 100 % error. Real firmware simulation: 98.3 % (`make test`; `metrics.json`
  → `lift_15dps_steady_error_ff_only_dps` 14.733 °/s).
- The integral builds the missing duty. As the arm rises the gravity load changes, so the integral is always
  catching up a little. Toy: 0.137 °/s, 0.91 %. Real: 0.133 °/s in `metrics.json`, which the firmware test
  prints as 0.8 %.

---

## Module 7

**7.1** 2000^(1/63) = **1.128**: each level is **12.8 %** above the last. Equal *ratios*, because the same
fractional error matters at 5 counts and at 500.

**7.2**
- Predict: q = 0.8p + 0.1·(left + right) = [0.16 + 0.08, 0.48 + 0.04, 0.16 + 0.08] = **[0.24, 0.52, 0.24]**.
- Likelihoods **[0.0591, 0.1216, 0.1233]**.
- Posterior **[0.1326, 0.5909, 0.2765]**.
- Estimate e^(0.5909 ln 2 + 0.2765 ln 4) = **2.21**. One sample of size 2 shifted the belief toward 4,
  because a Laplacian with s = 4 explains a 2 better than s = 1 does.

**7.3**
- β lets the probability jump to a far level within a few samples once the evidence is overwhelming: the
  onset. α makes the steady belief move slowly, so it averages many samples: the hold.
- T = **39.8 ms**, 50 % at **66.8 ms**. The firmware test measures 66 (`sim/test_core.cpp`, line 93) and
  the synthetic step test 62. The Bayesian envelope takes 1 ms in both.

**7.4**
- **[off, off, off, on, off]**: the count reaches 3 > 2 on the 4th sample, then drops below release.
- **[on, on, on, on, on, on, off]**: 3.5 is above release, 2.9 isn't.
- A slow filter averages a 3 ms pop away; a jump-capable estimator believes it.
  - Synthetic: 56 false starts a minute without the confirm, 1 with 20 ms.
  - Real (`latency_study.json`): 62 and 0.

**7.5** ≈ **43 ms**. Estimator alone: 103 → 97 ms. The rest came from the faster control tier (a 500 °/s²
ramp and λ = 30 ms), which is allowed only after `j` measures the motor (`firmware/results/latency_sweep.txt`).

---

## Module 8

**8.1** **10**: MOT_OK, encoder errors, lead-off, artefact, over-travel, overspeed, stall, runaway,
tracking, continuous motion. That matches the résumé.
- The `Safety.*` tests exercise **4 of the 10**: stall, runaway, MOT_OK and lead-off, plus the kill switch,
  which disarms rather than faults.
- If an interviewer asks, say "ten monitors, four of them covered by closed-loop tests".

**8.2**
- The **20th**. A 19-tick blip resets, then it trips on the 20th tick of the new run.
- Hold times keep noise from tripping a fault. A stall needs 400 ms, because an arm starting under load is
  briefly slow. A runaway needs only 60 ms, because positive feedback doubles fast and it is the dangerous
  one.

**8.3**
- It would restart into the same obstruction, stall again, and pulse against the person's arm forever.
- Arming re-checks everything (`armBlocker`): calibrated, zeroed, kill switch closed, MOT_OK, muscles
  relaxed, joint inside the limits.

**8.4**
- No: a hung loop never reaches its own check (`run.py safety`: the in-loop check never fires; the watchdog
  fires at 1200 ms).
- **200 ms.**
- The board's pull-downs on the driver pins (R2, R3, R6, R5) turn the driver off while the chip resets.

**8.5** Obstruction trips **stall**; the reversed encoder trips **runaway**.
- Toy: stall 470 ms after the joint is blocked; runaway after 205 ms and 1.8° of travel.
- Firmware simulation: stall 443 ms after contact; runaway after 211 ms and 2.3° (firmware README).

---

## Module 9

**9.1** `{tau / (K * lambda), 1.0 / (K * lambda)}`, `std::copysign`, `std::clamp`, `std::fabs`. `inline`
lets the same function be defined in every file that includes the header without the linker complaining.

**9.2** The parity test compares every tick. Floating-point rounding depends on the order of operations,
so the same order gives the same bits.

**9.3**
- Python: **0**. The test prints max |Δ duty| 0 over 4500 ticks: bit-identical.
- Firmware (float): about **6e−7** of duty, about float's 7 significant digits on numbers near 0.1-0.6.
  The test allows 1e−4.

**9.4** Two runs of the benchmark on the computer this was built on (yours will differ):
- your step about 3 ns (2.7-2.8), the firmware's float PI about 3 ns, the firmware's envelope on 4 channels
  2.7-3.8 µs, under 0.4 % of the tick;
- the envelope costs about 1000 times more than the PI: 64 exp() per channel per sample.

On the ESP32-S3 (240 MHz) everything is slower. Double precision runs in software there, because its FPU is
single precision, which is why the firmware uses float. The firmware's own comment guesses about 50 µs per
channel for the envelope, untimed on the chip: about 20 % of the tick for all four.

---

## Module 10

**10a** The biggest disagreement is Module 5's augmentation cost. The toy's model can't learn its
detectors (README, "What's simpler").

**10b** A model retell, about 90 seconds:

> "The orthosis lets someone with a weak arm drive their elbow with their own muscle signals: flex to raise
> it, extend to lower it, both to stop. I led the electronics team. The first board's front end lost almost
> everything: 1 kΩ inputs loaded by the electrodes cut its gain from 47 to 2.45, and a diode threw away
> anything under 0.4 V. I redesigned it around an INA333 with a right-leg drive, keeping the 23-474 Hz
> band-pass and dropping the rectifier, and to test it before fabrication I emulated both boards on 115 days
> of real four-channel forearm EMG.
> An LDA decoder on Hudgins features held 87 % across days through v2, within a point of no board at all,
> against 31 % through v1. On the firmware side, EMG sets a joint speed and a 1 kHz PI loop with
> plant-inversion feed-forward tracks it. It's IMC-tuned from a first-order motor model, with two anti-windup
> rules, and holds the speed error under 1 % lifting a simulated forearm. A Bayesian amplitude estimator with
> a 20 ms onset confirm, plus a faster control tier once the motor is identified, took onset-to-motion from
> 103 to 43 ms on 24 real contractions in simulation. Ten fault monitors latch the motor off, and a watchdog
> outside the loop resets the chip. The limit: all of it is simulation and emulation on one subject's data,
> and v2 hasn't been built yet."

---

## Module 11

**11.1**
- **12.90 N·m**. The damper is **7.68 N·m per rad/s**: the back-EMF, the 1/τ part of the first-order model.
- Armature **0.129 kg·m²**: the rotor turns N = 192 times faster than the joint, so its inertia counts N²
  ≈ 37,000 times. It outweighs the 0.085 kg·m² forearm.

**11.2** **Radians** (rad/s). `joint_speed_dps` converts with `np.degrees`.

**11.3**
- Lifting: feed-forward only 98.8 %, closed loop 0.9 %. The firmware simulation: 98.3 % and 0.8 %.
- Holding, closed loop: the elbow is still creeping (0.26° over 5 s) where the firmware shows 0.000°.
  `frictionloss` acts like a very stiff damper near zero speed, not a lock, so the joint slides slowly
  while the integral catches up; it settles after about 5 s. Stiction (1.3× the Coulomb level in
  `plant.h`) stops the firmware's plant dead.
