# Your real numbers (the project, not the rebuild)

Say these out loud. The rebuild's synthetic numbers (71 % cross-day, 0.91 % lift error and so on) are **not**
these. Decoder sources are relative to `orthosis-emg-decoder/`, firmware sources to `firmware/`. Every number
was re-read from its file. `make test` (firmware: 26 tests, all passing) and `make claims` (decoder) check
them. One gap in `make claims`: its résumé scan finds no text, because its regex looks for "emulation" and
your résumé says "emulated". It then reports the résumé as clean. The résumé's 31 % and 87 % are still
checked as claims (`resume_v1`, `resume_v2`); only the scan for unbacked numbers misses it
(`DECODER_ANSWERS.md`, D9.3).

## The résumé, bullet by bullet

### 1. *"Designed a 1 kHz feed-forward PI velocity controller with anti-windup in C++17 on an ESP32-S3, tuned from a first-order motor model to hold simulated speed error to 0.8%, guarded by 10 latching fault monitors and a watchdog."*

| Number | What it is | Say it like this | Source |
|---|---|---|---|
| **1 kHz** | a FreeRTOS task pinned to core 1, `xTaskDelayUntil`, one tick per EMG sample | "a 1 kHz control task, separate from serial" | `emg_orthosis/emg_orthosis.ino` lines 19, 170, 352; `orthosis_core.h` `fs_hz` (94) |
| **C++17** | one header with no hardware dependencies, compiled into the firmware and into a simulator | "the same file runs on the chip and in GoogleTest" | `CMakeLists.txt`; README intro |
| **first-order model** | K = 187 °/s per duty (datasheet), τ = 20 ms (J·R/Kt·Ke); `j` measures them: K 192 (true 193), τ 28 ms (true 22) | "IMC from K over tau s plus one" | `orthosis_core.h` `Config` (64-66); README "Control design", "Simulation results" |
| **Kp 0.00214, Ki 0.1070** | IMC gains at λ = 50 ms (duty per °/s, duty per °) | | `results/metrics.json` → `kp_duty_per_dps`, `ki_duty_per_deg` |
| **0.8 %** | steady speed error lifting the forearm at +15 °/s in the simulated arm (lowering: 0.3 %). Feed-forward alone: 98.3 % (it stalls). | "under 1 % in simulation, lifting; feed-forward alone stalls" | `sim/test_closed_loop.cpp` line 45 (`make test` prints 0.8 %); `metrics.json` → `lift_15dps_steady_error_closed_dps` = 0.133 °/s |
| **10 monitors** | MOT_OK, encoder errors, lead-off, artefact, over-travel, overspeed, stall, runaway, tracking, continuous motion; each latches FAULT and coasts the motor until a clear and a fresh arm | "ten runtime monitors, each with a hold time, all latching" | `orthosis_core.h` `supervise` (669), `superviseMotion` (685), `fault` (793) |
| **watchdog** | the ESP-IDF task watchdog, 200 ms, resets the chip; pull-downs then hold the driver off | "outside the loop it watches" | `emg_orthosis.ino` lines 335-340 |

**Careful wording.**
- **"On an ESP32-S3".** The firmware is *built* for the ESP32-S3 in CI, but it has never *run* on one: the
  v2 board isn't fabricated (firmware README, "Status"). Safer: *"…in C++17 for an ESP32-S3…"*. If asked,
  say: "it compiles for the chip in CI; every number is from simulation".
- **"0.8 %".** Two measurements disagree slightly. The test averages its own steady window and prints 0.8 %;
  `metrics.json` averages the last 0.5 s and gets 0.133 °/s, which is 0.89 %. "Under 1 %" is safe for both.
  It's one operating point (lifting at 15 °/s), in simulation.
- **"10 latching fault monitors".** The count is exact. Only **4** have closed-loop tests: stall, runaway,
  MOT_OK and lead-off (`sim/test_closed_loop.cpp`, `Safety.*`). The watchdog can't be tested in
  simulation. If asked "are they all tested?": "four of the ten, plus the kill switch".

### 2. *"Cut EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG estimator, validated in closed-loop simulation through the controller and a simulated arm on 24 real forearm contractions."*

| Number | What it is | Source |
|---|---|---|
| **103 → 43 ms** | median time from contraction start to the joint moving, on 24 real MultiDay contractions through the v2 board model, the full controller and the simulated arm. p90: 170 → 82 ms. | `results/latency_sweep.txt` part 1; README "Latency"; test `Latency.RealEmgOnsetToMotion` |
| **half speed 139 → 67 ms, stop 89 → 32 ms, jitter 2.43 → 0.99 °/s** | the same 24 trials | same |
| **"before"** | two-pole 4 Hz envelope, 150 °/s² ramp, λ 50 ms | same |
| **"now"** | Bayesian envelope + 20 ms onset confirm + the fast tier (500 °/s², λ 30 ms), which unlocks only after `j` | same |
| **43 ms budget** | ~16.5 ms for the muscle to reach a detectable level + 20 ms confirm + ~7 ms motor spin-up | README, "Where the 43 ms goes" |
| **102-109 / 43-45 ms** | how far the two medians move with small plant changes (gearbox 5.18, pulley +0.5 %, inertia +5 %, 12.1 V) | `latency_sweep.txt` part 5 |
| **97 → 48 ms** | open-loop envelope study, 200 real contractions: time to half *command*, two-pole 4 Hz vs Bayesian | `results/latency_study.json` (`lp2 4 Hz`, `bayes laplace a=0.0001 b=1e-12 MEAN confirm 20`) |
| **62 → 0 false starts/min** | one electrode pop per second on 60 s of rest: no confirm vs 20 ms confirm | `latency_study.json`; README "Bayesian envelope" |

**Careful wording.** The bullet credits the latency to the estimator alone, and the results don't support
that.
- At the old tuning, in the closed loop, the estimator by itself moved the start from 103 to **97 ms**.
- Half-speed and stop got slightly *worse*: 139 → 150 and 89 → 100 ms.
- Its clear win there was jitter, 2.43 → 0.96 °/s (`latency_sweep.txt`, line 4).
- The 43 ms needs the faster control tier on top. The website already says so.

Safer: *"Cut simulated EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG estimator and a
model-identified fast control tier, validated in closed-loop simulation on 24 real forearm contractions."*
If you keep the bullet, say it out loud the safe way.

### 3. *"Led a 5-engineer team redesigning the 4-channel EMG board in KiCad with an INA333 instrumentation front end, right-leg drive and ISO7763 motor isolation, raising day-to-day gesture decoding from 31% to 87% in emulated tests."*

| Number | What it is | Say it like this | Source |
|---|---|---|---|
| **31 %** | cross-day LDA through v1 with C9/R15 fitted (the envelope build), 11 classes, mean of 57 test days (30.8 %, SD 4.0). The rectified build: 22.3 %. | "31 % through the old board" | `results/lda.json` → `main/v1_envelope@0.0/cross_day/per_day/mean`; `claims.json` `resume_v1` |
| **87 %** | the same through v2: 87.3 % (SD 5.3) | "87 % through the redesign, within a point of no board" | `lda.json` → `main/v2@0.0/cross_day/per_day/mean`; `claims.json` `resume_v2` |
| **day-to-day** | train on days ≤ 60, test each day > 60, never recalibrated; 115 usable days (58 / 57) | | `results/data_audit.json` → `days_kept` |

**Careful wording.**
- **What raised 31 to 87.** Two things did: the INA333's high-impedance inputs (no more electrode
  loading), and dropping v1's rectifier. The right-leg drive and the ISO7763 don't contribute to that
  87 %, because it is measured with no hum.
  - Impedance sweep (E2): v1 with buffered inputs reaches 59.5 % (envelope) or 71.4 % (rectified).
  - The right-leg drive only matters with hum: v2 is 87.2 % at 100 mV and 82.8 % at 1 V.
  - The ISO7763 isn't in the emulation at all; it protects the person from the motor side.
  - If asked "what made the difference?", give those two reasons, and say what the right-leg drive and
    isolator are for.
- **"In KiCad".** The repo holds the v2 schematic as a PDF. The KiCad sources aren't committed (decoder
  README, "Next steps" 6), and v2 is schematic-complete, not laid out or fabricated.
- **"Emulated tests"** is the right phrase: real EMG run through a model of each board's analog chain and
  ADC. Keep it.
- **One subject**, and the source EMG already passed a lab amplifier, so v2 is bounded above by the
  no-board result.

## The portfolio page, claim by claim

| Claim (index.html) | Real value | Source |
|---|---|---|
| 87.3 % through v2 against 87.9 % with no board | 87.34 / 87.94 % | `lda.json` → `main/v2@0.0/...`, `main/reference@0.0/cross_day/per_day/mean` |
| both v1 builds below 45 % on every day | best single day 41.8 % (envelope), 28.0 % (rectified) | `lda.json` → `main/v1_envelope@0.0/cross_day/series`, `v1_raw@0.0` |
| CNN brought back to 86.3 % by polarity augmentation | 86.3 %, mean of 3 seeds; plain CNN 68.2 %, gain augmentation 60.6 % | `results/cnn.json` → `v2+flip/test_mean_over_seeds` |
| 115 days, 11 classes | 115 kept of 121 (3 byte-identical pairs dropped) | `data_audit.json` → `days_kept`, `identical_day_groups` |
| 23-474 Hz band-pass | analog −3 dB corners 23.0 / 473.9 Hz, peak × 2.405 | `results/board.json` → `cascade_minus3db_hz_analog` |
| 1.65 V reference | a design value: half the 3.3 V rail | `emgdec/board.py` `VREF`; `claims.json` `design_values` |

**Site wording to check.**
- **The channel figure's caption ends "…11-111× trim gain, and an anti-alias filter into the ADC."** The
  decoder README says R33/C24 (4.8 kHz) is *not* an anti-alias filter at 1 kHz: it isolates the op-amp from
  the ADC's sampling capacitor, and the 482 Hz low-pass is the only real anti-aliasing. Safer: *"…and an RC
  that isolates the op-amp from the ADC."*
- **"A hardware watchdog"** is fine (the firmware README calls it "the ESP-IDF hardware task watchdog"). If
  asked, say what it is: the chip's task watchdog, a timer the firmware sets up and feeds every tick, which
  resets the chip after 200 ms without a feed.
- **The 23-474 Hz band-pass was carried over from v1** (the same Sallen-Key parts on both boards). Don't
  present it as part of the redesign.

## The numbers behind the follow-ups

| Topic | Number | Source |
|---|---|---|
| Shuffled split (leaky) | 91.3 % vs 87.3 % cross-day: overstates v2 by 4 points | `lda.json` → `main/v2@0.0/random_split` |
| Same windows (DEBUG_LOG #9) | within-day 80.8 % vs cross-day 77.5 %, last 40 % of each test hold; cross-day 92.4 % on the first 60 % | `lda.json` → `.../within_day/test_days`, `.../cross_day/late_40pct`, `early_60pct` |
| Days since training | 89.9 % after 1 day → 81.0 % after 81-120 days (5-day training blocks) | `lda.json` → `main/v2@0.0/day_gap` |
| Confusions (v2) | key grip → no motion 18.9 %, supination → pronation 16.0 % | decoder README, "Results" |
| Hum (E1) | v2: 100 mV costs 0.1 points, 1 V costs 4.6. v1: 10 mV lifts 0.22 % → 20.3 % of samples over the diode, but accuracy falls 30.8 → 20.7 %. | `lda.json` → `main/*@0.1`, `sweeps/*@0.01@20000.0`, `.../info` |
| Impedance (E2) | v2 flat at 87.3 % from 0 to 50 kΩ; v1 59.5 % buffered → 30.8 % at 20 kΩ → 11.7 % at 50 kΩ | decoder README, E2 |
| CMRR | v1 22.6 dB (gain 2.45 with 20 kΩ ± 25 %); v2 116.1 dB with the right-leg drive (5 kΩ imbalance) | `board.json` |
| 1 kHz sampling (E4) | 87.3 → 86.9 %, the same with a sharp anti-alias filter: the loss is the band above 500 Hz, not folding | `results/sample_rate.json` |
| Recalibration (E5) | 2 s per class of same-day data alone: 77.5 %, matching 58 old days; pooling them doesn't help | decoder README, E5 |
| Polarity | 51 of 57 test days (every day from 69) have a lead pattern absent from training; median \|β\| 0.99 | `data_audit.json` → `polarity` |
| CNN (E3) | validated 92.5 %, tested 68.2 %. LDA on the CNN's exact windows: 86.9 %. Seen vs unseen pattern: CNN 91.5 / 65.5 %, + polarity augmentation 89.0 / 86.0 %. | `cnn.json`; `results/lda_control.json`; DEBUG_LOG #3 |
| Speed measurement | counts per 10 ms tick: 12-111 % error at arm speeds; edge timing: under 0.02 % | `sim/test_core.cpp` line 70; firmware README table |
| Robustness | 36 perturbed motors: conservative tier worst overshoot 16 %; fast tier 20 % once identified, 38 % if forced on with datasheet numbers | `latency_sweep.txt` part 2 |
| Holding | 0.000° drift over 5 s at zero command; a 4 N·m push sags it 0.74° (conservative) / 0.51° (fast) | `metrics.json` → `hold_drift_5s_deg`; `latency_sweep.txt` part 4 |
| Faults in simulation | stall 443 ms after contact; reversed encoder caught after 211 ms and 2.3° | firmware README, "Simulation results" |
| Soft limit | full speed into 30° stops at 29.47° (braking curve v ≤ √(2·a·d)) | same |
| Transmission | 191.95:1 overall, 0.06698° per count, 28 counts per motor turn | `metrics.json` → `deg_per_count`; `sim/plant.h` |
| Tests | 26 GoogleTests: 11 unit, 14 closed-loop, 1 real-EMG latency | `sim/test_core.cpp`, `test_closed_loop.cpp`, `test_latency.cpp` |

## Your debugging stories (five beats each)

The orthosis work has one debug log, the decoder's `DEBUG_LOG.md`. The firmware's stories are in its
README's "What changed" table.

**1. The CNN that validated at 92.5 % and tested at 68.2 %** (decoder DEBUG_LOG #3-#4). Your best story.
- **Symptom:** trained on days 1-50, the CNN picked its epoch at 92.5 % on days 51-60, then scored 68.2 %
  on days 61-121. LDA held 87.3 % on the same board output.
- **Hypotheses and tests:**
  - "It gets less data": LDA trained on exactly its windows scored 86.9 %. Falsified.
  - "Gain drift", the textbook cause: per-day accuracy showed a cliff, not a slope, and gain augmentation
    made it *worse* (60.6 %). Falsified.
- **Cause:** lead reversals. Per-channel skewness flips sign while RMS barely moves, and the cliff lands on
  day 69, the first day with a lead pattern training never saw. LDA's features are even functions, so it
  was immune; the CNN's first layer is linear, so it wasn't.
- **Fix:** random per-channel polarity flips in training: 86.3 %, level with LDA. The cost was 2.5 points
  on familiar days (91.5 → 89.0).
- **Lesson:** a validation set from the same sessions can't show a failure that only appears in new
  sessions.

**2. A dead board scoring 93-95 % within-day** (decoder DEBUG_LOG #1).
- **Symptom:** v1 scored 93-95 % within-day while almost no signal cleared its rectifier and cross-day sat
  near 30 %.
- **Cause:** the hum level was drawn once per *recording*, and each recording is one class. Within-day
  trains and tests on two halves of the same recording, so the hum level identified the class.
- **Test:** putting the per-recording draw back takes v1's within-day at 10 mV from 46.7 % to 92.3 %, while
  cross-day barely moves.
- **Fix:** draw the level per day, the phase per recording.
- **Rule:** draw every nuisance parameter at the level where it really varies, never at one that coincides
  with the label.

**3. Same-day calibration "losing" to a 60-day-old model** (decoder DEBUG_LOG #9).
- **Symptom:** within-day 81.3 % vs cross-day 87.3 %.
- **Hypothesis:** the two columns score different windows.
- **Test:** the cross-day model scores 92.4 % on the first 60 % of each hold and 77.5 % on the last 40 %,
  which is the only part within-day tests. The late windows have lower RMS (× 0.92) and lower mean frequency
  (× 0.94): fatigue.
- **Fix:** compare on identical windows: 80.8 % vs 77.5 %. Calibrating on the day does win.

**4. The envelope filter that was twice as slow as its comment** (firmware README, "What changed").
- **Symptom:** the first firmware's envelope (two poles at 3 Hz) was commented "~55 ms".
- **Test:** two equal one-pole filters have a mean delay of 2/(2π·fc). At 3 Hz that's 106 ms. The firmware
  test checks the formula at 4 Hz: 80 ms mean delay, 66 ms to half a step.
- **Cause:** the comment was wrong, and a slower filter is the only way a plain low-pass gets smooth.
- **Fix:** the Bayesian estimator, with a 20 ms confirm against pops. Then `make latency` showed the
  closed-loop gain needed the fast control tier too, which is why the site credits both.

**5. Speed from counts was 12-111 % wrong** (firmware README, "Speed measurement").
- **Symptom:** the first firmware differenced encoder counts every 10 ms.
- **Test:** at 25 °/s the motor makes 0.37 counts per ms, so a 10 ms window sees 3 or 4 counts: 11.9 %
  error. At 3 °/s it's 111 % (`sim/test_core.cpp`, line 70).
- **Fix:** time the edges instead, over one full quadrature cycle so uneven edge spacing cancels: under
  0.02 %.

## Likely questions, with model answers

- **"Walk me through the controller."** "EMG sets a joint speed: flexor minus extensor, with a dead zone,
  and both together means stop. A setpoint shaper limits acceleration and brakes into the soft limits.
  Feed-forward inverts a first-order motor model to supply most of the duty, and an IMC-tuned PI fixes what
  the model misses: mostly the forearm's weight. It runs at 1 kHz in a FreeRTOS task, and ten monitors can
  latch it off."
- **"If the feed-forward gives the duty, what does the PI do?"** "The model doesn't know the limb is there.
  Lifting at 15 °/s, feed-forward alone stalls: 98 % error. The integral supplies the gravity load, which
  takes the error under 1 %, and at zero command it's a position hold: 0.000° drift over 5 s in
  simulation."
- **"How did you tune it?"** "IMC: cancel the motor's pole with the integral time, Ti = τ, and put the closed
  loop at λ = 50 ms. That's Kp = τ/(Kλ) = 0.00214 and Ki = 1/(Kλ) = 0.107. A faster 30 ms tier unlocks only
  after the firmware has identified the real motor, because a fast loop leans on the model."
- **"What did the Bayesian estimator buy you?"** "It reacts in a sample or two at an onset but averages hard
  during a hold. On its own it halved the time to half command in the envelope study, 97 to 48 ms, and it
  cut jitter. In the closed loop, most of the 103 to 43 ms came from pairing it with a faster control tier.
  The 20 ms confirm costs 20 of those 43 ms, and it's what stops electrode pops moving the arm."
- **"How do you know it works?"** "26 GoogleTests: unit tests, closed-loop tests against a simulated motor,
  transmission and forearm, and real recorded EMG for latency. But it hasn't run on hardware: the v2 board
  isn't built, so every number is simulation."
- **"What's the worst thing that could go wrong?"** "Driving the elbow past its range, running the wrong way,
  or not stopping. Soft limits brake into a stop, and over-travel faults. A reversed encoder is caught by the
  runaway monitor in about 200 ms. Every fault latches until someone clears and re-arms. A task watchdog
  resets the chip if the loop hangs, and pull-downs keep the driver off."
- **"Why did v1 fail?"** "Its 1 kΩ inputs were loaded by 20 kΩ electrodes, so the gain fell from 47 to 2.45
  and the CMRR to 23 dB. Then a 0.4 V diode passed only 0.22 % of samples. v2's INA333 has 100 GΩ inputs and
  keeps the whole waveform: 31 % to 87 % in emulation."
- **"Why LDA and not a deep net?"** "LDA is 11 × 16 multiply-adds per decision and immune to lead reversals.
  The CNN matched it only with polarity augmentation, and its invariance is learned only for the case it was
  shown."
- **"What would you do next?"** "Record on the v2 board and check the emulation against it. Weighted
  recalibration, since pooling a few fresh seconds with old days doesn't help. A sign-invariant first layer
  for the CNN. And time the envelope on the ESP32-S3: the firmware only estimates it."

## Before the interview
Read this page out loud once. Then do one mock interview with Claude on the questions above, and record it.
