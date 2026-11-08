# Rebuild the orthosis work: one week, about 6.5 hours

You rebuild the core of your orthosis entry yourself, deriving each piece on paper with tiny numbers before
you code it:
- the decoder: features, the board's filter and ADC, why v1 failed and v2 passes, LDA and honest
  evaluation, electrode polarity;
- the firmware: the feed-forward PI speed loop, the Bayesian envelope and its latency, the fault monitors
  and watchdog;
- a C++17 port of the speed loop with GoogleTest;
- optionally, the same loop on a MuJoCo elbow.

Synthetic data (`simdata.py`) stands in for the 2 GB MultiDay dataset and the firmware's simulator, so the
Python runs on Windows on ARM with numpy alone. **Its numbers are not your project's.** Those are in
[`REAL_NUMBERS.md`](REAL_NUMBERS.md), each with its file and key.

## The week

| Day | Modules | Time | You write | Done when |
|---|---|---|---|---|
| 1 | Setup, **1** EMG and Hudgins features, **2** the front end: filter, ADC, aliasing | 70 min | ~32 lines | `python check.py 1`, `2` pass; `python run.py filter` |
| 2 | **3** why v1 failed and v2 passes; then set up WSL for C++ (15 min) | 50 min | ~14 lines | `check.py 3`; `run.py boards`; `rebuild_tests_solution` runs |
| 3 | **4** LDA and honest evaluation | 45 min | ~27 lines | `check.py 4`; `run.py lda` (~15 s) |
| 4 | **6** the motor speed loop | 45 min | ~18 lines | `check.py 6`; `run.py loop` |
| 5 | **5** electrode polarity, **8** safety: latching faults and the watchdog | 60 min | ~20 lines | `check.py 5`, `8`; `run.py polarity`, `run.py safety` |
| 6 | **7** the Bayesian envelope and latency; **10a** the report and your real numbers | 60 min | ~23 lines | `check.py 7`; `run.py report`; `REAL_NUMBERS.md` read aloud |
| 7 | **9** the speed loop in C++17; **10b** the retell and a mock interview | 65 min | ~14 lines C++ | `./build/rebuild_tests` passes; a recorded 90 s retell |
| any, after day 4 | **11** (optional) your loop on a MuJoCo elbow | +30 min | 3 lines | `check.py 11` in WSL |

Core: 6 h 35 min. Module 5 comes after 6 only because Module 8's last check drives your Module 6 loop.

**Short on time?** Skip, in this order:
1. Module 11 (optional).
2. Module 10's mock interview: do it later with Claude.
3. Worksheet 2.5-2.6 (ADC, aliasing) and 3.5 (hum as dither): read the answers instead.
4. Module 5's code: do the paper, then run `python run.py polarity --solutions`.
5. Module 9's benchmark: build and run the tests, skip the timing questions.

Don't skip 4, 6, 7, 8 or 10: they are your résumé bullets.

**Each module:**
1. **Paper first:** that module's questions in `WORKSHEET.md`. Tiny numbers, no calculator heroics.
2. **Code:** fill in the `TODO`s. Each docstring gives the equation, but derive it first.
3. **Check:** `python check.py N`. A failure names the entry that's wrong and what it expected.
4. **Say it:** the module's one-sentence interview line, out loud, without reading.
5. **Compare:** `ANSWERS.md`.
6. **Connect it (5 min):** "in my project this was X, the number was Y, the file is Z". The worksheet's
   "Connect it" line and the map below have the files. Open one and find your function in it.

**Stuck rule:** after 15 minutes stuck on one thing, read *that one function* in `solutions/`, close it, and
type it from memory. If a day runs 15 minutes over, stop and pick it up tomorrow.

## Setup

**Python (Modules 1-8, 10), Windows.** Install Python 3.10+ from python.org (the ARM64 installer is fine),
then in this folder (`orthosis-emg-decoder/rebuild`):
```
py -m pip install numpy matplotlib      (matplotlib is optional: only for run.py report --plot)
py check.py all --solutions             60 passed, 1 not written yet (the MuJoCo one, until Module 11)
py check.py all                         0 passed, 61 not written yet
```
If pip starts compiling numpy, install the regular x64 Python instead; Windows runs it under emulation.
(`py` and `python` are the same thing; the rest of these docs say `python`.)

**C++ and MuJoCo (Modules 9 and 11), in WSL.** In PowerShell run `wsl --install`, restart, open "Ubuntu", then:
```
sudo apt update && sudo apt install -y build-essential cmake libgtest-dev python3-venv
python3 -m venv ~/venv && ~/venv/bin/pip install numpy mujoco
cd /mnt/c/Users/<you>/<path to>/Troy_Mayer/orthosis-emg-decoder/rebuild
cd cpp && cmake -S . -B build && cmake --build build && ./build/rebuild_tests_solution && cd ..
~/venv/bin/python check.py 11 --solutions
```
What was tested, and where:
- **Tested** on Ubuntu x86-64: cmake 3.28, GoogleTest 1.14 from apt, MuJoCo 3.17 in a venv. The solution
  passes all 8 GoogleTests and the benchmark runs.
- **Not tested:** WSL on an ARM laptop.
- **Not tested:** CMake's GoogleTest download, used when apt's isn't found (Visual Studio without WSL).
  The download was blocked where this was built.
- For MuJoCo without WSL, `pip install mujoco` in an x64 Windows Python is the fallback (untested).

## What's in the folder

| File | What | Edit? |
|---|---|---|
| `WORKSHEET.md`, `ANSWERS.md` | the paper derivations, then the worked answers | on paper |
| `REAL_NUMBERS.md` | your project's real numbers with sources, the careful wordings, your debugging stories | read aloud |
| `m1_features.py` … `m8_safety.py`, `m11_mujoco.py` | one module each, with `TODO`s | **yes** |
| `cpp/velocity_pi.hpp` | Module 9 (tests, benchmark, fixture and CMake provided in `cpp/`) | **yes** |
| `check.py` | the checker: `python check.py N` (1-8, 11) or `all`, `--solutions` | no |
| `run.py` | the experiments: `features`, `filter`, `boards`, `lda`, `polarity`, `loop`, `latency`, `safety`, `mujoco`, `report`; `--plot` | no |
| `simdata.py` | the synthetic EMG; its docstring says what it keeps and drops | read the docstring |
| `solutions/`, `cpp/solutions/` | reference solutions, for the stuck rule | no |
| `solutions/worksheet_numbers.py` | recomputes every hand-calculation number in `ANSWERS.md` | no |
| `make_skeletons.py`, `cpp/make_fixture.py` | rebuild the TODO files from `solutions/`, and the C++ parity fixture (maintainers) | no |

## How it maps to your real project

Decoder paths are relative to `orthosis-emg-decoder/`, firmware paths to `firmware/`.

| You write | Real code |
|---|---|
| `windows`, `mav`, `wl`, `zc`, `ssc` (+ provided `hudgins`, `dead_band`) | `emgdec/features.py`: `windows` 10, `hudgins` 17, `dead_band` 36 |
| `sallen_key_params`, `analog_response`, `bilinear_biquad` | `emgdec/board.py`: lines 27-30, `sallen_key_analog` 40, `sallen_key` 48 |
| `hpf_corner_ratio`; provided `corners` | README "The filter band, derived"; `scripts/verify_board.py` (`minus_3db`) |
| `adc`, `alias_frequency` | `board.py`: `adc` 56, `sample_at` 64; README E4; `tests/test_board.py` (700 Hz → 300 Hz) |
| `diff_amp_gains`, `v1_cm_to_diff`, `input_to_clear_diode` | `board.py`: `diff_amp_gains` 97, `BoardV1.cm_to_diff` 144; `verify_board.py` |
| `peak_detector` | `board.py`: `_peak_detector` 81 |
| `v2_cm_to_diff`, `hum_at_rectifier` (+ provided couplings) | `board.py`: `BoardV2.cm_to_diff` 207, `cm_coupling` 149 / 212, `hum` 256; README E1 |
| `lda_fit`, `lda_predict`, `balanced_accuracy` | `emgdec/models.py`: `lda` 11 (scikit-learn LDA, `shrinkage="auto"`), `lda_score` 17 |
| `shares_a_sample`, `cross_day` (+ provided `random_split`, `within_day`) | `emgdec/protocols.py`: `random_split` 34, `within_day` 42, `cross_day` 58 |
| `identical_days` | `scripts/audit_data.py`, lines 67-73 |
| `skewness`, `rms`, `beta`, `call` | `scripts/audit_data.py`: skewness 78, beta 84, calls 85 |
| `flip_augment` | `emgdec/models.py`: `train_cnn(flip_aug=True)` 45; README E3 |
| `imc_gains`, `feedforward`, `VelocityPI.step` | `emg_orthosis/orthosis_core.h`: `Config::kp()/ki()` 151-152, `VelocityPI` 423, `step` 432 |
| `count_speed`, `edge_speed` | `orthosis_core.h`: `EdgeTracker` 183, `VelocityEstimator` 207; `sim/test_core.cpp` 70 |
| provided `ToyArm`, `Ramp` | `sim/plant.h` (the full plant); `orthosis_core.h`: `SetpointShaper` 386 |
| `bayes_step`, `two_pole`, `activate` | `orthosis_core.h`: `BayesEnvelope` 261 (`step` 274), `EmgChannel::updateActivation` 359; `analysis/latency_study.py`: `env_bayes` 48, `env_lp2` 33, `to_command` 88 |
| `held_for`, `Supervisor.step`, `clear` | `orthosis_core.h`: `timer` 806, `supervise` 669, `superviseMotion` 685, `fault` 793 |
| `Watchdog.expired` | `emg_orthosis.ino`: the task watchdog, lines 335-340, fed at 219 |
| `cpp/velocity_pi.hpp` | `orthosis_core.h` lines 151-152 and 432; tests like `sim/test_core.cpp`, `sim/test_closed_loop.cpp` |
| `motor_torque` (Module 11) | `sim/plant.h`, `Plant::step` |

## What's simpler here than the real project

**From the projects' own caveats** (decoder README "Assumptions" and "Limitations"; firmware README "Status"):
- **One subject.** Every decoder accuracy is one person's recordings, not a population estimate.
- **An emulation, not a board.** v2 isn't fabricated. The decoder's v2 numbers come from emulating its
  analog chain on recorded EMG.
- **The firmware has only run in simulation.** It has not run on the board.
- **The source EMG already went through a lab amplifier** (NeXus). The emulation can make a signal worse but
  never cleaner, and nothing above 1024 Hz exists to alias.
- **`cnn.json` predates the 0.16 % INA333 gain correction.** Torch wasn't available for the re-run.
- **Class 0 ("no motion") is the loudest class** on every day of MultiDay. Labels are used as given. The
  synthetic data doesn't copy this.

**Where the toy differs, and what it can't reproduce:**
- **The synthetic EMG is random MUAP trains from 6 random "muscles"**, not anatomy, over 20 days.
  - Cross-day LDA is 71 % here against the real 87 %.
  - Days differ more here: SD 11 points, real 5.
- **Within-day beats the shuffled split here (90 vs 81 %); in the real data the order is reversed (81 vs 91 %).**
  The synthetic electrodes move more between days, so pooling 10 days into one model costs more.
- **Late windows aren't harder here.** The real data's last 40 % of a hold scores 77.5 % against 92.4 %
  for the first 60 %. The synthetic holds fade by the real data's own ratios (RMS × 0.93, mean frequency
  × 0.94), yet both parts score about 71 %. Whatever makes the real late windows hard is more than that
  fatigue.
- **v2 at 1 V of hum** loses 4.6 points in the real emulation and nothing here. The synthetic EMG is
  louder: its 5th percentile is about 64 µV RMS, against the real data's 30 µV floor.
- **The raw-waveform model is fixed random kernels with an LDA on top, not a CNN.**
  - Without augmentation it reproduces the real cliff: 83 % on days with a lead pattern seen in training,
    15 % on unseen ones.
  - Polarity augmentation removes the cliff, but costs about 20 points on the familiar days, where the
    real CNN lost 2.5. A linear model can only become sign-blind by averaging its features, and the
    cost shrinks as you give it more kernels (4 → 43 %, 8 → 54 %, 16 → 62 %).
- **The toy arm (Module 6) is one first-order equation.** Its constants are from `firmware/sim/plant.h`.
  It has no stiction, no cable and no encoder quantisation.
  - It can't show the ramp anti-windup's effect on overshoot (the real 9 % → none), so the checks test
    that logic directly.
  - Its lift error, 0.91 %, is close to the firmware simulation's 0.133 °/s (0.9 %), and for the same
    reason: a load that grows as the arm rises.
- **MuJoCo's `frictionloss` is a soft constraint, not stiction.** Held at zero command, the elbow keeps
  creeping for about 5 s (0.26° over 5 s) where the firmware's plant stops dead (0.000°).
- **Module 7's contractions are synthetic MUAP trains** (4 → 200 counts RMS, a 50 ms ramp), not the
  firmware's 24 real ones.
  - The Bayesian envelope still halves the time to half command: synthetic 113.5 → 59.5 ms, real 97 → 48.
  - The synthetic false starts are 56 → 1 a minute where the real ones are 62 → 0.
  - The 103 → 43 ms closed-loop number isn't rebuilt here: it needs the full plant, so it stays in
    `firmware/` (`make latency`).
- **Module 9 ports the speed loop only, in double precision.** The firmware is float, because the
  ESP32-S3's FPU is single-precision.

**Never quote rebuild numbers as project results.**
