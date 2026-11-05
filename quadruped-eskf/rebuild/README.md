# Rebuild the filter: one week, under 5 hours

You rebuild the core of the quadruped estimator yourself, in Python. Each step is derived on paper
first and then coded. Nothing here needs MuJoCo or a GPU, so it runs on Windows on ARM. A synthetic
trotting robot (`simdata.py`) stands in for the simulator.

By the end you'll have written the predict step, the leg update, the Jacobians, the
finite-difference tests, and the core of the walking controller (gait clock, Raibert foot placement,
levelling loops) behind every line on your résumé. You'll have watched the IMU drift,
the legs stop it, and a turn separate tilt from accelerometer bias.

## The week

| Day | Module | Time | Paper (WORKSHEET.md) | Code | Done when |
|---|---|---|---|---|---|
| 1 | Setup + **1: Kalman filter in 1D** | 30 min | 1.1-1.4: derive the gain | `m1_kalman_1d.py` (6 lines) | `python check.py 1` passes |
| 2 | **2: position + velocity, with matrices** | 35 min | 2.1-2.4: one predict and one update by hand | `m2_kalman_2d.py` (7 lines) | `python check.py 2` passes |
| 3 | **3: rotations, gravity leak, finite differences** | 35 min | 3.1-3.4 | `m3_rotations_fd.py` (10 lines) | `python check.py 3` passes |
| 4 | **4: the ESKF predict step** | 55 min | 4.1-4.4 | `eskf.py`, 4 functions (~30 lines) | `python check.py 4` passes, then `python run.py imu-only` |
| 5 | **5: the leg update** | 60 min | 5.1-5.5 | `eskf.py`, 3 functions (~35 lines) | `python check.py 5` passes, then `python run.py full` |
| 6 | **6: the walking controller** (gait clock, Raibert, levelling loops) | 45 min | 6.1-6.3 | `m6_walking_control.py` (6 lines) | `python check.py 6` passes |
| 7 | **7: experiments, your numbers, retell** | 35 min | 7.1-7.4 | nothing new | `python run.py report` (1-3 min), then the 90-second retell, recorded once |

Total: about 4 h 55 min. Each day follows the same order:

1. **Paper first.** Do that module's worksheet questions. Tiny numbers, no calculator heroics.
2. **Code.** Fill in the `TODO`s. Each docstring gives the equation, but derive it before you look.
3. **Check.** Run `python check.py N`. A failure tells you which entry is wrong and what it expected.
4. **Say it.** Each module ends with one interview sentence. Say it out loud without reading.
5. **Compare.** Check your answers against `ANSWERS.md`.

**Stuck rule:** if you're stuck on one thing for 15 minutes, open `solutions/<file>.py` and read *that one
function*. Close it, then type it yourself from memory. If a day runs 15 minutes over, stop and
pick up there tomorrow. The 5-hour cap matters more than finishing perfectly.

## Setup (Windows on ARM)

1. Install Python 3.10 or newer from python.org. The Windows ARM64 installer is fine.
2. In a terminal, in this folder (`quadruped-eskf/rebuild`), run:
   ```
   py -m pip install numpy
   py -m pip install matplotlib      (optional: only for the plot on day 7)
   ```
   Recent NumPy versions ship Windows-on-ARM wheels. If pip starts compiling something or fails,
   install the regular **x64** Python instead; Windows runs it under emulation and everything here works.
3. Check the setup:
   ```
   py check.py all --solutions       should say: 28 passed
   py check.py all                   should say: 0 passed, 0 failed, 28 not written yet
   ```
   (On Windows, `py` and `python` are the same thing; the rest of these docs say `python`.)

## What's in the folder

| File | What it is | You edit it? |
|---|---|---|
| `WORKSHEET.md` | the paper derivations, module by module | yes, on paper |
| `ANSWERS.md` | worked answers to the worksheet | no; read it after each module |
| `m1_kalman_1d.py`, `m2_kalman_2d.py`, `m3_rotations_fd.py` | warm-up modules with `TODO`s | **yes** |
| `eskf.py` | the filter: 7 functions to write, the rest provided | **yes** |
| `m6_walking_control.py` | the controller module: 4 functions to write | **yes** |
| `check.py` | the auto-checker (`python check.py 1` ... `6`, or `all`) | no |
| `run.py` | runs your filter on the walk (`imu-only`, `full`, `report`) | no |
| `simdata.py` | the synthetic trotting robot | no; read its docstring |
| `so3.py` | rotation helpers (skew, Exp, quaternions) | no |
| `solutions/` | reference solutions. Only for the stuck rule. | no |

## How it maps to your real project

Every function you write here has a twin in the real project. On day 7, open them side by side.

| You write (rebuild/eskf.py) | Real project (python/qeskf/eskf.py) | C++ (cpp/src/eskf.cpp) |
|---|---|---|
| `propagate_nominal` | line 100 | `propagateNominal`, line 24 |
| `error_transition` (F) | line 110 | `coreTransition`, line 34 |
| `process_noise` (Q) | line 128 | inside `predict`, line 67 |
| `predict` | line 141 | line 56 |
| `measurement` (r, H) | line 199 | line 134 |
| `update` (K, Joseph) | line 219 | line 154 |
| `inject` (+ reset) | line 248 | line 184 |
| provided: `touchdown` / `liftoff` | `augment` 155 / `marginalize` 177 | lines 83 / 104 |
| provided: `static_alignment`, `initial_covariance` | lines 283 / 305 | |
| `check.py` Jacobian checks | `python/tests/test_eskf.py` | `cpp/tests/test_jacobians.cpp` |

| You write (rebuild/m6_walking_control.py) | Real project (python/qeskf/gait.py) | C++ (cpp/src/gait.cpp) |
|---|---|---|
| `leg_phase`, `scheduled_stance` | lines 51, 55 | `legPhase` 59, `scheduledStance` 66 |
| `raibert_offset` | line 96, inside `foot_targets` | line 112 |
| `level_command` (attitude PD) | line 75: a leg-length PD (the QP version is `qp_balance.py` line 115) | line 87 |
| the height PI (worksheet 6.3d) | lines 69-70 | lines 81-82 |

**What's simpler here than in the real project.** You'll need these if you're asked to compare.

- The leg sensor gives the foot vector directly. The real filter computes it with forward kinematics
  on 14-bit encoders and propagates encoder noise through the leg Jacobian.
- Contact is perfect: it comes straight from the gait schedule. The real filter combines a 5 N force
  threshold with the schedule, and ends contact 5 ms early.
- A lifted foot keeps a fixed slot with its covariance cleared. The real filter resizes P.
- Module 6's Raibert demo uses an inverted pendulum, where foot placement is the only speed
  control. Your real trot also sweeps the stance feet at the commanded speed, so don't carry the
  pendulum's gains or numbers over to the robot.
- The noise is white and the floor is rigid, so the numbers here are better than the project's:
  about 0.004 m/s velocity RMSE here, against 0.013 m/s there. Don't quote the synthetic numbers as
  project results.

## After the week (optional)

- Port `predict` to C++/Eigen and test it against your Python, the way the project's parity test does.
- Mis-tune the filter: set `leg_noise` in `Params` (eskf.py) to 10× the simulator's 3 mm and see what
  happens to the RMSE and to the filter's own σ. That's the idea behind your project's NEES check.
- Use `python run.py report --seed 3`. Seed 3 has a large accelerometer bias, so the lean problem is bigger.
