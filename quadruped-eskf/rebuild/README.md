# Rebuild the project: one week, about 6.5 hours

You rebuild the core of the quadruped project yourself, deriving each piece on paper before coding it:
- the filter (predict, leg update, Jacobian tests);
- the legs (forward kinematics, encoder noise);
- contact detection, including your own DEBUG_LOG #4 bug;
- the walking controller;
- a C++/Eigen port with GoogleTest;
- consistency and observability checks;
- optionally, a small MuJoCo simulation.

A synthetic trotting robot (`simdata.py`) stands in for the simulator, so the Python runs on Windows on
ARM. Its numbers are not your project's: those are in [`REAL_NUMBERS.md`](REAL_NUMBERS.md).

## The week

| Day | Modules | Time | You write | Done when |
|---|---|---|---|---|
| 1 | **1** Kalman filter in 1D, **2** position + velocity with matrices | 50 min | 13 lines | `python check.py 1`, `2` pass |
| 2 | **3** rotations, gravity leak, finite differences; then set up C++/MuJoCo | 50 min | 10 lines | `check.py 3` passes; `rebuild_tests_solution` runs |
| 3 | **4** the predict step | 50 min | ~30 lines | `check.py 4`; `python run.py imu-only` |
| 4 | **5** the leg update | 55 min | ~35 lines | `check.py 5`; `python run.py full` |
| 5 | **6** legs: forward kinematics, encoders; **7** contacts and your bug #4 | 60 min | ~30 lines | `check.py 6`, `7`; `run.py legs`, `run.py contacts` |
| 6 | **8** walking controller; **9** predict in C++ with GoogleTest | 80 min | ~45 lines | `check.py 8`; `./build/rebuild_tests` passes |
| 7 | **10** NEES, observability, your real numbers, retell; **11** MuJoCo (optional, +30 min) | 50 min | ~15 lines | `run.py report`; a recorded 90 s retell |

**Short on time?** Skip Module 11, then Module 6. Don't skip 4, 5, 8, 9 or 10: those are your résumé.

**Each module:**
1. Paper first: that module's questions in `WORKSHEET.md`.
2. Fill in the `TODO`s.
3. Run `python check.py N`.
4. Say the module's "Say it" line out loud.
5. Check `ANSWERS.md`.
6. Spend 5 minutes connecting it to the real project: "in my project this was X, the number was Y, the
   file is Z". The tables below have the files.

**Stuck rule:** after 15 minutes stuck on one thing, read that one function in `solutions/`, close it,
and type it from memory.

## Setup

**Python (Modules 1-8 and 10).** Install Python 3.10+ from python.org (the ARM64 installer is fine),
then in this folder run:
```
py -m pip install numpy matplotlib      (matplotlib is optional: the report's plot)
py check.py all --solutions             42 passed, 1 not written yet (the MuJoCo one, until Module 11)
py check.py all                         0 passed, 43 not written yet
```
If pip tries to compile numpy, install the x64 Python instead; Windows runs it under emulation.
(`py` and `python` are the same thing; the docs say `python`.)

**C++ and MuJoCo (Modules 9 and 11), in WSL.** Both are easiest in Ubuntu under WSL. In PowerShell run
`wsl --install`, restart, open "Ubuntu", then:
```
sudo apt update && sudo apt install -y build-essential cmake libeigen3-dev libgtest-dev python3-venv
python3 -m venv ~/venv && ~/venv/bin/pip install numpy mujoco
cd /mnt/c/Users/<you>/<path to>/Troy_Mayer/quadruped-eskf/rebuild
cd cpp && cmake -S . -B build && cmake --build build && ./build/rebuild_tests_solution && cd ..
~/venv/bin/python check.py 11 --solutions
```
These commands were tested on Ubuntu x86-64, not on an ARM laptop.
- Without WSL: Visual Studio's C++ tools and CMake also work. CMake then downloads Eigen and GoogleTest
  itself. The Eigen download was tested; the GoogleTest one couldn't be from here.
- For MuJoCo, `pip install mujoco` in an x64 Python on Windows is the fallback.

## What's in the folder

| File | What | Edit? |
|---|---|---|
| `WORKSHEET.md`, `ANSWERS.md` | paper derivations, then worked answers | on paper |
| `REAL_NUMBERS.md` | your project's real numbers, the honest wording, your two bug stories | read aloud |
| `m1_kalman_1d.py` … `m3_rotations_fd.py` | warm-ups | **yes** |
| `eskf.py` | the filter: 7 functions to write; contact bookkeeping provided | **yes** |
| `m6_legs.py`, `m7_contacts.py`, `m8_walking_control.py`, `m10_consistency.py`, `m11_mujoco.py` | one module each | **yes** |
| `cpp/predict.hpp`, `cpp/fd.hpp` | Module 9 (tests, benchmark and fixture provided in `cpp/`) | **yes** |
| `check.py`, `run.py` | the checker (`check.py N` or `all`) and the experiments (`imu-only`, `full`, `legs`, `contacts`, `report`) | no |
| `simdata.py`, `legs.py`, `so3.py` | the synthetic robot, the Go1 leg geometry and IK, rotation helpers | read the docstrings |
| `solutions/`, `cpp/solutions/` | reference solutions, for the stuck rule | no |

## How it maps to your real project

Paths are relative to `quadruped-eskf/`. Python is `python/qeskf/`, C++ is `cpp/src/`.

| You write | Real Python | Real C++ |
|---|---|---|
| `propagate_nominal`, `error_transition` (F), `process_noise` (Q), `predict` | `eskf.py` lines 100, 110, 128, 141 | `eskf.cpp` 24, 34, 67, 56 |
| `measurement` (r, H), `update`, `inject` | `eskf.py` 199, 219, 248 | `eskf.cpp` 134, 154, 184 |
| provided `touchdown` / `liftoff` | `augment` 155 / `marginalize` 177 | `eskf.cpp` 83 / 104 |
| Module 3 and 9 finite-difference tests | `python/tests/test_eskf.py` | `cpp/tests/test_jacobians.cpp` |
| Module 9 parity test | `make reproduce` → `metrics.json` (1.4e-13 m) | `cpp/tests/test_parity.cpp` |
| `foot_position`, `foot_jacobian`, `leg_noise_cov` | `kinematics.py`; `kin_cov`, `eskf.py` 149 | `kinematics.cpp`; `kinCov`, `eskf.cpp` 77 |
| `detect_contacts` | `pipeline.py` (`estimator_contacts`); `closed_loop.py` 147-148 | |
| `nees`, `observability_matrix` | `pipeline.py` (`nees`); `observability.py` | |
| `leg_phase`, `scheduled_stance`, `raibert_offset`, `level_command` | `gait.py` 51, 55, 96, 75 (`qp_balance.py` 115) | `gait.cpp` 59, 66, 112, 87 |
| Module 11 IMU check and soft foot | `python/tests/test_sim_imu_consistency.py`; `models/go1/` | |

## What's simpler here than the real project

- White noise and a rigid world, so the numbers are better than the project's (about 0.004 m/s vs
  0.013 m/s, and NEES 2.3 vs 13). **Never quote rebuild numbers as project results.**
- The filter keeps a fixed 3 mm leg noise. The real one recomputes the encoder part each update.
- Foot force is a weight-sharing profile, not a contact solver. The toy reproduces your bug #4 and the
  velocity side of the latency experiment, but not its 6.6° of yaw.
- A lifted foot keeps a fixed slot in P. The real filter resizes P.
- Module 8's Raibert demo is an inverted pendulum, where foot placement is the only speed control. Your
  real trot also sweeps the stance feet at the commanded speed, so don't carry its numbers over.
- Module 9 ports predict only (15 states). The real C++ runs the whole step with feet.
