# Your real numbers (the project, not the rebuild)

Say these out loud. The rebuild's synthetic numbers (0.004 m/s RMSE and so on) are **not** these.
Each source is relative to `quadruped-eskf/`, and every number was re-read from it.

## The résumé, number by number

Your current résumé PDF says: *"…estimating base velocity to 0.013 m/s RMSE against 1.38 m/s for IMU-only."*
and *"Verified every Jacobian against finite differences, then ran a Raibert trot controller and a QP balance
controller on the estimate instead of ground truth, walking 5 seeds with zero falls and velocity tracking within
0.004 m/s."*

| Number | What it is | Say it like this | Source |
|---|---|---|---|
| **0.013 m/s** | 3D world velocity RMSE vs ground truth, 60 s walk, **mean of 10 sensor-noise seeds** (0.0108-0.0162, sd 0.0019), in simulation | "0.013 m/s RMS error, averaged over 10 noise seeds of a one-minute simulated walk" | `results/metrics.json` → `monte_carlo` |
| **1.38 m/s** | IMU-only dead reckoning on the same data. Passes 0.1 m/s at 7.8 s, 1 m/s at 24 s. | "The IMU alone averages 1.38 m/s of error" | `metrics.json` → `imu_only`; README "Dead reckoning" |
| **1 kHz** | one update per IMU sample; C++ step 18 µs median, 60 µs p99, 0.14 % deadlines missed (host jitter) | "about 20 µs of the 1 ms budget" | `results/timing.json` |
| **1e-8** | finite-difference tolerance on every Jacobian (F, H, augmentation, leg); actual ~1e-10; 20 GoogleTests = 4 tests × 5 seeds | "nudge each input, compare the slope with my formula, to 1e-8" | `cpp/tests/test_jacobians.cpp` |
| **1.4e-13 m** | C++ vs Python on the full 60 s log (|Δv| 3.3e-13 m/s) | "same maths in both, down to rounding" | `metrics.json` → `parity_python_vs_cpp` |
| **0 falls, 5 seeds** | Trot on the estimate: 0/5 falls. **QP on the estimate: 1/5 fell (seed 3); 0/5 after a 4 s startup turn.** | "The trot walked all five; the QP needed a 4 s startup turn first" | `results/closed_loop.json`, `results/qp_compare.json` |
| **0.004 m/s** | a **difference**: trot tracking 0.173 m/s on the estimate vs 0.169 on ground truth. The 0.17 is the controller's own error. (QP: 0.087 vs 0.092, a 0.005 difference.) | "Tracking was 0.17 either way, so my estimate cost 0.004" | same files |

**Careful wording.** "Walking 5 seeds with zero falls" is true for the trot, and for the QP only after the
startup turn. "Within 0.004 m/s" is the trot's number; the QP's is 0.005. If they ask, say it the way the table
does. The website already words it precisely.

## The numbers behind the follow-ups

| Topic | Number | Source |
|---|---|---|
| Tuning | 4 parameters (foot drift 1 mm/√s, kinematic noise 3 mm, end contact 5 ms early, 10 ms settle), best of 150, tuned on a **separate** 120 s walk (seed 100) | `results/tuned_params.json` |
| Contact | force > 5 N AND schedule. Force only, 10 ms latency: 0.045 m/s and **6.6° yaw** over 60 s. With the schedule: 0.011 m/s and 0.1°. | `results/experiments.json` E4 |
| Slip | 2 cm/s of slip doubles the error (0.0214); 0.1 m/s of slip gives 0.0996 m/s | E2 |
| Q | datasheet Q: 0.0108 m/s. Tutorial Q = 0.01·I: 0.47 m/s and 103° tilt error. Datasheet ÷10: NEES median 88 (overconfident). | E5 |
| Honesty (NEES) | Even with datasheet Q, the velocity NEES median is **13** (3 is honest). The real filter is overconfident: foot sinking, slip and FK error aren't the white noise its Q and R assume. The rebuild's is 2.3 because its noise is white. | E5 |
| What drifts | exactly **4** unobservable directions (3 position + yaw) in all 30 windows. After 300 s and 51 m: position 0.80 m (1.6 %), yaw 2.8°. | `results/observability.json`; README "Over a long walk" |
| The lean | trot on the estimate leaned 4.1° (3.3-6.3) vs 1.15° on truth; tilt error 6.8° vs 2.3°. With a 4 s turn: 1.16° and 0.03°. | `closed_loop.json` |
| Closed-loop estimator | 0.025 m/s when the controller runs on the estimate; 0.009 m/s with the startup turn | `closed_loop.json` |
| Tests | 33 C++ test cases (20 Jacobian, 5 invariant, 1 parity, 4 gait, 3 loop), 17 Python tests | `cpp/tests/`, `python/tests/` |

## Your debugging stories (five beats each)

**#14, the lean.**
- **Symptom:** on the estimate, the trot leaned 3-6° on the first straight vs 1.15° on truth.
- **Hypothesis:** tilt and accelerometer bias are inseparable walking straight, and the controller levels the tilt it is told.
- **Tests:**
  - tilt error was 5.6-10° closed loop vs 1-4° open loop;
  - the lean vanished at the first turn;
  - a 4 s startup turn gave 1.16° and 0.03°.
- **Cause:** weak tilt observability, made worse by the loop.
- **Fix:** the startup turn.

**#4, the height drifting 0.6 m down in 60 s.**
- **Hypothesis:** the foot sinks after touchdown. That predicted **upward** drift, fixed by waiting after touchdown.
- **Test:** the drift was downward, and waiting made it *worse* (−0.61 → −1.05 m). Falsified.
- **Cause:** the foot rises ~4 mm at the end of stance as the other pair takes the load.
- **Confirmation:** ending contact earlier flipped the sign (−0.61 → −0.08 → +0.31 m).
- **Fix:** end contact a few ms before scheduled liftoff. That's the "5 ms early".
- (Module 7 rebuilds this bug. Say "10 stance events per second"; the log's "~14" is a typo.)

**#15, the QP walking at 0.115 m/s when told 0.4** (Module 12).
- **Symptom:** tracking error 0.23 m/s, worse than the trot's 0.169.
- **Hypothesis:** either the QP can't deliver the forward force, or something absorbs it.
- **Test:** it asked for +13.2 N forward, well inside friction, but the stance feet delivered −2.7 N.
- **Cause:** the legs' joint damping and friction, which the rigid-body model doesn't contain.
- **Fix:** integral action (ki_v = 4), not cancelling the damping (that cost 1.7-2.6° of roll): tracking
  0.23 → 0.091 m/s. The integrator hits its clamp, so it still runs about 0.1 m/s slow.

**QP numbers** (`README.md`, `make qp-compare`, 5 seeds × 60 s): tracking **0.091** m/s fed truth vs the trot's
0.169; **0.086** fed the estimate; standing forces **30.7-31.8 N** per foot, summing to m·g; largest side push
survived 8.9 N·s (trot 11.9). "In the style of MIT Cheetah 3's balance controller": say "simplified", since it has
no prediction horizon (not MPC).

## Before the interview
Read this page out loud once. Then do one mock interview with Claude on the real project and record it.
