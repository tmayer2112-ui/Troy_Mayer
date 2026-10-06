# Debug log

One entry per problem: **symptom**, **hypothesis**, **how it was tested**, **actual cause**, **fix**,
and a command that brings the failure back. Numbers are the ones printed at the time.
Entries 1–12 were recorded during the initial build on 2026-09-23, 13–14 while closing the loop on 2026-10-05.
Append new ones as they happen.

`repro N` below means `PYTHONPATH=python python3 python/scripts/debug_repro.py N` (entries 4 and 7 need `make data`).

---

### 1. Trunk "falls" at 0.14 m/s while its height stays constant
**Symptom:** Printing body-frame velocity once per second during the first walk showed `v_z ≈ −0.14 m/s`
at every sample (t = 5, 10, 15, … s), while trunk height stayed at 0.25 m.
**Hypothesis:** Frame bug. MuJoCo free-joint `qvel[0:3]` is body frame, not world (or the reverse),
so `Rᵀv` rotates the wrong vector.
**Test:** Finite-difference `qpos`: `(p[k+1] − p[k])/dt` against `v[k]` and `v[k+1]`.
**Actual cause:** Aliasing. The gait period is 0.40 s, so whole seconds always land on the same gait phase
(just after a diagonal pair touches down, trunk sagging). `qvel[0:3]` *is* world frame: it matches
`v[k+1]` to 3e-14 m/s, which also shows MuJoCo's semi-implicit Euler, `p[k+1] = p[k] + v[k+1]·dt`.
**Fix:** None in code. Diagnostics use whole-run statistics or non-commensurate sample times.
**Repro:** `repro 1` (after the gait change in #2 the aliased value is +0.19 m/s; the tell is that it's the same at every whole second).

### 2. Feet skid 0.7 m/s at touchdown
**Symptom:** Foot-sphere speed in the first ms of contact averaged 0.73 m/s horizontally and took
~20 ms to decay. The planted-foot assumption is false for that window.
**Hypothesis:** Low friction, feet slipping on the floor.
**Test:** Traced one foot through touchdown: height, velocity, contact force, scheduled phase.
**Actual cause:** The swing foot hit the ground at ~85 % of swing, mid-trajectory, still moving forward at
full swing speed. The trunk sagged ~2 cm under servo compliance (kp = 100 position servos), so the ground
was higher in the body frame than the swing profile assumed.
**Fix:** Ground-speed matching in the swing (the touchdown target slides backwards along the
extrapolated stance path so the foot already moves at −v in the body frame), a raised-cosine lift
(zero vertical velocity at touchdown) and trunk-height feedback. Horizontal touchdown speed: 0.73 → 0.16 m/s.
**Left over:** Vertical touchdown speed stays ~1.3 m/s (servo lags the swing target ~20 ms). Softer
swings traded it for more horizontal slip, so the default stayed.

### 3. The foot keeps moving for 20 ms after contact. It's the floor.
**Symptom:** After #2, contact force crossed 5 N with the foot-sphere centre at z = 21.9 mm, and it kept
descending to z = 3.1 mm over the next 20 ms.
**Hypothesis:** Still the touchdown skid from #2.
**Actual cause:** Menagerie's Go1 foot uses `solimp="0.015 1 0.023"`, a deliberately soft contact (a
stand-in for the rubber foot). The sphere sinks ~19 mm into the floor under load, so the foot is "in contact"
by force well before it is stationary.
**Fix:** Nothing in the simulator; tuning the world to suit the estimator would hide a real modelling error.
It drives #4.

### 4. Vertical position drifts 0.6 m *down* in 60 s on flat ground
**Symptom:** Contact-aided filter: velocity RMSE 0.024 m/s, but z error −0.61 m at 60 s and growing;
mean v_z error −8 mm/s.
**Hypothesis:** Touchdown-compression ratchet. The foot state is added at first contact, the foot then
sinks 19 mm (#3), the filter reads "foot fixed, FK vector got longer" as the trunk rising, and the next
touchdown inherits the error.
**Prediction:** drift *upward*, and waiting 10–30 ms after touchdown before adding the foot should fix it.
**Test result:** Drift is *downward*, and the settle delay made it *worse*
(−0.61 → −0.89 → −0.97 → −1.05 m for 0/10/20/30 ms). Hypothesis falsified.
**Actual cause:** Unloading at the end of stance. Over 860 stance windows the foot-sphere centre sinks
~2 mm early in stance and rises ~4 mm in the last 20 % as load moves to the other diagonal: net +1.4 mm
per stance from first contact (+3.1 mm if the first 20 ms are skipped, which is why the settle delay hurt).
~14 stance events/s × ~1.4 mm, shared by two stance legs, predicts ~7 mm/s of trunk drift; observed −8 mm/s.
**Confirming test:** Ending the stance window 0/10/20/30/40 ms before scheduled liftoff moves the final z
error −0.61 → −0.08 → +0.31 → +0.60 → +0.87 m. The sign flips, so this is a contact-model bias set by which
part of stance is used, not noise.
**Fix:** Liftoff anticipation became a tuned parameter, chosen on a separate tuning walk, never on the
headline run (README, "How the numbers were chosen"). It's causal on a robot because the gait planner
knows its own schedule.
**Repro:** `repro 4` (numbers above were taken before fixes #5/#6, so they differ; look for the sign flip).

### 5. Noise-free IMU dead reckoning is off by 0.1 m/s² after one step
**Symptom:** Sanity test: integrate MuJoCo's own ideal accelerometer/gyro from the true initial state.
After a single 1 ms step, |Δv| = 9.7e-5 m/s, i.e. ~0.1 m/s² acceleration mismatch.
**Hypothesis:** A missing Coriolis/transport term (`ω × v`) in the accelerometer model, because it had
the right magnitude (0.14 m/s² RMS).
**Test:** Add and subtract `ω × v`. Both made the y residual *worse* (0.028 → 0.145 m/s²). Ruled out.
**Actual cause:** MuJoCo's Euler integrator applies joint damping implicitly (`eulerdamp`, on by default).
Go1's joints have damping = 2, and through the mass matrix that changes the free joint's velocity update,
so `v[k+1] − v[k] ≠ qacc[k]·dt`. The accelerometer is computed from `qacc` and ground truth from the
integrated `qvel`, so they disagree.
**Fix:** `<flag eulerdamp="disable"/>` in `models/go1/scene.xml`. x/y residual 0.06 → 1e-14 m/s².
**Not a bug:** the gyro reports ω[k] but MuJoCo rotates with ω[k+1]; the error is α·dt² per step and
telescopes (4e-5 rad after 2 s).
**Repro:** `repro 5`. **Test:** `python/tests/test_sim_imu_consistency.py`.

### 6. A constant 3.35 mm/s² z residual after fixing #5
**Symptom:** x/y residuals 1e-14, z a flat 3.35e-3 m/s².
**Actual cause:** MuJoCo's default gravity is 9.81; the filter uses standard gravity 9.80665.
9.81 − 9.80665 = 0.00335. It would have shown up as an accelerometer z-bias no datasheet predicts.
**Fix:** `gravity="0 0 -9.80665"` in the scene. Residual 1.4e-14 on all axes.
**Repro:** `repro 6`.

### 7. Contact-aided roll is *worse* than IMU-only (0.54° vs 0.23° RMSE)
**Symptom:** After #5/#6 the contact-aided filter's roll RMSE was 0.54° against 0.23° for dead reckoning.
**Hypothesis:** Sign error in the attitude block of H or F.
**Test:** Wrote finite-difference tests for every Jacobian (transition, measurement, augmentation,
kinematics). All pass to 1e-8, so the math is right.
**Actual cause:** Weak observability. Traced over time: roll error 1.4–1.6° for t < 12 s, then 0.02° after
t ≈ 14 s, the first commanded turn. While the robot walks straight, tilt and horizontal accelerometer bias
are nearly indistinguishable (a tilt error δθ_y looks like a bias g·δθ_y), and the filter pushed b_a,y
0.26 m/s² off chasing small contact-model errors. Yaw rotation decorrelates them. The numerical
observability analysis later confirmed the weakest observable direction is exactly (δb_a,x, δθ_y) at ratio 1/g.
**Fix:** None; it's a property of the problem. Documented as a limitation, with the pitch RMSE split into
before/after the first turn (1.7° → 0.02–0.08°).
**Repro:** `repro 7`.

### 8. Parity test: `config key 'acc_rw' missing or wrong length`
**Symptom:** The first C++ vs Python parity run threw on the config file the Python side had written.
**Hypothesis:** The C++ key/value parser.
**Actual cause:** NumPy 2 changed `repr` of scalars: `f"{x!r}"` wrote `acc_rw np.float64(0.00016…)`, the
parser stopped at the `n` and the key had zero values.
**Fix:** `repr(float(x))`. **Repro:** `repro 8`.

### 9. "Builds clean" failed: two GCC 13 warnings in the config reader
**Symptom:** `-Wdangling-reference`, then `-Warray-bounds` inside `stl_algobase.h` memmove.
**Actual cause:** Both are GCC 13 false positives. The first comes from returning a reference out of a
`std::map` lookup keyed by a temporary string; the second from copying a one-element `std::vector<double>`.
**Fix:** Return by value, and a `scalar()` accessor for one-element keys. No `-Wno-…` flags; the build
is warning-free with `-Wall -Wextra -Wpedantic`.

### 10. SO(3) round-trip test fails: `log(exp(φ)) ≠ φ`
**Symptom:** `test_so3_roundtrip` failed for 1 of 100 random vectors.
**Hypothesis:** Bug in `log_so3` near θ = π.
**Actual cause:** The test drew |φ| = 3.55 > π. `log` returns the same rotation with angle 2π − |φ| about
the opposite axis, which is correct; the test was wrong.
**Fix:** Sample |φ| < π in the test (Python and C++). **Repro:** `repro 10`.

### 11. "The EKF gains 40 % spurious yaw information." It doesn't.
**Symptom:** Information along the analytic global-yaw direction, `NᵀP⁻¹N`, rose from 5.5e4 to 7.7e4 between
t = 10 s and 30 s. For a consistent estimator it can't rise, and this is the textbook standard-EKF failure
(linearising at changing estimates).
**Test:** Split each kinematic update into (a) the update at a fixed linearization point and (b) the move of
the linearization point caused by the correction. Also check `H·N` directly.
**Actual cause:** The diagnostic. N was normalised after mixing metres and radians, and its position part
grows as the robot walks away from the origin, so the direction being measured kept changing. With N left
unnormalised, the information falls steadily (1.0e6 → 6.3e5 → 3.7e5 → 2.1e5 at 0/10/30/60 s) and its
rises sum to 2 % of the starting value. Per update: |H·N| = 4e-16 (the measurement respects the symmetry
exactly); the update at a fixed linearization point changes it by −6e-9 (median, relative); the move of the
linearization point adds 1e-7. The spurious gain is real but small here.
**Fix:** Both measurements are now in `scripts/observability.py` (`yaw_information` in
`results/observability.json`). README states the size and names FEJ / OC-EKF / invariant EKF as the untaken fix.

### 12. 21 ms "compute" spike in the 1 kHz loop
**Symptom:** One benchmark run showed a 21 ms maximum filter-step time and 100 deadline misses (0.17 %).
**Hypothesis:** An allocation or a pathological update inside the filter.
**Test:** Compute p99.9 in the same run was 171 µs, and the misses line up with wake-up lateness, not
compute. Pinning to one core (`--cpu 3`) didn't help (0.23 %). Repeated runs miss 0.05–0.23 %.
**Actual cause:** The host. This is a shared cloud VM without an RT kernel; SCHED_FIFO + mlockall can't stop
the hypervisor. The filter itself uses ~2 % of the 1 ms budget at the median.
**Fix:** `timing.py` now runs three repetitions and reports the spread, not one lucky or unlucky run.

### 13. First closed-loop run falls before the robot takes a step
**Symptom:** With the controller fed the estimator's output, the robot fell at t = 2.9 s. Trunk pitch reached
−33° during the 2 s standing phase, before the filter had even started.
**Hypothesis:** Before the filter starts there is no attitude estimate, so the first version fed the
controller roll/pitch from the low-passed accelerometer. That should be fine for a robot standing still.
**Test:** Printed true vs fed-back pitch every 0.1 s. At t = 0.1 s the controller was told +16° while the true
pitch was −6.5°: the robot is still settling from its start pose, so the accelerometer measures
acceleration, not just gravity. The attitude loop corrected hard in the wrong direction, the stance legs hit
their reach limit, and the robot stayed pitched at −33°. The filter then calibrated on a robot that was
moving, and the walk started from garbage.
**Fix:** Until the filter starts, the controller holds a fixed stance (level attitude, zero velocity and
rates, height from leg kinematics). That is what a real robot does while it calibrates.

### 14. Closed loop: the robot leans 3–6° on a straight walk
**Symptom:** Fed the estimate, the robot walked the full 60 s without falling, but leaned 3.3–6.3° on the first
straight segment, against 1.15° when fed ground truth. Velocity tracking barely changed.
**Hypothesis:** The tilt/accelerometer-bias ambiguity from #7. The controller levels the estimated trunk, so
the estimate's tilt error becomes the real lean.
**Test:** (a) The estimator's tilt error over that segment is 5.6–10° closed loop, against 1.0–3.8° open loop on
the same physics. (b) The lean disappears at the first turn (t ≈ 12–14 s). (c) Prepending a 4 s left/right turn
before the same walk: lean 1.16°, tilt error 0.03°, the same as the ground-truth-fed controller.
**Actual cause:** Weak observability of tilt without turning, made worse by the loop: three of five noise seeds
give almost the same error (5.6–5.8°), so the closed-loop motion drives it, not sensor noise. The exact
coupling isn't isolated.
**Fix:** A 4 s turn at startup (`closed_loop.turn_first`). README, "Closing the loop".

### 15. QP controller walks at 0.11 m/s when told 0.4 m/s
**Symptom:** The first full run of the QP force-balance controller (seed 0, fed ground truth) walked the whole
60 s without falling, but velocity tracking RMSE was 0.23 m/s, worse than the scripted trot's 0.169. On the
first straight it settled at 0.115 m/s for a 0.4 m/s command.
**Hypothesis:** Either the QP can't deliver the forward force (friction, the moment balance), or it delivers it
and something absorbs it.
**Test:** (a) Over 7–12 s the QP's commanded forces summed to +13.2 N forward, close to what the velocity gain asks
for (14.5 N), and well inside the friction pyramid (max |f_x| + |f_y| = 0.26 f_z). (b) The real contact forces
from MuJoCo, split by scheduled stance and swing: the stance feet pushed the robot −2.7 N (backwards), and swing
feet scuffing the ground +2.7 N. So the force the QP commanded never reached the ground. (c) The Menagerie
model has 1–2 N·m·s/rad of joint damping and 0.2 N·m frictionloss on every joint, which the rigid-body model
ignores. Feeding that damping forward on the stance legs raised the speed to 0.21 m/s, and to 0.31 m/s with the
controller's own 0.5 N·m·s/rad stance damping also removed. But roll RMS rose to 1.7–2.6° (from 0.73°) and
estimator velocity RMSE to 0.04–0.06 m/s (from 0.006). With no damping at all the estimate-fed robot fell at 2.9 s.
**Actual cause:** Unmodelled leg dynamics (joint damping, leg inertia, swing-leg reaction). The scripted trot
never noticed because stiff position servos override them.
**Fix:** Integral action, not damping cancellation: ki_v = 4 s⁻² on horizontal velocity and ki_ωz = 10 s⁻² on
yaw rate, each clamped to 0.5. Seed 0, fed ground truth: tracking 0.23 → 0.091 m/s, yaw rate 0.248 → 0.196 rad/s.
The yaw integral alone cost nothing. The velocity integral raises the lean on the first straight from 1.07° to 2.69°
and estimator velocity RMSE from 0.006 to 0.016 m/s. The vx integrator saturates at its clamp from t ≈ 8 s, so
the robot still runs about 0.1 m/s slow and unwinds slowly at stops. `QPGains.comp_damping` (default 0) keeps the
rejected variant reproducible.

### 16. QP controller fed the estimate falls on seed 3
**Symptom:** In `make qp-compare`, the QP controller fed the estimate fell on sensor seed 3 at t = 7.0 s, during
the first straight. The other four seeds, and all five fed ground truth, walked the full 60 s.
**Hypothesis:** The tilt/accelerometer-bias ambiguity again (#7, #14), with a controller that acts on it harder.
**Test:** Estimated minus true roll/pitch at t = 2, 3, 4, 5 s for both controllers, seeds 0–4. Seed 3 starts
worst for both controllers (−2.9° roll, −2.3° / −2.5° pitch at t = 2 s, from static alignment with that seed's
accelerometer bias). The scripted trot's error grows to −15.4° roll by t = 5 s and it survives. The QP's grows to
−24.3° pitch by t = 4 s. True pitch reaches 28°, and the height fed back (leg kinematics through the wrong
attitude) reads 0.29–0.31 m while the trunk is at 0.17 m. Gains: kr 150 → 75 falls at 9.9 s; ki_v = 0 falls at
10.6 s; ki_v = 2 falls at 7.1 s. So the fall isn't caused by the tuning in #15, and no gain I tried avoids it.
**Actual cause:** Weak observability of tilt while walking straight, as in #14. The QP's force-controlled stance
has more attitude authority than the position-servoed trot, so it converts the estimate's error into real tilt
faster, and the height estimate collapses with it.
**Fix:** None in the controller. With the 4 s startup turn (`closed_loop.turn_first`) all five seeds walk fed
the estimate. Reported as 1/5 falls in the README.
