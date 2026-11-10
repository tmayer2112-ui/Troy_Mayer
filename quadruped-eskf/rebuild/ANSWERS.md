# Answers

Check one module at a time, after you've written your own answer. The run numbers below come from
seed 1, the default. Yours will match if your code is right.

---

## Module 1

**1.1** (a) K = 0 gives x = 10 (ignore the sensor). K = 1 gives x = 12 (ignore the prediction). K is
how far you move from the prediction toward the measurement.
(b) Var(x) = (1 − K)²·4 + K²·1.

**1.2** dVar/dK = −2(1 − K)P + 2KR = 0, so K(P + R) = P and **K = P / (P + R)**.
K = 4/5 = **0.8**, x = 10 + 0.8 × 2 = **11.6**, Var(x) = 0.2² × 4 + 0.8² × 1 = 0.16 + 0.64 = **0.8**.

**1.3** With K = P/(P + R), 1 − K = R/(P + R), so
Var = R²P/(P + R)² + P²R/(P + R)² = PR/(P + R) = (1 − K)P. Here 0.2 × 4 = 0.8.
It's smaller than both P (4) and R (1): two independent witnesses together beat either one alone.

**1.4** x ← x + 0.05 and P ← P + 0.0009. Predict adds a new, independent error, and independent
variances add. Update combines two sources of information, which can only shrink the uncertainty.

---

## Module 2

**2.1** F = [[1, dt], [0, 1]].

**2.2** (a) F P = F = [[1, 0.1], [0, 1]]. Fᵀ = [[1, 0], [0.1, 1]].
(F P) Fᵀ = [[1·1 + 0.1·0.1, 1·0 + 0.1·1], [0·1 + 1·0.1, 0·0 + 1·1]] = [[1.01, 0.1], [0.1, 1]].
Adding Q gives **P⁻ = [[1.01, 0.1], [0.1, 1.01]]**.
(b) The off-diagonal 0.1 is new. A wrong velocity becomes a wrong position after 0.1 s, so the two
errors are now **linked**: if one is high, the other probably is too.

**2.3** (a) S = 1.01 + 0.25 = **1.26**. H = [1, 0] picks out the top-left entry, so H P Hᵀ = P⁻[0,0].
(b) K = [1.01, 0.1] / 1.26 = **[0.8016, 0.0794]**. The velocity entry comes from the off-diagonal 0.1.
(c) Position moves by 0.80 m and velocity by **0.079 m/s**, from a position-only measurement.
(d) I − K H = [[0.1984, 0], [−0.0794, 1]], so
**P⁺ = [[0.2004, 0.0198], [0.0198, 1.0021]]**. Position variance fell from 1.01 to 0.20. Velocity
barely moved (1.01 to 1.002): one measurement says little about velocity, but a steady stream of them
does (the demo gets σ_v down to about 0.08 m/s).

**2.4** A velocity error turns into a position error over time, and P records that link. When a
position residual shows up, K assigns part of it to velocity.

---

## Module 3

**3.1** (a) (1, 0.1, 0, 0), length √1.01 = **1.005**: not a rotation any more.
(b) (cos 0.1, sin 0.1, 0, 0) = (0.995, 0.0998, 0, 0), length **1** exactly.
(c) First, adding breaks the unit length, so you renormalize behind the filter's back and the
covariance no longer matches the state. Second, the 4×4 covariance is singular along the
quaternion's own direction, where there's no real uncertainty.
The error-state fix: keep the quaternion as the nominal state, estimate a 3-number rotation error,
and multiply it in.

**3.2** (a) (0, δ, 0) × (0, 0, 9.81) = (δ·9.81 − 0, 0 − 0, 0 − 0) = **(9.81δ, 0, 0)**.
(b) 9.81 × 0.01745 = **0.171 m/s²**, along x (forward). The filter doesn't see this acceleration,
so its velocity drifts at 0.171 m/s every second.
(c) R̂ (δθ × f) = −R̂ (f × δθ) = **−R̂ [f]× δθ**, so the block is −R̂ [f]× dt. For f = (0, 0, 9.81),
[f]× = [[0, −9.81, 0], [9.81, 0, 0], [0, 0, 0]]. Its negative has +9.81 in row 0, column 1, so
the (v_x, tilt_y) entry = 9.81 × 0.001 = **0.00981**. `python m3_rotations_fd.py` prints the same
number from nudging.

**3.3** (a) Both are (0.171, 0) pointing north. Identical, so the legs can't tell them apart.
(b) Tilt: still (0.171, 0) north. Bias: now **(0, 0.171)**, pointing east.
(c) A second, independent equation. Two equations, two unknowns, one answer. That's why a turn
lets the filter separate tilt from bias, and why the robot in your project stopped leaning after one.

**3.4** (a) Forward: (9.006001 − 9)/0.001 = **6.001**. Central: (9.006001 − 8.994001)/0.002 = **6.000**.
Central is better: its error shrinks like h² instead of h. It's exact for a parabola.
(b) Column j = (f(x + e_j) − f(x − e_j)) / 2h.
(c) Rounding. With h = 1e-12, f(x + h) and f(x − h) agree in their first ~12 digits, so their
difference keeps only ~4 good ones (an error of about 1e-16/h). With h = 1e-6 the two errors balance
near 1e-10. In your C++ tests, a sweep put the best step near 1e-5, and 1e-6 gives about 1e-10.

---

## Module 4

**4.1** a_w = R̂ (f − b̂_a) + g. p ← p + v dt + ½ a_w dt². v ← v + a_w dt. q ← q ⊗ Exp((ω − b̂_g) dt),
then normalize.
p uses the old v because the step starts at that velocity. With constant acceleration over the step,
p + v dt + ½ a dt² is exact.
Tiny numbers: a_w = (0, 0, 9.81) + (0, 0, −9.81) = **0**, so v stays **0**.

**4.2**

| block | in words |
|---|---|
| position ← velocity, I·dt | a velocity error, held for dt, becomes a position error |
| velocity ← tilt, −R̂ [a]× dt | the gravity leak from 3.2 |
| velocity ← accel bias, −R̂ dt | true − estimated acceleration = R̂(f − b) − R̂(f − b̂) = −R̂ δb |
| tilt ← tilt, Exp(−ω dt) | the error is written in the body frame, and the body rotates by ω dt each step. So a tilt error that is fixed in the *world* looks rotated backwards in the new body frame. That's why the tilt error "stays put" when the robot turns (3.3b). |
| tilt ← gyro bias, −J_r dt | true − estimated rate = −δb_g, so an unseen gyro bias turns into tilt error at δb_g per second |
| feet ← feet, I | planted feet don't move. Their small allowed drift is in Q. |

**4.3** (a) (1.72e-3)² × 0.001 = **2.9e-9 (m/s)²**. (b) 1000 × 2.9e-9 = 2.9e-6, so σ_v = **1.7 mm/s**
after 1 s, 3.8 mm/s after 5 s (√5 times more), and so on.

**4.4** F is the slope of the step *as it was taken*: it uses the R̂ and the biases at the start of the
step. Computed after the step, it uses an R̂ already rotated by ω dt. The result is slightly wrong,
and `check.py 4` catches it.

**imu-only run (seed 1):** about 0.1 m/s of velocity error at 10 s, 0.3 at 15 s, 1.6 at 20 s, 4.0 at 30 s.
**Why it jumps after the turn:** the static alignment hid the horizontal accelerometer bias inside the
tilt. While the robot faces the same way, the two fake accelerations cancel exactly. After the 90°
turn the bias has turned with the body and the tilt error hasn't, so they stop cancelling. The
mismatch is |R(90°) b − b| = √2 × 0.175 = 0.25 m/s², and the velocity error grows at about that rate
from 15 to 20 s. That's 3.3 again, seen from the other side.

---

## Module 5

**5.1** h = Iᵀ ((1.19, −0.13, 0) − (1, 0, 0.27)) = **(0.19, −0.13, −0.27)**.

**5.2** (a) h = R̂ᵀ(d − p): increasing p shrinks d − p, so the block is **−R̂ᵀ**.
(b) **+R̂ᵀ**. (c) **[h]×**.
(d) h doesn't depend on velocity, the biases or the other feet *right now*. They reach the measurement
only through earlier predict steps, which is exactly what the correlations in P carry. That's how one
foot residual still corrects velocity and the biases.

**5.3** (a) S = P + R and K = P/(P + R). It's Module 1.
(b) Rounding. Over thousands of updates, (I − K H) P can drift away from symmetric, or even get a
negative variance. The Joseph form is a sum of two positive pieces, so it stays symmetric and
positive. Both give the same answer in exact arithmetic, and the check accepts either.

**5.4** (a) The attitude error is defined as a rotation, R_true = R̂ Exp(δθ). Multiplying keeps q a
valid rotation and matches that definition (3.1).
(b) The nominal state has just been moved by the best guess of its error, so the best guess of the
error that's left is zero.

**5.5** (a) Nudging p̂ moves d by the same amount, so the block is I. Tilt: −R̂ [z]×, as shown.
(b) The new foot's position is computed *from* the body's position and orientation. If the body
estimate is 7 cm off, the foot gets placed 7 cm off too. Its error is the body's error plus leg
noise, and it is correlated with the body's. P has to record that, or later updates would count the
same information twice.

**full run (seed 1):** about 0.004 m/s velocity RMSE, against 1.76 m/s for IMU only.

---

## Module 6

**6.1** (a) x = 0 and z = −0.426: the leg hangs straight down, 2 × 0.213 m.
(b) x = −0.213 sin 0.8 − 0.213 sin(−0.8) = **0**, and z = −2 × 0.213 × cos 0.8 = **−0.2968**. A symmetric
knee puts the foot straight under the hip, 0.297 m down.

**6.2** y = −0.08 and z = −0.426, plus the hip, gives **(0.1881, −0.12675, −0.426)**.

**6.3** dx/dq2 = −L cos q2 − L cos(q2 + q3) = **−0.2968**. dx/dq3 = −L cos(q2 + q3) = **−0.1484**.
dz/dq3 = L sin(q2 + q3) = 0.213 × sin(−0.8) = **−0.1528**. (dz/dq2 = 0 at this pose.) The abduction
column for FR is (0, −z, d) = (0, 0.2968, −0.08). `python m6_legs.py` prints the whole matrix.

**6.4** One count = 2π / 16,384 = **3.8e-4 rad = 0.022°**. σ = count / √12 = 1.1e-4 rad, and × 0.3 m
gives **≈ 0.03 mm** (the code finds up to 0.04 mm, adding up all three joints). The 3 mm model error wins,
by about 100× in σ and 10,000× in variance. At 6 bits one count is 5.6° and the rounding reaches about
10 mm. `run.py legs`: velocity RMSE is 0.0037 m/s at 14 bits (unchanged), 0.0041 at 8 bits, and
**0.025 at 6 bits**, where the filter still assumes 3 mm and is fooled.

---

## Module 7

**7.1** Force > 5 N: **[0, 0, 1, 1, 1, 1, 0, 0]**. One sample late: **[0, 0, 0, 1, 1, 1, 1, 0]**. It
starts a sample late, which is harmless, and ends a sample late, which is not.

**7.2** Advance 2: **[0, 1, 1, 1, 0, 0, 0, 0]**. Settle 2: **[0, 0, 0, 1, 1, 1, 0, 0]**. Force AND
schedule, advance 1: **[0, 0, 1, 1, 1, 0, 0, 0]**.

**7.3** (a) It reads the foot's motion as the *body* moving the other way, so the velocity error is about
the foot's speed for those samples. It's the slip result again.
(b) The schedule is the controller's own plan: no sensor, no delay. With AND, contact ends as soon as
either signal says "off".
(c) No. The controller plans the gait, so the estimator may use the plan. It fails on contacts nobody
planned: a stumble, a push, rough ground.

**7.4** It drifts **down**: a foot rising while it's assumed fixed reads as the body sinking. Waiting
after touchdown makes it **worse**, because it skips the early sink that partly cancelled the rise.
Ending early helps, and 20 ms overshoots so the drift goes **up**. Reference run (20 s):

| Policy | Height error at 20 s |
|---|---|
| whole stance | −60 mm |
| settle 20 ms | −115 mm |
| end 5 ms early | −35 mm |
| end 10 ms early | −16 mm |
| end 20 ms early | **+15 mm** (the sign flips) |

Your project measured −0.61 → −1.05 m with settling, and −0.61 → −0.08 → +0.31 m ending 0/10/20 ms
early, over 60 s: the same pattern. Part B (latency): force only 0.0047 → 0.0061 → 0.0154 m/s at
0/10/30 ms; force AND schedule 0.0047 → 0.0050 → 0.0069. The toy doesn't reproduce your project's 6.6°
yaw result, so quote that one from the project.

---

## Module 8

**8.1** (a) FR 0.25, FL 0.75, RR 0.75, RL 0.25, so **FR and RL** are planted.
(b) At 0.25 s: FR 0.625, FL 0.125, RR 0.125, RL 0.625, so **FL and RR**. At 0.22 s: FR 0.55,
FL 0.05, RR 0.05, RL 0.55, so **all four**.
(c) One pair is down from 0 to 0.24 s and the other from 0.20 to 0.44 s. They overlap for 0.04 s
twice per cycle: 0.08 s of every 0.4 s = **20%**. `m8_walking_control.py` measures the same.
(d) The line between two diagonal feet passes under the middle of the body, so with only two feet
down the body is still balanced over its support, front to back and side to side.
(e) Contact detection: a foot counts as planted only if the force is over 5 N AND the schedule says
stance, and the schedule ends contact 5 ms before liftoff (Module 7, and your E4 experiment).

**8.2** (a) 0.5 × 0.2 / 2 = **0.05 m**. The code uses the pendulum's exact version, 0.045 m.
(b) 0.06 × 0.1 = **6 mm** further ahead. It **slows the robot down**: with the foot further ahead,
the body spends longer behind it (braking) than in front of it (pushing).
(c) g/h = 9.81 / 0.27 = **36.3 s⁻²** and ω = **6.03 1/s**. With x − foot = −0.02 m,
x'' = 36.3 × (−0.02) = **−0.73 m/s²**: braking.
(d) k = 0 never changes speed: told to walk, the robot stays at 0 (the k = 0 row). A very large k
over-corrects: each step flips the error and makes it bigger (the k = 0.25 row). For this pendulum,
k ≈ 0.11 s corrects the whole error in one step, and anything above ≈ 0.22 s is unstable.
(e) A **P controller on velocity** that acts once per step (every 0.2 s), through foot placement.
(g) 0.06 × 0.013 = 0.00078 m, about **0.8 mm**. The body travels 12 cm during each stance, so the
estimate barely moves where the feet land. The 0.17 m/s tracking error comes from the controller
itself. Your estimator is the speedometer and the controller is the driver.

**8.3** (a) 0 = −kp(θ + e), so **θ = −e**. The robot leans by exactly the estimate's error, the
other way.
(b) 0 = −kp θ + 3, so θ = 3/150 = **0.02 rad = 1.15°**.
(c) With the push: **θ = 0**. With the estimate off by e: **θ = −e**, still. **No**: the integral
drives the *measured* angle to zero, so a wrong estimate stays a real lean.
(d) Attitude: **PD** (P on the angle, D on the rate). Height: **PI**. The trunk sags under its
weight because position servos act like springs. That's a constant load on a P-like loop, which
leaves a constant offset (8.3b), and the I removes it.

---


**8.4** (a) τ = (s − 0.5) × 0.24 = −0.12, 0, +0.12 s, so x − nominal = −0.5·τ = **+60, 0, −60 mm**. The foot
lands ahead of the hip, passes under it at mid-stance, and lifts off behind it.
(b) **0.06 m**. It's Raibert's v·T/2, using the **commanded** speed and one foot's 0.24 s stance. The toy
used the measured speed and the exact pendulum formula.
(c) At s = 0.8: s_early = −0.133, τ = −0.152 s, x = **+76 mm**. At s = 1: **+60 mm**. That's −16 mm in
0.032 s = **−0.5 m/s** in the body frame, exactly −v_cmd, so the foot's speed **over the ground is 0** at
touchdown (the demo prints −0.50 and 0.00). It lands without skidding, which was DEBUG_LOG #2's fix.
(d) Lift: **0, 70, 0 mm**. A raised cosine has zero vertical speed at liftoff and touchdown (its slope,
sin 2πs, is 0 at both ends), so the foot doesn't slam down. A triangle would hit the ground at full speed.
(e) FL: 1.2 × 0.0349 × 0.127 = **+5.3 mm**; FR: **−5.3 mm**. A higher target means a shorter leg, so the
high left side is pulled down and the right side pushed up: it levels. It's a **PD** (P on the angle, D on
the rate). The lever arm appears because a foot 0.13 m out must move 0.13 × angle to undo that angle.
(f) While the body yaws, a planted foot stays put in the world, so in the body frame it turns the other way.
(g) **v_body** shifts touchdown through the Raibert term (0.06 × error). **roll, pitch, ω** set the
attitude dz on every leg. **height** sets h_cmd through the height PI. The **tilt** estimate made the
robot lean: a 6.8° tilt error asked every leg to "level" a trunk that was already level.

---

## Module 9

**9.1** `Mat15::Zero()`, `A * B`, `A.transpose()`, `F.block<3, 3>(3, 6)`, `q1 * q2`.

**9.2** On the reference machine both took about 1.1 µs for F P Fᵀ, so at 15×15 fixed size isn't
faster on average. The reason to use it in a 1 kHz loop is predictability: no heap allocation inside
the loop. Your project's one 21 ms spike turned out to be the host OS (DEBUG_LOG #12), which is why it
reports the median (18 µs) and p99 (60 µs) instead of one number.

**9.3** Dense: 2 × 19,683 = **39,366**. Structured: 6,750 + 2,700 = **9,450**. About **4.2× fewer**.

**9.4** Parity compares two versions of the *same* maths, so a mistake made in both still passes. The
finite-difference test checks each Jacobian against the code's own nonlinear step, and the RMSE against
ground truth checks the model itself. Reference: all 7 tests pass. Parity after 1000 predicts:
|Δp| = **1.4e-14 m**, |Δv| = 1e-16 m/s.

---

## Module 10 (reference numbers, seed 1)

**10.1** e/σ = (1, 2, 0), so NEES = 1 + 4 + 0 = **5**. An average of 30 means the errors are about
√(30/3) ≈ 3× bigger than P claims (overconfident). An average of 0.3 means P is about 3× too big
(underconfident).

**10.2** Q too small → P too small → NEES big. Q too big → NEES small. The report: Q ×0.1 → **32**,
×1 → **2.3**, ×10 → **0.74**. Your real project is overconfident even with the datasheet Q (median 13),
because foot sinking and FK error aren't the white noise Q and R assume (`REAL_NUMBERS.md`).

**10.3** O = [[1, 0], [1, 0.1], [1, 0.2]]: rank 2, so it sees both. With H = [0, 1]:
O = [[0, 1], [0, 1], [0, 1]], rank 1, and **position** is invisible: shift the cart and nothing measured
changes. For the filter: 3 position + yaw. The report finds **4**, and the next weakest pair tilt with
horizontal accelerometer bias at a ratio of **0.102 ≈ 1/g**. That's the lean again, and it matches your
project's README (0.10 ≈ 1/g).

**10.4**

- Legs cut velocity RMSE from **1.76** to **0.0037** m/s, about 476×.
- After 30 s, yaw is off **0.34°** and position **0.04 m**. The filter's σ for them only grows
  (0.35° and 0.04 m), so it's honest about not knowing them.
- Before the turn, |corr(tilt, bias)| = **1.000** and the tilt error is **0.91°**. That's close
  to the bias / g = 1.02° that alignment hid. After the turn: **0.03°**. Without the turn it
  stays at **0.87°**.
- 10 cm/s of slip gives **0.100 m/s**. The filter assumes the foot is fixed, so a foot sliding back
  at 0.1 m/s reads exactly like the body moving 0.1 m/s faster. Your project measured the same
  thing (E2: 0.1 m/s of slip → 0.0996 m/s of error).
- One flipped sign in F gives **5.9 m/s**. `check.py 4`'s finite-difference test names the block.
- NEES 32 / 2.3 / 0.74. Unobservable directions: 4, with the next weakest at tilt/bias = 0.102.

**10.5** (short versions; your own words are better)

- *Lean:* "Tilt and accel bias look the same until the robot turns: 1° of tilt leaks 0.17 m/s² of
  gravity sideways, like a 0.17 m/s² bias. The controller levelled the tilt it was told about, so the
  estimate's error became a real lean. A turn separates them because the bias rotates with the body
  and gravity doesn't. In my rebuild the tilt error drops from 0.9° to 0.03° at the turn."
- *What drifts:* "Position and yaw. The legs measure relative to the body, so shifting or spinning the
  whole world changes nothing. Gravity pins roll and pitch."
- *Position fixes velocity:* "Velocity errors turn into position errors over time, and P records that
  link, so K sends part of each foot-position residual to velocity."
- *Error-state:* "A quaternion is 4 numbers for 3 degrees of freedom. Adding a correction breaks unit
  length and the 4×4 covariance is singular. So I estimate a 3-number rotation error and multiply it in."
- *Jacobians:* "I nudge each error-state input by ±1e-6, rerun the real nonlinear step, and compare the
  measured slope with my formula. They agree to about 1e-10. A flipped sign shows up a million times
  over the tolerance."
- *Raibert:* "Each foot lands at the neutral point, half the stance travel ahead, plus k times the speed
  error. Too fast, it lands further ahead and the leg brakes. It's a P loop on velocity that acts once per
  step. My estimate enters through the speed in that term, and through the roll, pitch and height the
  levelling loops use."
- *Contacts:* "Force over 5 N AND the schedule, ending 5 ms early. Force alone is late at liftoff, when
  the foot starts to move. That caused my vertical-drift bug, and Module 7 rebuilds it."
- *Leg measurement:* "Forward kinematics on 14-bit encoders. Rounding is about 0.04 mm at the foot, so
  the 3 mm model error dominates."

**10.6** Things the real filter has that the rebuild doesn't:
- the encoder-noise covariance used inside every update (`kin_cov`); the rebuild's filter keeps a fixed 3 mm;
- P resized at every touchdown and liftoff (`augment`, `marginalize`) instead of fixed foot slots;
- contact forces from MuJoCo's contact solver, not a weight-sharing profile;
- an optional χ² gate on each foot's residual;
- the whole step in C++ (feet and updates too), timed at 18 µs median.

---

## Module 11

**11.1** Free fall: **0**. Resting level: **+9.81 up**. Pitched 90° nose-down: **−9.81 on the body x
axis**. Body x now points down, and the reading points up.

**11.2** Force passes 5 N at **75 ms** with the foot centre at **22.1 mm**, 0.9 mm into the floor. Your
project's foot was at 21.9 mm. It sinks **11.6 mm** more and settles at **150 ms**. The stiff default
sinks 4.0 mm and bounces. So "in contact" isn't "stopped". The soft foot stands in for the rubber pad,
and your project kept it because "tuning the world to suit the estimator would hide a real modelling
error" (DEBUG_LOG #3).

