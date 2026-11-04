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

## Module 6 (reference numbers, seed 1)

**6.1**

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

**6.2** (short versions; your own words are better)

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

**6.3** Things the real filter has that the rebuild doesn't:
- forward kinematics on encoders, with encoder noise mapped through the leg Jacobian (`kin_cov`);
- P resized at every touchdown and liftoff (`augment`, `marginalize`);
- contact from force AND schedule, ending 5 ms early, with a 10 ms settle;
- an optional χ² gate;
- a C++ twin tested for parity.
