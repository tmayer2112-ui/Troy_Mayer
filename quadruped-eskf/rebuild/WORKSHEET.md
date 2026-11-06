# Worksheet: derive it before you code it

Do these on paper. Every question uses tiny numbers you can work by hand, and the code checks the
same numbers. Answers are in `ANSWERS.md`. Look only after you've written something down.

Notation: x = estimate, P = its variance (or covariance matrix), z = measurement, R = measurement
noise variance, Q = noise added by one predict step, K = Kalman gain.

---

## Module 1: the Kalman filter in one dimension (day 1)

**The idea:** two witnesses disagree. The prediction says one thing, the sensor says another, and each
has an uncertainty. How much should you believe each?

**1.1 Blending.** The prediction says x̂ = 10.0 m with variance P = 4 (σ = 2 m). A sensor says
z = 12.0 m with variance R = 1 (σ = 1 m). You'll blend them as

  x = x̂ + K (z − x̂)

- (a) What is x if K = 0? If K = 1? Say in words what K controls.
- (b) The blended estimate's error is (1 − K)·(prediction error) + K·(sensor error), and the two
  errors are independent. Using Var(aA + bB) = a² Var(A) + b² Var(B), write Var(x) as a function of K.

**1.2 The best K.** Find the K that makes Var(x) smallest: differentiate with respect to K and set it
to zero. Write the general formula in terms of P and R. Then plug in the numbers:
K = ____, x = ____, Var(x) = ____.

**1.3 Uncertainty after the update.** Show that, at the best K, Var(x) = (1 − K) P. Is the new
variance smaller than P? Smaller than R? Why does that make sense?

**1.4 Predict.** The robot then moves u = 0.05 m by odometry, and odometry adds error variance
q = 0.0009 per step. Write the new x and P. Why does predict *add* to P, while update *multiplies*
it by (1 − K)?

**Code:** `m1_kalman_1d.py`, then `python check.py 1`, then `python m1_kalman_1d.py`. Watch P rise on
every predict and drop at every measurement.

**Say it:** *"The gain is the prediction's variance over the total. If the prediction is less sure
than the sensor, the estimate moves most of the way to the measurement."*

---

## Module 2: position and velocity, with matrices (day 1)

Now the state has two numbers, x = [p, v], and P is a 2×2 matrix. The diagonal holds the two
variances. The off-diagonal is how the two errors are **linked**.

**Matrix product reminder (2×2).** Row times column:
[[a, b], [c, d]] @ [[e, f], [g, h]] = [[ae + bg, af + bh], [ce + dg, cf + dh]].
The transpose Aᵀ swaps rows and columns.

**2.1 F.** Over one step dt, at constant velocity: p_new = p + v·dt and v_new = v. Write the 2×2
matrix F such that x_new = F x.

**2.2 Predict by hand.** Take P = I (identity), dt = 0.1, and Q = [[0, 0], [0, 0.01]].

- (a) Compute F P, then (F P) Fᵀ, then add Q. Write P⁻ = [[__, __], [__, __]].
- (b) One entry of P⁻ was 0 in P and isn't any more. Which one? Say in words what it means.
  Hint: if the velocity estimate is wrong, what happens to the position estimate after 0.1 s?

**2.3 Update by hand: measure position only.** H = [1, 0] and R = 0.25.

- (a) S = H P⁻ Hᵀ + R = ____. Why is it just P⁻[0,0] + R?
- (b) K = P⁻ Hᵀ / S = [____, ____]. K has a **velocity** entry, even though velocity was never
  measured. Which entry of P⁻ put it there?
- (c) The sensor says the cart is 1.0 m further than predicted (residual z − H x = 1.0). How much
  does the position estimate move? The velocity estimate?
- (d) P⁺ = (I − K H) P⁻ = [[__, __], [__, __]]. Which variance shrank a lot? Which one barely moved?

**2.4 In words.** How does a position measurement fix velocity? Two sentences.

**Code:** `m2_kalman_2d.py`, then `python check.py 2`, then `python m2_kalman_2d.py`. Velocity is found
within about 0.1 m/s from position alone. Watch the corr(p, v) column.

**Say it:** *"P keeps the correlation between position and velocity errors, so a position residual
also corrects velocity. That's how foot-position measurements fix the robot's velocity."*

---

## Module 3: rotations, the gravity leak, finite differences (day 2)

**3.1 Why not add to a quaternion?** A quaternion has 4 numbers but a rotation has only 3 degrees of
freedom; the 4 numbers must have length 1. A small rotation δθ changes a quaternion by about
(0, δθ/2). Start at q = (1, 0, 0, 0), no rotation, and correct by 0.2 rad about x.

- (a) **Adding**, the way a normal EKF would: q + (0, 0.1, 0, 0) = ____. Its length is ____.
- (b) **Multiplying** by the rotation, the way the error-state filter does: Exp(0.2 about x) =
  (cos 0.1, sin 0.1, 0, 0), and q ⊗ that = ____. Its length is ____.
- (c) Name the two things that break if you put the quaternion straight into a normal EKF.

**3.2 The gravity leak.** This is the key equation behind the lean question. A robot stands still,
level. Its accelerometer reads f = (0, 0, 9.81): it feels the floor pushing up. The filter turns that
into world acceleration with a = R̂ f + g, where g = (0, 0, −9.81). Level and correct gives a = 0.

Now suppose the true orientation is the estimate tilted by a small error δθ: R_true = R̂ Exp(δθ) ≈
R̂ (I + [δθ]×). Here [δθ]× b just means the cross product δθ × b.

- (a) Cross product warm-up. For a = (a1, a2, a3) and b = (b1, b2, b3),
  a × b = (a2b3 − a3b2, a3b1 − a1b3, a1b2 − a2b1). Compute (0, δ, 0) × (0, 0, 9.81) = (__, __, __).
- (b) So the true acceleration differs from the filter's by R̂ (δθ × f). For δ = 1° = 0.01745 rad
  and R̂ = I, how big is that, and along which axis? ____ m/s²
- (c) Using a × b = −(b × a), rewrite R̂ (δθ × f) as (a matrix) · δθ. Over one step dt the velocity
  error grows by that times dt. That 3×3 matrix is the **velocity ← tilt block of F**.
  For R̂ = I, f = (0, 0, 9.81), dt = 0.001: its (v_x, tilt_y) entry = ____.

**3.3 Tilt vs accelerometer bias (the lean).** An accelerometer bias b, glued to the body, adds a
fake acceleration R̂ b.

- (a) Facing north (yaw 0, R̂ = I): write the fake world acceleration from a 1° tilt error (from
  3.2b) and from a bias b = (0.171, 0, 0). Can the legs tell them apart?
- (b) The robot turns to face east (yaw 90°). The bias turns with the body. The tilt error is about
  gravity, so it stays put in the world. Write both fake accelerations again.
- (c) Facing north, the legs only see the sum: "9.81·tilt + bias = 0.171" is one equation in two
  unknowns. What does the turn give you?

**3.4 Finite differences.** f(x) = x² at x = 3, with h = 0.001.

- (a) Forward difference (f(3 + h) − f(3)) / h = ____. Central difference
  (f(3 + h) − f(3 − h)) / 2h = ____. The true slope is 6. Which one is better, and why?
- (b) For a function of many inputs, write what column j of the Jacobian is, using a nudge e_j
  (all zeros except h at position j).
- (c) Why use h = 1e-6 and not 1e-12? Hint: a computer keeps about 16 significant digits.

**Code:** `m3_rotations_fd.py`, then `python check.py 3`, then `python m3_rotations_fd.py`. Part 2 prints
your formula next to the nudged answer. Part 3 prints the north/east table from 3.3.

**Say it:** *"I check every hand-derived Jacobian by nudging each input up and down by 1e-6,
rerunning the real code, and comparing the measured slope with the formula. In my tests they agree
to 1e-10."*

---

## Module 4: the predict step (day 3)

The full filter's nominal state is p, v, q, b_a (accel bias), b_g (gyro bias), plus one world position
per planted foot. The error state is 27 numbers: δp, δv, δθ, δb_a, δb_g, then 3 per foot.

**4.1 The nominal predict.** Write the update for one IMU sample (f = accel, ω = gyro, both body frame):

- a_w = ____   (remove the bias, rotate to world, add gravity back)
- p ← ____    (uses the *old* v; why?)
- v ← ____
- q ← ____    (multiply by Exp of the de-biased gyro × dt)
- then re-normalize q.

Check it with tiny numbers: level, still, f = (0, 0, 9.81), no bias. Then a_w = ____, so v stays ____.

**4.2 F, block by block.** For each block, say in words why it is there and why it has that sign:

| block (row ← column) | formula | in words |
|---|---|---|
| position ← velocity | I·dt | |
| velocity ← tilt | −R̂ [a]× dt | (you derived it in 3.2) |
| velocity ← accel bias | −R̂ dt | |
| tilt ← tilt | Exp(−ω dt) | |
| tilt ← gyro bias | −J_r(ω dt) dt ≈ −I·dt | |
| feet ← feet | I | |

(The position ← tilt and position ← bias blocks are the ½dt² versions of the velocity ones.)
For the velocity ← accel bias row: the filter uses f − b̂ but the truth is f − b, so what is the
true-minus-estimated acceleration in terms of δb = b − b̂?

**4.3 Q, with numbers.** The noise density of the accelerometer is N = 1.72e-3 m/s²/√Hz.

- (a) Q for velocity, per step, is N²·dt. With dt = 0.001, that's ____ (m/s)².
- (b) After 1000 steps (1 s) the variance has added up to ____, so σ_v = ____ m/s.
  This is a random walk: σ grows like √t.

**4.4 Order.** The code computes F, *then* moves the nominal state, *then* does P = F P Fᵀ + Q. Why must
F come from the state at the start of the step?

**Code:** the four Module 4 functions in `eskf.py`, then `python check.py 4`, then `python run.py imu-only`.
Write down the velocity error at 10 s and 20 s. Why does it jump after the turn (12-16 s)? Hint: 3.3.

**Say it:** *"Predict is IMU only: remove the biases, rotate the accelerometer into the world, add
gravity back, integrate. The uncertainty grows with F P Fᵀ + Q. Nothing gets corrected there."*

---

## Module 5: the leg update (day 4)

**5.1 The measurement.** A planted foot sits at world position d. The leg sensor (forward kinematics,
in the real robot) reports where the foot is relative to the body, in the body frame:

  z = R_trueᵀ (d − p_true) + noise.     The filter predicts h = R̂ᵀ (d̂ − p̂), and r = z − h.

Tiny numbers: body at p̂ = (1, 0, 0.27), level (R̂ = I), foot at d̂ = (1.19, −0.13, 0). Then h = ____.

**5.2 H by nudging, on paper.** H says how h changes when you nudge each error. Do three nudges:

- (a) Body position: p̂ → p̂ + δp. Then h changes by ____ δp, so the position block = ____.
- (b) Foot position: d̂ → d̂ + δd. The foot block = ____.
- (c) Tilt: R → R (I + [δθ]×), so Rᵀ → (I − [δθ]×) Rᵀ. Then h changes by −δθ × h = h × δθ = [h]× δθ.
  The tilt block = ____.
- (d) Every other block (velocity, biases, the other feet) is zero. Why?

**5.3 Trust.** S = H P Hᵀ + R_n and K = P Hᵀ S⁻¹.

- (a) Show that for one number (H = 1) this is exactly Module 1's K = P/(P + R).
- (b) The update uses the Joseph form, P ← (I − K H) P (I − K H)ᵀ + K R_n Kᵀ. In exact arithmetic it
  equals (I − K H) P. Why might you prefer the longer one in a filter that runs 60,000 times a minute?

**5.4 Inject and reset.** After the update, the error estimate dx is folded into the nominal state.

- (a) p, v, the biases and the feet are just added. Why is q multiplied by Exp(δθ) instead?
- (b) After injecting, the error estimate is zero again. Why does that make sense?
- (c) The reset G = I − ½[δθ]× in the tilt block is a tiny correction for the error's reference
  rotating. You don't need to derive it, just know it exists and that it's close to I.

**5.5 Touchdown (provided code: read `touchdown()` in eskf.py).** A new foot is placed at d = p̂ + R̂ z.

- (a) Nudge p̂: d moves by ____. Nudge the tilt: R̂ (I + [δθ]×) z = R̂ z + R̂ (δθ × z) = R̂ z − R̂ [z]× δθ.
  So J = [ I, 0, −R̂ [z]×, 0, ... ].
- (b) The new foot's variance is J P Jᵀ + (leg noise). In words: why does a new foot inherit the
  body's uncertainty?

**Code:** the three Module 5 functions in `eskf.py`, then `python check.py 5`, then `python run.py full`.

**Say it:** *"The measurement is the foot position from the legs, minus where the filter thinks that
foot is. Trust is the filter's uncertainty versus the leg noise. P's correlations spread the
correction to every state, including velocity and the biases."*

---

## Module 6: legs, forward kinematics and encoder noise (day 5)

The real filter doesn't get foot positions. It reads 12 joint encoders and runs forward kinematics (FK).

**6.1 A two-link leg in its own plane.** Thigh and calf are both L = 0.213 m. The hip angle q2 and
knee angle q3 both turn about y. Relative to the hip:
x = −L sin q2 − L sin(q2 + q3) and z = −L cos q2 − L cos(q2 + q3).

- (a) All joints at 0: x = ____, z = ____.
- (b) Standing pose q2 = 0.8, q3 = −1.6 (use sin(−a) = −sin a): x = ____, z = ____. Where is the foot?

**6.2 Abduction.** The whole leg plane turns about x by q1, and the thigh sits d = ±0.08 m to the
side (− for right legs): y = cos q1·d − sin q1·z and z' = sin q1·d + cos q1·z. Then add the hip position.
FR's hip is at (0.1881, −0.04675, 0). With all joints at 0, the FR foot is at (____, ____, ____).

**6.3 The leg Jacobian.** Column j says how the foot moves per radian of joint j. At the standing
pose: dx/dq2 = ____, dx/dq3 = ____, dz/dq3 = ____. Then check yours against fd_jacobian.

**6.4 Encoder noise.** A 14-bit encoder has 2¹⁴ = 16,384 counts per turn, so one count = ____ rad
= ____°. Rounding is uniform over ±½ count, which gives a variance of count²/12. Through a 0.3 m lever,
the foot's σ ≈ ____ mm. In R_leg = J (count²/12) Jᵀ + (3 mm)² I, which term wins? What would 6 bits do?

**Code:** `m6_legs.py`, then `python check.py 6`, `python m6_legs.py`, `python run.py legs` (14, 8 and 6 bits).

**Say it:** *"The leg measurement is forward kinematics on 14-bit joint encoders. Its noise is the
encoder rounding pushed through the leg Jacobian, plus 3 mm of model error, and the 3 mm dominates by
about a hundred times."*

---

## Module 7: contacts, and your bug #4 (day 5)

**7.1 Force.** Force samples [0, 2, 10, 20, 20, 10, 3, 0] N, and planted means > 5 N. Which samples
are planted? Which, if the force arrives 1 sample late?

**7.2 Schedule, advance, settle.** The schedule says stance for [0, 1, 1, 1, 1, 1, 0, 0].
- *Advance* 2 ends contact 2 samples before the schedule's liftoff: ____.
- *Settle* 2 counts a foot only after more than 2 samples in contact: ____.
- Force AND schedule with advance 1: ____.

**7.3 Why.**
- (a) A late force keeps a lifting foot "planted". What does the filter make of a foot it thinks is
  fixed, but which is moving at 0.5 m/s?
- (b) Why can the schedule end contact on time when the force is late?
- (c) Is using the schedule cheating on a real robot? When does it fail?

**7.4 Your bug, rebuilt.** In `python run.py contacts`, part A, each foot sinks 2 mm after touchdown and
rises 3.4 mm before liftoff, as in your DEBUG_LOG #4. Predict before you run it:
- If the filter treats a rising foot as fixed, does the height estimate drift up or down?
- What does waiting 20 ms after touchdown do?
- What does ending contact 5, 10 or 20 ms early do?

Then run it and tell the story in five beats: symptom, hypothesis, test, cause, fix.

**Code:** `m7_contacts.py`, then `python check.py 7`, then `python run.py contacts` (about a minute).

**Say it:** *"A foot counts as planted when the force is over 5 N and the schedule says stance, ending
5 ms early. Force alone is late at liftoff, and the end of stance is exactly when the foot starts to
move. That's what caused my vertical-drift bug."*

---

## Module 8: the walking controller (day 6)

Your résumé says "closed-loop control using Raibert foot placement and attitude/height controllers".
This module is the core of each, small enough to work by hand. In your real controller
(`python/qeskf/gait.py`) all three live inside `foot_targets()`.

**8.1 The gait clock.** A trot cycle takes 0.4 s. Each foot is planted for 60% of it (duty 0.6),
and the phase offsets are FR 0, FL 0.5, RR 0.5, RL 0. A foot's phase is
(time since walking started / 0.4 + its offset), keeping only the part after the decimal point.
The foot is planted while its phase < 0.6.

- (a) 0.10 s after walking starts, each foot's phase = ____. Which feet are planted?
- (b) Same at 0.25 s, and at 0.22 s.
- (c) Each foot is down for 0.24 s of every 0.4 s. What fraction of the time are all four down?
  Hint: draw both diagonal pairs on one timeline.
- (d) Why move diagonal feet together? Hint: draw a line from FR to RL. Where does it pass?
- (e) Your estimator uses this same clock. Where, and for what?

**8.2 Raibert foot placement: where a foot lands sets the speed.**

- (a) **The neutral point.** The body moves at v during a stance of T. If the foot lands v·T/2 ahead
  of the body, the body passes over it exactly at mid-stance. The leg brakes for the first half and
  pushes for the second half, equally, so the speed comes out unchanged.
  For v = 0.5 m/s and T = 0.2 s, the neutral point is ____ m ahead.
- (b) **The correction:** foot = neutral + k (v − v_des). The robot is going 0.6 m/s when told 0.5,
  and k = 0.06 s (your gait.py value). The foot lands ____ mm further ahead than neutral. Does that
  speed the robot up or slow it down? Why?
- (c) **A pendulum to see it with numbers.** Model the body as a point on a stiff, massless leg (the
  "linear inverted pendulum"). It falls away from its foot: x'' = (g/h)(x − foot).
  With h = 0.27 m: g/h = ____ s⁻² and ω = √(g/h) = ____ 1/s. If the body is 2 cm *behind* the
  foot, its acceleration is ____ m/s². Speeding up or braking?
- (d) Predict before you run anything: the robot starts standing still and is told to walk at 0.5
  m/s. What does k = 0 do? What might a very large k do?
- (e) In the terms you used for P, PD and PI: what kind of controller is Raibert's rule, and how
  often does it act?
- (f) **Your real controller is not this pendulum.** In gait.py the stance feet are
  position-controlled to sweep backwards at the *commanded* speed, so the stance legs do most of the
  speed control and the Raibert term only corrects what's left. On the pendulum, foot placement is
  the *only* speed control. Keep that in mind before using pendulum numbers for your robot.
- (g) **The 0.004 m/s question.** In gait.py, only the Raibert correction uses the measured
  (estimated) speed. If the estimate is off by 0.013 m/s, the foot moves by ____ mm. Use that to
  explain why tracking barely changed when the controller ran on your estimate (0.173 vs 0.169 m/s).

**8.3 Levelling loops (attitude and height).** A toy body has angle θ, with θ'' = u + push. The
controller only sees the *measured* angle θ_m = θ + e (e is the estimate's error) and the true rate
(from the gyro):

  u = −kp·θ_m − kd·θ' − ki·∫θ_m,    with kp = 150 and kd = 20.

- (a) PD only (ki = 0), no push, estimate off by e. At rest (θ' = 0 and θ'' = 0, so u = 0), solve
  for the true angle: θ = ____. That's your robot's lean.
- (b) PD only, a perfect estimate (e = 0), and a constant push of 3 rad/s². At rest:
  θ = ____ rad = ____°. A constant load on a spring-like controller leaves a constant offset.
- (c) Add the integral (ki = 300). It only stops changing when θ_m = 0. So with the push,
  θ = ____. With the estimate off by e, θ = ____. Can an integral fix a wrong estimate?
- (d) In gait.py, the attitude loop changes each leg's length by k_att·angle + d_att·rate
  (k_att 1.2, d_att 0.06 s). Is that P, PD or PI? The height loop is k_height·error plus an
  integral (0.002 × error added every 2 ms). Which is it, and why did height need the I?
  Hint: 8.3b, and the position servos are springs.

**Code:** `m8_walking_control.py`, then `python check.py 8`, then `python m8_walking_control.py`.
Part B prints the speed after each step for k = 0, 0.06, 0.11 and 0.25. Part C prints the four
cases from 8.3.

**Say it:** *"Raibert foot placement puts the foot at the neutral point, half the stance travel
ahead, plus a correction proportional to the speed error: too fast, the foot lands further ahead
and the leg brakes. The attitude loop is a PD that levels the estimated trunk, so an estimate error
becomes a real lean, and an integral can't remove it, because it zeroes the measured angle, not
the true one."*

---

---

## Module 9: the predict step in C++ (day 6)

You port Module 4 to C++ with Eigen, write the finite-difference test, and run a GoogleTest suite
with a parity test against the Python reference. This is the shape of your real `cpp/` folder.

**9.1 Python → Eigen.** Fill in the Eigen column:

| numpy | Eigen |
|---|---|
| `np.zeros((15, 15))` | `Mat15::________()` |
| `A @ B`, `A.T` | `A ___ B`, `A.________()` |
| `F[3:6, 6:9] = X` | `F.block<3, 3>(___, ___) = X` |
| `quat_mul(q1, q2)` | `q1 ___ q2` (`Eigen::Quaterniond(w, x, y, z)`; careful: `.coeffs()` is x, y, z, w) |

**9.2 Fixed or dynamic size.** `Matrix<double, 15, 15>` has its size known when compiling and lives
on the stack. `MatrixXd` is sized at run time and allocates on the heap. Run the benchmark: is fixed
size faster at 15×15? If not, why use it in a 1 kHz loop? (Hint: the worst case matters, not the average.)

**9.3 Your real filter's trick** (`cpp/src/eskf.cpp`, lines 61-66). With 4 feet the state is 27 and
F = [[F_c, 0], [0, I]], where F_c is 15×15. Count multiply-adds:
- dense F P Fᵀ: 2 × 27³ = ____;
- structured, F_c P_cc F_cᵀ (2 × 15³) plus F_c P_cf (15·15·12): ____.

How many times fewer?

**9.4 Tests.** `TEST`, `TEST_P` with `INSTANTIATE_TEST_SUITE_P(..., Range(0u, 5u))`, `EXPECT_LT`.
Why can a parity test (C++ = Python) pass with a wrong formula, and which test catches it?

**Build and run** (README, "C++ and MuJoCo"):
```
cd cpp
cmake -S . -B build
cmake --build build
./build/rebuild_tests_solution
./build/rebuild_tests
./build/rebuild_bench
```
`rebuild_tests_solution` checks your setup; `rebuild_tests` runs your code. Your job: `predict.hpp` and `fd.hpp`.

**Say it:** *"I ported the predict step to C++ with Eigen, with fixed-size 15×15 matrices so the 1 kHz
loop never allocates, tested F against central differences in GoogleTest, and matched my Python
reference to 1e-14 m."*

---

## Module 10: honest uncertainty, what can't be known, and your numbers (day 7)

**10.1 NEES.** A velocity error e = (0.01, 0.02, 0) m/s, with P = diag(0.01², 0.01², 0.01²).
NEES = eᵀP⁻¹e = ____. If P is honest, NEES averages 3 (one per component). What does an average of 30
mean? Of 0.3?

**10.2 Predict before running.** If Q is 100× too small, is NEES big or small? And if Q is 100× too big?

**10.3 Observability by hand, with Module 2's cart.** F = [[1, 0.1], [0, 1]].
- Measuring position, H = [1, 0]: O = [H; HF; HF²] = ____. Can it see both states?
- Measuring only velocity, H = [0, 1]: O = ____. Which direction is invisible, and why?
- Now the filter: which 4 directions should the SVD find (Module 5's "shift and spin" test)?

**Code:** `m10_consistency.py`, then `python check.py 10`, then `python run.py report`. Add `--plot` if
matplotlib is installed. It takes 1-2 minutes.

**10.4** Fill these in from the report, then say each line out loud:

- Legs cut velocity RMSE from ____ to ____ m/s.
- After 30 s, yaw is off ____° and position ____ m. The filter's own σ for them ____.
- Before the turn, |corr(tilt, bias)| = ____ and the tilt error is ____°. After the turn: ____°.
- 10 cm/s of foot slip gives ____ m/s. Why is that almost exactly the slip speed?
- One sign flipped in F gives ____ m/s. Which check catches it?
- NEES with Q ×0.1 / ×1 / ×10: ____ / ____ / ____.
- Unobservable directions: ____. The next weakest pair tilt with ____, at a ratio of ____.

**10.5** Answer these in 3 sentences each, using what you built:

- Why did the robot lean on your estimate, and why didn't the filter fix it?
- What can't the filter estimate, and why not?
- How does a foot-position measurement correct velocity?
- Why an error-state filter instead of a normal EKF?
- How do you know your Jacobians are right?
- How do you decide a foot is planted, and what goes wrong if you get it late?
- Where does the leg measurement come from, and what is its noise?
- What is Raibert foot placement, and where does your estimate enter the controller?

**10.6** Open the real project next to your rebuild. The README has the map. Find each of your
functions, and note one thing the real one does that yours doesn't.

**10.7 Your real numbers and the retell.**
1. Read `REAL_NUMBERS.md` out loud.
2. Close everything and draw the diagram (IMU → predict → state ← update ← legs, and the controller
   on top).
3. Explain the project in 90 seconds: problem, idea, predict, update, results, closing the loop, one
   limit. Record it.
4. Then do one mock interview with Claude on the real project.

---

## Module 11 (optional): MuJoCo (day 7)

You simulate one Go1 foot and an IMU. Setup is in the README.

**11.1 What an accelerometer reads:** f = Rᵀ(a − g). Predict each reading:
- free fall: ____;
- resting level: ____;
- resting, pitched 90° nose-down (body x points down): ____, on which axis?

**11.2 When is a foot planted?** A quarter of the Go1's weight drops 2.7 cm onto one foot sphere with
your model's soft contact. Predict: when the force passes 5 N, has the foot stopped?

Run it, then fill in: force > 5 N at ____ ms, foot centre at ____ mm, sinks a further ____ mm, settled
at ____ ms. Compare with MuJoCo's stiff default.

Why did your project keep the soft foot instead of stiffening it? (DEBUG_LOG #3)

**Code:** `m11_mujoco.py`, then `python check.py 11`, then `python m11_mujoco.py`.

**Say it:** *"An accelerometer reads specific force: zero in free fall, +g at rest. In MuJoCo the Go1's
soft foot reports 5 N of contact while it still has 12 mm to sink, which is why contact needs a settle
time and an early liftoff."*
