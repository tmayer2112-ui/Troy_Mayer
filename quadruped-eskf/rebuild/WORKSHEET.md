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

## Module 2: position and velocity, with matrices (day 2)

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

## Module 3: rotations, the gravity leak, finite differences (day 3)

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

## Module 4: the predict step (day 4)

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

## Module 5: the leg update (day 5)

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

## Module 6: experiments and your numbers (day 6)

Run `python run.py report`. Add `--plot` if matplotlib is installed. It takes 1-3 minutes.

**6.1** Fill in from the report, then say each line out loud:

- Legs cut velocity RMSE from ____ to ____ m/s.
- After 30 s, yaw is off ____° and position ____ m. The filter's own σ for them ____.
- Before the turn, |corr(tilt, bias)| = ____ and the tilt error is ____°. After the turn: ____°.
- 10 cm/s of foot slip gives ____ m/s of velocity error. Why is it almost exactly the slip speed?
- Flipping one sign in F gives ____ m/s. Which check would have caught it?

**6.2** Now answer the questions you missed before, in 3 sentences each, using what you built:

- Why did the robot lean when the controller used your estimate, and why didn't the filter fix it?
- What can't the filter estimate, and why not?
- How does a foot-position measurement correct velocity?
- Why an error-state filter instead of a normal EKF?
- How do you know your Jacobians are right?

**6.3** Open `python/qeskf/eskf.py` from the real project next to your `rebuild/eskf.py` (the README has
the line map). Find each of your functions in it. Note one thing the real one does that yours
doesn't.

---

## Day 7: retell

Close everything. Draw the whiteboard diagram (IMU → predict → state ← update ← legs). Then explain
the filter in 90 seconds as if to an interviewer: problem, idea, predict, update, results, one
limit. Record it once. Compare with how you answered before this week.
