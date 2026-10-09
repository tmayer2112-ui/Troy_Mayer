# QP force-balance controller: a plain-language summary

All results are **in simulation** (MuJoCo, Unitree Go1 model, 1 kHz physics). Numbers come from
`make qp-compare` (`results/qp_compare.json`, `results/logs/qp_compare.log`); the scripted-trot numbers come
from `make closed-loop` (`results/closed_loop.json`).

## What problem it solves

The robot already had a walking controller, the "scripted trot". It decides where every foot should be at
every instant and tells stiff joint motors to hold those positions. It works, but it never thinks about
*forces*. So it can't directly ask the ground to push the body forward a bit harder, or to stop a tilt.

The QP force-balance controller changes the question for the feet on the ground. Instead of "where should
each foot be?" it asks: **"what push should each foot give the ground, so that the body moves the way I
want?"** The feet in the air still follow the scripted trot's swing path.

## How it works, in three steps (500 times a second)

**1. Decide how the body should accelerate.** Compare what the body is doing with what was commanded, and
turn each error into a desired acceleration, like a spring and damper:

    a_des = [ kv·(vx_cmd − vx) + ki_v·∫(vx_cmd − vx),
              kv·(vy_cmd − vy) + ki_v·∫(vy_cmd − vy),
              kpz·(h_des − h) − kdz·vz ]
    ω̇_des = [ −kr·roll − kdr·ωx,
              −kr·pitch − kdr·ωy,
              kdy·(ωz_cmd − ωz) + ki_ωz·∫(ωz_cmd − ωz) ]

(v = velocity, h = trunk height, ω = rotation rate. The ∫ terms slowly build up to cancel a steady error.)

**2. Find foot forces that produce that acceleration.** Pretend the robot is a single rigid box of mass m
and rotational inertia I. Newton and Euler say the forces f_i at the stance feet must satisfy

    Σ f_i          = m·(a_des − g)       (total push = mass × acceleration, plus holding up the weight)
    Σ r_i × f_i    = I·ω̇_des             (total twist = inertia × angular acceleration)

where r_i points from the centre of mass to foot i. Stack this as A·f = b and pick the f that fits best,
**while making sure no foot slips**:

    minimize ‖W(A·f − b)‖² + α‖f‖²   subject to   |f_x| + |f_y| ≤ μ·f_z  for each foot

The friction rule is a pyramid that sits just inside the true friction cone. Writing each foot force as a
non-negative mix of the pyramid's four edges turns the whole problem into "least squares with all
unknowns ≥ 0", which `scipy.optimize.nnls` solves exactly.

**3. Turn forces into motor torques.** A leg is a lever. The leg's Jacobian J says how joint motion moves
the foot, and its transpose turns a foot force into joint torques:

    τ_i = J_iᵀ · (−f_i)        (the foot pushes on the ground with −f_i, so the ground pushes back with f_i)

## Design choices, and why

* **μ = 0.6 in the controller, 0.8 on the simulated feet.** It plans as if the floor were slipperier than it is,
  which leaves a safety margin.
* **A level "heading" frame.** The math only needs roll, pitch, velocity, rotation rates and height. It never
  needs the robot's compass heading or position, which the state estimator can't measure.
* **Swing legs stay position-controlled** (a joint spring of 100 N·m/rad), because moving a foot through the
  air is a path-following job, not a force job.
* **Stance comes from the gait clock,** not from sensing touchdown. That keeps it simple, but it's a limitation.
* **Integral terms were the only tuning.** Without them the robot walked at 0.11 m/s when told 0.4 m/s: the
  model ignores the legs' own weight, inertia and joint friction, and those soaked up the push.

## Results (60 s walk, 5 sensor-noise seeds, in simulation)

| | Scripted, fed truth | QP, fed truth | Scripted, fed estimate | QP, fed estimate |
|---|---|---|---|---|
| Falls | 0 / 5 | 0 / 5 | 0 / 5 | **1 / 5** |
| Velocity tracking RMSE | 0.169 m/s | 0.091 m/s | 0.173 m/s | 0.086 m/s * |
| Yaw-rate tracking RMSE | 0.403 rad/s | 0.196 rad/s | 0.406 rad/s | 0.199 rad/s * |
| Lean on the first straight | 1.15° | 2.69° | 4.1° | 2.97° * |
| Estimator velocity RMSE | 0.012 m/s | 0.016 m/s | 0.025 m/s | 0.018 m/s * |
| Falls after a 4 s turn at startup | 0 / 5 | 0 / 5 | 0 / 5 | 0 / 5 |
| Largest sideways push survived (0.1 s) | 11.9 N·s | 8.9 N·s | 11.9 N·s | 8.9 N·s |

\* Mean of the four seeds that didn't fall.

## What to take away

* **Better tracking.** Velocity tracking error roughly halves (0.169 → 0.091 m/s) and so does yaw-rate error
  (0.403 → 0.196 rad/s). It still runs about 0.1 m/s slow on the straights, because the integral term hits its limit.
* **Worse in two ways.** It leans more (about 2.7°, mostly in pitch) and survives smaller pushes (8.9 vs 11.9 N·s).
* **One fall, explained.** On one noise seed the estimator starts about 3° wrong about which way is level,
  and walking straight gives it no way to notice. The QP controller trusts that wrong "level" more forcefully
  than the scripted trot does, tilts the real robot, and falls at 7 s. A 4 s left/right turn at startup lets
  the estimator correct itself, and then all five seeds walk.
* **Limits.** It's Python, it plans only for the current instant (no look-ahead or MPC), it treats the robot as
  one rigid box (the legs are about 60 % of the mass), and it uses the gait clock for stance.

More detail: README section "QP force-balance controller", DEBUG_LOG #15 and #16.
