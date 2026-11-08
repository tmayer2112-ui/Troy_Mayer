"""Module 6: the motor speed loop.

  1. Do WORKSHEET.md, Module 6, on paper first.
  2. Fill in the TODOs below (about 18 lines in total).
  3. Run:  python check.py 6      then:  python run.py loop
Units: speed in joint deg/s, angle in deg, motor command in duty (-1..1), time in s.
"""
import math

import numpy as np

# The controller's model and settings: the firmware's defaults (orthosis_core.h, Config).
K_MODEL = 187.0          # deg/s per unit duty: the datasheet's no-load speed through the transmission
TAU_MODEL = 0.020        # s: mechanical time constant J R / (Kt Ke)
FRICTION_FF = 0.05       # duty the feed-forward adds to break friction
LAMBDA = 0.050           # s: closed-loop time constant, conservative tier
DUTY_MAX = 0.60          # hard ceiling on |duty|


def imc_gains(K, tau, lam):
    """PI gains by IMC for the plant K / (tau s + 1): cancel the plant's pole (Ti = tau) and put the
    closed loop at 1 / (lam s + 1). Returns (Kp in duty per deg/s, Ki in duty per deg). (Worksheet 6.2)"""
    # TODO (1 line)
    raise NotImplementedError


def feedforward(ref, dref, K=K_MODEL, tau=TAU_MODEL, friction=FRICTION_FF):
    """Plant inversion: the duty that would give speed `ref`, accelerating at `dref`, if the model
    were exact:  (ref + tau dref) / K,  plus `friction` in the direction of motion when |ref| > 0.5.
    (Worksheet 6.1)"""
    # TODO (3 lines). math.copysign(friction, ref) is friction with ref's sign.
    raise NotImplementedError


class VelocityPI:
    """Feed-forward plus PI on the speed error, with anti-windup. One call to step() per 1 ms tick.
    The same logic as VelocityPI in firmware/emg_orthosis/orthosis_core.h, line by line."""

    def __init__(self, K=K_MODEL, tau=TAU_MODEL, lam=LAMBDA, friction=FRICTION_FF, duty_max=DUTY_MAX,
                 closed_loop=True, antiwindup=True):
        self.K, self.tau, self.friction, self.duty_max = K, tau, friction, duty_max
        self.kp, self.ki = imc_gains(K, tau, lam) if closed_loop else (0.0, 0.0)
        self.antiwindup = antiwindup
        self.integ = 0.0          # the integral term, already in duty
        self.prev_ref = 0.0

    def step(self, ref, meas, dt, ramping=False):
        """ref and meas in deg/s. ramping: the setpoint is being acceleration-limited this tick.
        Returns the duty, clamped to +/- duty_max.

          e   = ref - meas;   dref = (ref - previous ref) / dt
          out = clamp(feedforward + Kp e + integ)
          integrate (integ += Ki e dt) UNLESS
            - the output is saturated and e pushes the same way (anti-windup, part 1), or
            - the setpoint is ramping and e asks for more in the direction of motion (part 2:
              tracking a ramp is the feed-forward's job)
          then clamp integ to +/- duty_max.   (Worksheet 6.3)
        Part 1 and part 2 apply only when self.antiwindup is True.
        """
        # TODO (about 10 lines). np.clip(v, lo, hi) clamps a number.
        raise NotImplementedError


def count_speed(counts, dt):
    """The old firmware's speed: encoder counts read every dt seconds, differenced. counts/s.
    (Worksheet 6.5)"""
    # TODO (1 line): np.diff
    raise NotImplementedError


def edge_speed(t_edges, k=4):
    """The firmware's speed (orthosis_core.h, VelocityEstimator): k counts divided by the time they took,
    from the newest edge back k edges. t_edges = every edge time so far, oldest first. counts/s.
    k = 4 is one full quadrature cycle, so the uneven spacing of the 4 edges cancels.   (Worksheet 6.5)"""
    # TODO (1 line)
    raise NotImplementedError


# ------------------------------------------------------------------ provided from here down
# The toy arm: the firmware's plant (firmware/sim/plant.h) cut down to one first-order equation.
#   tau d(omega)/dt = K (duty - gravity - friction) - omega
K_PLANT = 192.6          # 12 V / Ke = 645 rad/s at the motor, / 191.95 = 192.6 deg/s per duty at the joint
TAU_PLANT = 0.0218       # J R / (Kt Ke) with the forearm reflected through 192:1 (plant.h, J())
FRICTION_PLANT = 0.027   # duty: what 'j' measures on the firmware's plant (firmware/README.md)
# 3.5 N m of forearm at the joint / 191.95, divided by the 0.87 x 0.90 gearbox and screw efficiency,
# is 0.0233 N m at the motor. Holding that at stall takes 0.0233 / Ke x R / 12 V = 0.136 duty.
GRAVITY_DUTY = 0.136
ELBOW_AT_ZERO = 45.0     # deg: the elbow angle when the joint reads 0 (plant.h)


class ToyArm:
    """Duty in, joint speed and angle out. Coulomb friction holds it still until the drive beats it."""

    def __init__(self, theta=5.0, K=K_PLANT, tau=TAU_PLANT, friction=FRICTION_PLANT, gravity=GRAVITY_DUTY):
        self.theta, self.omega = theta, 0.0
        self.K, self.tau, self.friction, self.gravity = K, tau, friction, gravity
        self.blocked = False      # True: something holds the joint (for the windup demo)

    def step(self, duty, dt, substeps=10):
        h = dt / substeps
        for _ in range(substeps):
            if self.blocked:
                self.omega = 0.0
                continue
            drive = duty - self.gravity * math.sin(math.radians(self.theta + ELBOW_AT_ZERO))
            if self.omega == 0.0 and abs(drive) <= self.friction:
                continue                                   # stuck: friction wins
            direction = math.copysign(1.0, self.omega if self.omega != 0.0 else drive)
            new = self.omega + h / self.tau * (self.K * (drive - self.friction * direction) - self.omega)
            self.omega = 0.0 if new * direction < 0 else new  # friction stops it; it can't push it back
            self.theta += self.omega * h


class Ramp:
    """The setpoint shaper's acceleration limit (orthosis_core.h, SetpointShaper): the speed
    reference moves toward the command by at most accel * dt per tick."""

    def __init__(self, accel=150.0):
        self.accel, self.ref = accel, 0.0

    def step(self, command, dt):
        max_step = self.accel * dt
        ramping = abs(command - self.ref) > max_step
        self.ref += float(np.clip(command - self.ref, -max_step, max_step))
        return self.ref, ramping


def simulate(ctrl, arm, command, seconds, dt=1e-3, accel=150.0, release_at=None):
    """Run the 1 kHz loop. command(t) gives the speed command in deg/s. Returns arrays t, ref,
    omega, theta, duty, integ. release_at: unblock the arm at this time (the windup demo)."""
    ramp = Ramp(accel) if accel else None
    n = int(round(seconds / dt))
    log = {k: np.zeros(n) for k in ("t", "ref", "omega", "theta", "duty", "integ")}
    for k in range(n):
        t = k * dt
        if release_at is not None and t >= release_at:
            arm.blocked = False
        ref, ramping = ramp.step(command(t), dt) if ramp else (command(t), False)
        duty = ctrl.step(ref, arm.omega, dt, ramping)
        for key, v in (("t", t), ("ref", ref), ("omega", arm.omega), ("theta", arm.theta),
                       ("duty", duty), ("integ", ctrl.integ)):
            log[key][k] = v
        arm.step(duty, dt)
    return log


def lift(closed_loop, speed=15.0, seconds=1.5):
    """Lift the forearm from 5 deg at `speed`. Returns (log, mean speed error over the last 0.5 s),
    measured the way firmware/sim/demo.cpp measures lift_15dps_steady_error."""
    log = simulate(VelocityPI(closed_loop=closed_loop), ToyArm(5.0), lambda t: speed, seconds)
    return log, float(np.mean(speed - log["omega"][-500:]))


def hold(closed_loop, theta=40.0, seconds=5.5):
    """Zero speed command at `theta`. Returns (log, drift over the last 5 s after 0.5 s to settle)."""
    log = simulate(VelocityPI(closed_loop=closed_loop), ToyArm(theta), lambda t: 0.0, seconds)
    return log, float(log["theta"][-1] - log["theta"][500])


def pi_only_step(lam=LAMBDA, speed=20.0, seconds=0.4):
    """The IMC promise, checked: model = plant exactly, no gravity, no friction, no feed-forward,
    no ramp. The closed loop should then be 1 / (lam s + 1): 63 % of the step after lam seconds."""
    ctrl = VelocityPI(lam=lam, friction=0.0)
    ctrl.K = math.inf                              # (ref + tau dref) / inf = 0: no feed-forward
    arm = ToyArm(0.0, K=K_MODEL, tau=TAU_MODEL, friction=0.0, gravity=0.0)
    log = simulate(ctrl, arm, lambda t: speed, seconds, accel=None)
    t63 = float(log["t"][np.argmax(log["omega"] >= 0.632 * speed)])
    return log, t63


def windup(antiwindup, block_s=0.5, seconds=1.5, speed=20.0):
    """Command 20 deg/s while something holds the joint for 0.5 s, then let go. Returns
    (log, peak speed after release, integral term at release)."""
    arm = ToyArm(20.0)
    arm.blocked = True
    log = simulate(VelocityPI(antiwindup=antiwindup), arm, lambda t: speed, seconds, release_at=block_s)
    after = log["t"] >= block_s
    return log, float(log["omega"][after].max()), float(log["integ"][np.argmax(after)])


# The encoder (firmware/sim/plant.h): 28 counts per motor turn, 0.06698 joint deg per count, and the
# four edges of each quadrature cycle are not evenly spaced (edge_err, a fraction of a count). The ISR
# timestamps each edge in whole microseconds.
DEG_PER_COUNT = 0.06698          # firmware/results/metrics.json, deg_per_count
EDGE_ERR = (0.0, 0.12, 0.0, -0.10)


def encoder_edges(dps, seconds=2.0):
    """Edge times (s) for a joint turning at a constant dps: edge n sits at count n + EDGE_ERR[n % 4]."""
    cps = dps / DEG_PER_COUNT
    n = np.arange(1, int(cps * seconds))
    return np.round((n + np.array(EDGE_ERR)[n % 4]) / cps * 1e6) * 1e-6


def speed_errors(dps, tick=0.01, seconds=2.0):
    """RMS error, as a fraction of the true speed, of (count difference every `tick`, edge timing read
    every 1 ms), from the 5th edge on. The firmware's test VelocityEstimator.EdgeTimingBeatsCountDifference."""
    cps = dps / DEG_PER_COUNT
    edges = encoder_edges(dps, seconds)
    t = np.arange(edges[4], seconds, tick)
    counts = np.searchsorted(edges, t, side="right")
    by_count = count_speed(counts, tick)
    t_ms = np.arange(edges[4], seconds, 0.001)
    by_edge = np.array([edge_speed(edges[:np.searchsorted(edges, tt, side="right")]) for tt in t_ms])
    rms = lambda v: float(np.sqrt(np.mean((v / cps - 1.0) ** 2)))
    return rms(by_count), rms(by_edge)
