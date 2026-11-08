"""Module 8: safety: latching faults and the watchdog.

  1. Do WORKSHEET.md, Module 8, on paper first.
  2. Fill in the TODOs below (about 13 lines in total).
  3. Run:  python check.py 8      then:  python run.py safety
"""
import numpy as np

import m6_speed_loop


def held_for(ticks, cond, ms, fs=1000):
    """One tick of a persistence timer (orthosis_core.h, Controller::timer, line 806).

    ticks counts how many ticks in a row `cond` has been true (any false tick resets it to 0).
    Returns (new ticks, tripped): tripped once cond has held for at least `ms` milliseconds.
    With ms = 0 a single true tick trips it.   (Worksheet 8.2)"""
    # TODO (2 lines)
    raise NotImplementedError


class Supervisor:
    """A small version of the Controller's state machine: IDLE -> ARMED -> FAULT, and FAULT LATCHES.

    monitors: {name: ms}, how long each condition must hold before it trips.
    """

    def __init__(self, monitors, fs=1000):
        self.monitors, self.fs = dict(monitors), fs
        self.ticks = {name: 0 for name in self.monitors}
        self.state, self.reason = "IDLE", None

    def arm(self):
        """Only from IDLE (Controller::armBlocker: "can only arm from IDLE")."""
        if self.state == "IDLE":
            self.state = "ARMED"
            self.ticks = {name: 0 for name in self.monitors}

    def step(self, conditions):
        """One 1 ms tick. conditions: {name: bool} for this tick. Returns True if the motor may be
        driven, False to coast (driver enable off).

        Unless ARMED, return False. When ARMED, run every monitor's held_for. The first one that trips
        moves the state to FAULT and records its name in self.reason; return False. A FAULT stays a
        FAULT however the conditions change: only clear() leaves it.   (Worksheet 8.3)"""
        # TODO (about 8 lines)
        raise NotImplementedError

    def clear(self):
        """The operator's 'd' (disarm): FAULT -> IDLE, never straight back to ARMED. Arming again is a
        separate, deliberate step that re-checks everything (Controller::handleRequest, Disarm)."""
        # TODO (2 lines)
        raise NotImplementedError


class Watchdog:
    """A timer OUTSIDE the loop it watches: the loop feeds it every tick; if feeding stops for
    timeout_ms, it fires (on the ESP32-S3 it resets the chip; emg_orthosis.ino, line 335)."""

    def __init__(self, timeout_ms=200.0):
        self.timeout_ms, self.last_feed_ms = timeout_ms, 0.0

    def feed(self, t_ms):
        self.last_feed_ms = t_ms

    def expired(self, t_ms):
        """True once more than timeout_ms has passed since the last feed.   (Worksheet 8.4)"""
        # TODO (1 line)
        raise NotImplementedError


# ------------------------------------------------------------------ provided from here down
# The real monitors, in the order the firmware checks them while ARMED
# (firmware/emg_orthosis/orthosis_core.h: supervise() line 669, superviseMotion() line 685).
# (name, what it watches, ms it must hold, line)
REAL_MONITORS = [
    ("MOT_OK", "the driver's own fault line goes low", 0, 670),
    ("encoder errors", "more than 20 illegal quadrature transitions since arming", 0, 671),
    ("lead-off", "an EMG channel railed for 20 samples, or 20 % of the last 100", 0, 673),
    ("artefact", "an envelope above 2x the calibrated MVC", 20, 676),
    ("over-travel", "the joint 3 deg past a soft limit", 0, 677),
    ("overspeed", "joint speed above 1.5 x 25 = 37.5 deg/s", 50, 679),
    ("stall", "commanding motion with |duty| > 0.30 while the joint moves < 1.5 deg/s", 400, 687),
    ("runaway", "|duty| > 0.25 while the joint moves the OTHER way faster than 5 deg/s", 60, 692),
    ("tracking", "speed more than 20 deg/s away from the setpoint", 500, 695),
    ("continuous motion", "motion commanded without a break", 15000, 697),
]

# The ones the toy arm can produce, with the firmware's thresholds
TOY_MONITORS = {"overspeed": 50, "stall": 400, "runaway": 60, "tracking": 500}


def toy_conditions(ref, vel, duty):
    """The four motion monitors' conditions on one tick, with the firmware's thresholds."""
    return {"overspeed": abs(vel) > 37.5,
            "stall": abs(ref) > 2.0 and abs(duty) > 0.30 and abs(vel) < 1.5,
            "runaway": abs(duty) > 0.25 and duty * vel < 0 and abs(vel) > 5.0,
            "tracking": abs(ref - vel) > 20.0}


def scenario(kind, seconds=2.0, dt=1e-3):
    """Run the module-6 loop on the toy arm under the supervisor, with one thing going wrong:
      "obstruction": something blocks the joint at t = 0.5 s while it moves at 10 deg/s
      "reversed encoder": the encoder is wired backwards, so the loop sees -speed
    Returns (time it faulted, the reason, joint travel in deg, ms between the problem and the fault)."""
    arm = m6_speed_loop.ToyArm(20.0)
    ctrl = m6_speed_loop.VelocityPI()
    ramp = m6_speed_loop.Ramp(150.0)
    sup = Supervisor(TOY_MONITORS)
    sup.arm()
    start = arm.theta
    problem_at = 0.5 if kind == "obstruction" else 0.0
    for k in range(int(seconds / dt)):
        t = k * dt
        if kind == "obstruction" and t >= problem_at:
            arm.blocked = True
        vel = -arm.omega if kind == "reversed encoder" else arm.omega
        ref, ramping = ramp.step(10.0, dt)
        duty = ctrl.step(ref, vel, dt, ramping)
        if not sup.step(toy_conditions(ref, vel, duty)):
            return t, sup.reason, arm.theta - start, 1e3 * (t - problem_at)
        arm.step(duty, dt)
    return None, None, arm.theta - start, None


def hung_loop(hang_at_ms=1000, timeout_ms=200):
    """The control loop stops at hang_at_ms (stuck in a bad sensor read, say). A check INSIDE the loop
    never runs again; the watchdog outside it does. Returns (ms when the watchdog fires, whether the
    in-loop check ever fired)."""
    wd = Watchdog(timeout_ms)
    in_loop_fired = False
    for t in range(0, 3000):
        if t < hang_at_ms:                    # the loop is alive: it checks itself and feeds the dog
            wd.feed(t)
            in_loop_fired = in_loop_fired or (t - wd.last_feed_ms > timeout_ms)
        if wd.expired(t):
            return t, in_loop_fired
    return None, in_loop_fired
