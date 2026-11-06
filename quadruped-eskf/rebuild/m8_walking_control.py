"""Module 8: the walking controller.

  1. Do WORKSHEET.md, Module 8 on paper first.
  2. Fill in the four TODOs below (about 6 lines).
  3. Run:  python check.py 8      then:  python m8_walking_control.py

Three small pieces, each the core of something in your controller (python/qeskf/gait.py):
  A. the gait clock: which feet are planted when          (gait.py: leg_phase, scheduled_stance)
  B. Raibert foot placement: where a foot lands sets the speed      (gait.py, line 96)
  C. levelling loops: PD / PI on an angle, fed a measurement that may be wrong
                                                                     (gait.py lines 69-76; qp_balance.py)
"""
import numpy as np

G = 9.80665

# ---------------------------------------------------------------- A. the gait clock (same numbers as gait.py)
STAND = 2.0                                      # s standing before the clock starts
PERIOD = 0.4                                     # s, one trot cycle
DUTY = 0.6                                       # fraction of the cycle each foot is planted
PHASE_OFFSET = np.array([0.0, 0.5, 0.5, 0.0])    # FR, FL, RR, RL: diagonal pairs move together
LEGS = ["FR", "FL", "RR", "RL"]


def leg_phase(t):
    """Where each leg is in its cycle, 0 to 1 (0 = touchdown). The clock starts at t = STAND."""
    # TODO (2 lines), worksheet 8.1: time since the clock started (never negative: use max(..., 0.0)),
    # divided by PERIOD, plus PHASE_OFFSET, wrapped to [0, 1) with % 1.0
    raise NotImplementedError


def scheduled_stance(t):
    """(4,) bool: True for each foot the schedule has planted. All four while standing."""
    # TODO (3 lines): before STAND return np.ones(4, bool); after it, a foot is planted while its phase < DUTY
    raise NotImplementedError


# ---------------------------------------------------------------- B. Raibert foot placement on a pendulum
HEIGHT = 0.27                                    # m, trunk height (as in gait.py)
OMEGA = np.sqrt(G / HEIGHT)                      # 1/s, how fast the pendulum falls away from its foot
T_STEP = 0.2                                     # s, one support phase (half a trot cycle)


def neutral_point(v):
    """(Provided.) How far ahead of the body to put the foot so the speed comes out unchanged.

    Raibert's rule is v * T / 2: the body travels v*T during stance, so put the foot halfway along
    and the body passes over it at mid-stance. For this pendulum the exact answer is
    (v / omega) * tanh(omega * T / 2), which is 0.089 v instead of 0.1 v here.
    """
    return v / OMEGA * np.tanh(OMEGA * T_STEP / 2)


def raibert_offset(v, v_des, k):
    """Where to put the next foot, ahead of the body: the neutral point plus k times the speed error."""
    # TODO (1 line), worksheet 8.2: use neutral_point(v)
    raise NotImplementedError


def stance(x, v, foot, T=T_STEP):
    """(Provided.) One support phase of the linear inverted pendulum, x'' = omega^2 (x - foot).
    The body accelerates away from the foot: forwards if it's ahead of the foot, backwards if behind.
    Exact solution, so with k = 0 the speed is kept exactly. Returns (x, v) at the end of the step."""
    e = x - foot
    c, s = np.cosh(OMEGA * T), np.sinh(OMEGA * T)
    return foot + e * c + v / OMEGA * s, e * OMEGA * s + v * c


def walk_steps(k, n_steps=40, v0=0.0):
    """(Provided.) Take n_steps steps, placing each foot with raibert_offset.
    The command is 0.5 m/s for 20 steps, then 0.2 m/s. Returns (speed after each step, command)."""
    x, v = 0.0, v0
    vs, cmd = [], []
    for n in range(n_steps):
        v_des = 0.5 if n < 20 else 0.2
        x, v = stance(x, v, x + raibert_offset(v, v_des, k))
        vs.append(v)
        cmd.append(v_des)
    return np.array(vs), np.array(cmd)


# ---------------------------------------------------------------- C. levelling loops
def level_command(meas, rate, integral, kp, kd, ki):
    """PD (plus I) on an angle: drive the MEASURED angle to zero. Returns the commanded angular
    acceleration. Same shape as your attitude loops (qp_balance.py uses kp = 150, kd = 20)."""
    # TODO (1 line), worksheet 8.3: minus kp times the angle, minus kd times the rate, minus ki times the integral
    raise NotImplementedError


def level_loop(kp=150.0, kd=20.0, ki=0.0, meas_error=0.0, push=0.0, T=5.0, dt=1e-3):
    """(Provided.) A body whose angle obeys theta'' = u + push. The controller sees theta + meas_error
    (an estimate that is off) and the true rate (a gyro). Returns the true angle after T seconds."""
    th = w = integral = 0.0
    for _ in range(int(round(T / dt))):
        meas = th + meas_error
        integral += meas * dt
        u = level_command(meas, w, integral, kp, kd, ki)
        w += (u + push) * dt
        th += w * dt
    return th


def demo():
    print("A) The gait clock: which feet are planted (t in s; walking starts at 2.0)")
    print("      t     " + "  ".join(f"{n:>3s}" for n in LEGS))
    for t in (1.0, 2.10, 2.22, 2.25, 2.30, 2.42):
        print(f"   {t:5.2f}    " + "  ".join("  X" if s else "  ." for s in scheduled_stance(t)))
    ts = np.arange(2.0, 6.0, 0.001)
    all4 = np.mean([scheduled_stance(t).all() for t in ts])
    print(f"   all four planted {all4:.0%} of the time while walking")

    print("\nB) Raibert foot placement on a pendulum: speed after each 0.2 s step (command 0.5, then 0.2 m/s)")
    print("    k (s)   step 1  step 2  step 3  step 5  step 10  step 20  step 25  step 40")
    for k in (0.0, 0.06, 0.11, 0.25):
        v, _ = walk_steps(k)
        print(f"   {k:5.2f}  " + "".join(f"{v[i]:8.3f}" if abs(v[i]) < 10 else "    huge"
                                       for i in (0, 1, 2, 4, 9, 19, 24, 39)))
    print("   k = 0 never changes speed. 0.06 (your gait.py value) settles in a few steps. 0.11 lands in one.")
    print("   0.25 over-corrects: each step flips the error and makes it bigger.")

    print("\nC) Levelling loop, theta'' = u + push, kp = 150, kd = 20 (ki = 300 when on). Final true angle:")
    cases = [("PD,  estimate off by +1 deg", dict(meas_error=np.radians(1))),
             ("PD,  constant push 3 rad/s^2", dict(push=3.0)),
             ("PID, constant push 3 rad/s^2", dict(push=3.0, ki=300.0)),
             ("PID, estimate off by +1 deg", dict(meas_error=np.radians(1), ki=300.0))]
    for name, kw in cases:
        print(f"   {name:32s} {np.degrees(level_loop(**kw)):+7.3f} deg")
    print("   A wrong estimate becomes a real lean, and an integral can't remove it: it zeroes the MEASURED angle.")


if __name__ == "__main__":
    demo()
