"""Module 8: the walking controller.

  1. Do WORKSHEET.md, Module 8 on paper first.
  2. Fill in the seven TODOs below (about 25 lines).
  3. Run:  python check.py 8      then:  python m8_walking_control.py

Four pieces, each the core of something in your controller (python/qeskf/gait.py):
  A. the gait clock: which feet are planted when          (gait.py: leg_phase, scheduled_stance)
  B. Raibert foot placement: where a foot lands sets the speed      (gait.py, line 96)
  C. levelling loops: PD / PI on an angle, fed a measurement that may be wrong
                                                                     (gait.py lines 69-76; qp_balance.py)
  D. your real trot: every foot target, exactly as gait.py computes it      (gait.py lines 59-102)
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


# ---------------------------------------------------------------- D. your real trot: foot targets (gait.py lines 59-102)
from legs import HIP_OFFSETS, SIDE, THIGH_Y  # noqa: E402

T_STANCE = PERIOD * DUTY      # 0.24 s: how long each foot is planted
SWING_HEIGHT = 0.07           # m
K_RAIBERT = 0.06              # s, gait.py raibert_k
K_ATT, D_ATT = 1.2, 0.06      # gait.py k_att, d_att: leg-length PD on roll and pitch
NOMINAL_HEIGHT = 0.27         # m


def nominal_foot(leg):
    """(Provided.) Where foot `leg` sits under its hip, body frame (z = 0: height is added separately)."""
    return HIP_OFFSETS[leg] + np.array([0.0, SIDE[leg] * THIGH_Y, 0.0])


def _rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


def attitude_dz(nom, rpy, omega):
    """How much to raise (+) or lower (-) this foot's target to level the trunk: a PD on roll and pitch,
    scaled by how far the foot is from the centre (its lever arm)."""
    # TODO (1 line), worksheet 8.4e: K_ATT (roll * y - pitch * x) + D_ATT (omega_x * y - omega_y * x),
    # with x, y = nom[0], nom[1]
    raise NotImplementedError


def stance_xy(nom, s, cmd):
    """Where a planted foot is in the body frame at stance progress s (0 = touchdown, 1 = liftoff).
    The foot is fixed in the world, so it sweeps backwards at the commanded speed and turns against
    the commanded yaw rate. cmd = (vx, vy, wz)."""
    # TODO (2 lines), worksheet 8.4a: tau = time from mid-stance = (s - 0.5) * T_STANCE;
    # the nominal xy turned by -wz*tau (use _rot2), minus the commanded (vx, vy) times tau
    raise NotImplementedError


def foot_target(leg, phase, cmd, v_body, rpy, omega, h_cmd=NOMINAL_HEIGHT):
    """Body-frame target for one foot while walking: the same maths as gait.py foot_targets.
    Stance: sweep backwards. Swing: from where stance ended to the next touchdown point (which slides
    backwards, ground-speed matching) plus the Raibert correction, lifted on a raised cosine."""
    # TODO (about 14 lines), worksheet 8.4. In stance (phase < DUTY): xy = stance_xy at s = phase / DUTY,
    #   z = -h_cmd + attitude_dz.  In swing: s = (phase - DUTY) / (1 - DUTY);
    #   start = stance_xy(s = 1);  s_early = -(1 - s)(1 - DUTY)/DUTY;
    #   end = stance_xy(s_early) + K_RAIBERT * (v_body xy - commanded xy)          <- Raibert
    #   blend b = _smoothstep(min(s / 0.8, 1));  xy = (1 - b) start + b end
    #   z = -h_cmd + dz + SWING_HEIGHT * 0.5 * (1 - cos(2 pi s))
    raise NotImplementedError


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

    print("\nD) Your real trot (gait.py foot_targets): the front-right foot over one cycle at 0.5 m/s")
    print("   phase   part     x - nominal (mm)   z (mm)")
    nom = nominal_foot(0)
    for ph in (0.0, 0.3, 0.599, 0.7, 0.9, 0.999):
        tg = foot_target(0, ph, (0.5, 0.0, 0.0), np.array([0.5, 0, 0]), np.zeros(3), np.zeros(3))
        part = "stance" if ph < DUTY else "swing"
        print(f"   {ph:5.3f}   {part:6s} {1000 * (tg[0] - nom[0]):+12.1f} {1000 * tg[2]:12.1f}")
    a = foot_target(0, 0.995, (0.5, 0, 0), np.array([0.5, 0, 0]), np.zeros(3), np.zeros(3))[0]
    b_ = foot_target(0, 0.999, (0.5, 0, 0), np.array([0.5, 0, 0]), np.zeros(3), np.zeros(3))[0]
    foot_speed = (b_ - a) / (0.004 * PERIOD)
    print(f"   just before touchdown the foot moves {foot_speed:+.2f} m/s in the body frame, so"
          f" {foot_speed + 0.5:+.2f} m/s over the ground (ground-speed matching: no skid)")
    fast = foot_target(0, 0.999, (0.5, 0, 0), np.array([0.6, 0, 0]), np.zeros(3), np.zeros(3))[0]
    print(f"   0.1 m/s too fast: touchdown moves {1000 * (fast - b_):+.1f} mm forward (Raibert, k = {K_RAIBERT} s)")
    roll = np.radians(2.0)
    print(f"   rolled 2 deg: left feet {1000 * attitude_dz(nominal_foot(1), (roll, 0, 0), np.zeros(3)):+.1f} mm,"
          f" right feet {1000 * attitude_dz(nominal_foot(0), (roll, 0, 0), np.zeros(3)):+.1f} mm (target height)")

if __name__ == "__main__":
    demo()
