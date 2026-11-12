"""Module 12: the QP force-balance controller (your qp_balance.py).

  1. Do WORKSHEET.md, Module 12 on paper first.
  2. Fill in the four TODOs below (about 25 lines). joint_torques uses your Module 6 foot_jacobian.
  3. Run:  python check.py 12   then:  python m12_force_balance.py

The scripted trot (Module 8) moves every foot along a path and lets stiff servos follow it. This
controller instead decides how hard each stance foot pushes on the ground:
  1. what motion do I want?     PD/PI on the body state -> desired acceleration and angular acceleration
  2. what forces give it?       one rigid body: sum f = m (a - g),  sum r x f = I wdot   ->  A f = b
  3. best forces that are legal  least squares, feet can only push and mustn't slip (friction pyramid)
  4. what the motors do         tau = J^T (-f)
Swing legs keep Module 8's trajectory, including Raibert. Same maths as python/qeskf/qp_balance.py.
"""
from dataclasses import dataclass

import numpy as np

from legs import leg_ik
from m6_legs import foot_jacobian, foot_position
from so3 import exp_so3, rpy_from_rot, skew

G_W = np.array([0.0, 0.0, -9.80665])
# The Go1 as one rigid body, from the MuJoCo model at the standing keyframe (qp_balance.rigid_body_params)
MASS = 12.743448                                    # kg, whole robot
INERTIA_B = np.array([[0.1268389934214804, -0.0004377505643336533, -0.01432617764520657],
                      [-0.0004377505643336533, 0.3940320309748341, -9.140054498162244e-05],
                      [-0.01432617764520657, -9.140054498162244e-05, 0.4228811022905318]])
                                                    # kg m^2, about the centre of mass, body axes
COM_B = np.array([-0.0021129496223712984, 0.0008767780006516311, -0.018991707124520363])
                                                    # m, centre of mass in the trunk frame
H_DES = 0.27                                        # m, trunk height command (gait.py)
STAND_Q = np.array([0.0, 0.9, -1.8])                # rad, each leg at the standing keyframe


@dataclass
class Gains:
    """qp_balance.QPGains, the values you used."""
    kv: float = 4.0          # 1/s, horizontal velocity
    kpz: float = 100.0       # 1/s^2, height
    kdz: float = 20.0        # 1/s
    kr: float = 150.0        # 1/s^2, roll/pitch
    kdr: float = 20.0        # 1/s
    kdy: float = 10.0        # 1/s, yaw rate
    ki_v: float = 4.0        # 1/s^2, integral on horizontal velocity error (DEBUG_LOG #15)
    ki_wz: float = 10.0      # 1/s^2, integral on yaw-rate error
    i_max: float = 0.5       # clamp on each integrator
    mu: float = 0.6          # friction the controller assumes (the feet really have 0.8)
    weights: tuple = (1.0, 1.0, 5.0, 10.0, 10.0, 2.0)   # how much each row of A f = b matters
    alpha: float = 1e-4      # small penalty on force size


GAINS = Gains()


# ---------------------------------------------------------------- provided helpers
def rot_rp(roll, pitch):
    """(Provided.) Body -> level 'heading' frame (z up, x along the heading): pitch after roll, no yaw."""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Ry @ Rx


def cone_edges(mu):
    """(Provided.) The four edges of the friction pyramid, as columns (3 x 4).

    Any force f = V beta with beta >= 0 has |f_x| + |f_y| <= mu f_z and f_z >= 0: it pushes and doesn't slip.
    """
    return np.array([[mu, -mu, 0, 0], [0, 0, mu, -mu], [1, 1, 1, 1.0]])


def nnls(A, b):
    """(Provided.) min ||A x - b|| subject to x >= 0, by Lawson-Hanson (what scipy.optimize.nnls does).

    Plain idea: start with every x at 0. Repeatedly free the variable that would reduce the error most,
    solve ordinary least squares on the free ones, and if any of them went negative, step back until it
    hits 0 and lock it again. It stops when no locked variable would help.
    """
    m, n = A.shape
    x = np.zeros(n)
    free = np.zeros(n, bool)
    tol = 10 * np.finfo(float).eps * np.abs(A).sum(0).max() * max(m, n)
    for _ in range(3 * n):
        w = A.T @ (b - A @ x)                       # which way each variable would like to move
        if free.all() or w[~free].max() <= tol:
            break
        free[np.argmax(np.where(free, -np.inf, w))] = True
        while True:
            s = np.zeros(n)
            s[free] = np.linalg.lstsq(A[:, free], b, rcond=None)[0]
            if s[free].min() > 0:
                break
            bad = free & (s <= 0)
            step = np.min(x[bad] / (x[bad] - s[bad]))
            x = x + step * (s - x)
            free &= x > tol
            x[~free] = 0.0
        x = s
    return x


# ---------------------------------------------------------------- 1. what motion do I want?
def desired_accel(cmd, v, rpy, w, h, i_err, h_des=H_DES, g=GAINS):
    """Desired CoM acceleration and angular acceleration, both in the heading frame (worksheet 12.1).

    cmd = (vx, vy, wz) commanded; v, w = body velocity and angular rate in the heading frame;
    rpy = (roll, pitch, yaw); h = trunk height; i_err = integrated (vx, vy, wz) error.
    """
    # TODO (about 6 lines), worksheet 12.1. Same gains as Module 8's levelling loop.
    #   err    = [cmd vx - v x, cmd vy - v y, cmd wz - w z]
    #   a_des  = [kv err_x + ki_v i_x,  kv err_y + ki_v i_y,  kpz (h_des - h) - kdz v_z]
    #   wd_des = [-kr roll - kdr w_x,  -kr pitch - kdr w_y,  kdy err_z + ki_wz i_z]
    # return a_des, wd_des   (gains are g.kv, g.ki_v, ...)
    raise NotImplementedError


# ---------------------------------------------------------------- 2. what forces give that motion?
def build_A_b(r, a_des, wd_des, mass=MASS, inertia=INERTIA_B):
    """The rigid-body equations as A f = b, with f = all stance-foot forces stacked (worksheet 12.2).

    r: (n, 3) stance feet relative to the CoM; inertia already in the same frame as r.
    Rows 0-2: sum f_i = m (a_des - g).   Rows 3-5: sum r_i x f_i = I wd_des.
    """
    # TODO (about 7 lines), worksheet 12.2.
    #   A is 6 x 3n. For foot j, its 3 columns are: rows 0-2 the identity, rows 3-5 skew(r[j]) (r x f = skew(r) f).
    #   b = [mass (a_des - G_W),  inertia @ wd_des]   (6 numbers; note MINUS gravity: G_W points down)
    raise NotImplementedError


# ---------------------------------------------------------------- 3. the best legal forces
def solve_forces(A, b, g=GAINS):
    """Foot forces (n, 3) that best satisfy A f = b, pushing only, inside the friction pyramid (worksheet 12.3).

    Write f = V beta with beta >= 0 (V = cone_edges for every foot), then
    min  sum_k weights_k (A V beta - b)_k^2  +  alpha ||beta||^2   over beta >= 0,  which nnls solves.
    """
    # TODO (about 7 lines), worksheet 12.3.
    #   1. Vb = np.kron(np.eye(n), cone_edges(g.mu))      one pyramid per foot, 3n x 4n
    #   2. W = sqrt(weights). Stack  [W * (A @ Vb)]   over   [sqrt(alpha) * identity(4n)]   (rows: 6 + 4n)
    #      and the right side       [W * b]          over   [zeros(4n)]
    #      (W * M for a matrix means each row k times W[k]: W[:, None] * M)
    #   3. beta = nnls(lhs, rhs);  return (Vb @ beta) reshaped to (n, 3)
    raise NotImplementedError


# ---------------------------------------------------------------- 4. what the motors do
def joint_torques(leg, q, f_heading, RH):
    """Joint torques (3,) that make this foot push the ground with -f, so the ground pushes back with f
    (worksheet 12.4). f is in the heading frame; the leg Jacobian is in the body frame."""
    # TODO (2 lines), worksheet 12.4. Rotate f into the body frame (RH.T @ f), then tau = J^T (-f_body)
    # with J = foot_jacobian(leg, q) from your Module 6.
    raise NotImplementedError


# ---------------------------------------------------------------- provided: one 500 Hz control step
def plan(stance, q, fb, cmd, i_err, g=GAINS):
    """(Provided.) qp_balance.QPBalanceController.plan for the stance legs, built from your four functions.

    stance: (4,) bool; q: (12,) joint angles; fb = (v_body, rpy, w_body, height) from the estimator.
    Returns forces (4, 3) in the heading frame (zero for swing legs) and stance torques (12,).
    """
    v_b, rpy, w_b, h = fb
    RH = rot_rp(rpy[0], rpy[1])
    v, w = RH @ np.asarray(v_b), RH @ np.asarray(w_b)
    a_des, wd_des = desired_accel(cmd, v, rpy, w, h, i_err, g=g)
    legs = np.flatnonzero(stance)
    r = np.array([RH @ (foot_position(leg, q[3 * leg:3 * leg + 3]) - COM_B) for leg in legs])
    A, b = build_A_b(r, a_des, wd_des, MASS, RH @ INERTIA_B @ RH.T)
    f = np.zeros((4, 3))
    tau = np.zeros(12)
    f[legs] = solve_forces(A, b, g)
    for leg in legs:
        tau[3 * leg:3 * leg + 3] = joint_torques(leg, q[3 * leg:3 * leg + 3], f[leg], RH)
    return f, tau


# ---------------------------------------------------------------- provided: a single-rigid-body toy
def simulate(T=3.0, cmd=(0.4, 0.0, 0.0), ki_v=GAINS.ki_v, drag=0.0, tilt_deg=(0.0, 0.0), trot=True, dt=1e-3):
    """(Provided.) The trunk as one rigid body, pushed only by your QP forces.

    Kept: the trot schedule (two diagonal feet most of the time, all four 20% of it); planted feet stay
    put on the ground; each foot touches down where Module 8 puts it (neutral point + Raibert, k = 0.06);
    the friction pyramid; full 3D rigid-body dynamics; your controller in the loop at 1 kHz.
    Dropped: the legs' own mass and motors, swing-leg reaction, servo limits, sensor noise (feedback is
    the true state), and the attitude dz (here the QP alone levels the trunk).
    drag (N s/m) is a force the controller's model doesn't know about: a stand-in for the legs' joint
    damping that DEBUG_LOG #15 found. Returns a dict of time series.
    """
    import simdata
    g = Gains(ki_v=ki_v)
    nom = np.array([foot_position(leg, STAND_Q) for leg in range(4)])      # standing feet, trunk frame
    t_half = simdata.DUTY * simdata.PERIOD / 2                              # 0.12 s: half a stance
    R = rot_rp(np.radians(tilt_deg[0]), np.radians(tilt_deg[1]))
    trunk = np.array([0.0, 0.0, H_DES])
    feet = np.array([trunk + R @ n for n in nom])
    feet[:, 2] = 0.0                                 # planted feet, world frame
    com = trunk + R @ COM_B
    v, w = np.zeros(3), np.zeros(3)                  # world frame
    i_err = np.zeros(3)
    log = {k: [] for k in ("t", "vx", "vy", "roll", "pitch", "h", "fz", "ix", "max_slip")}
    t0 = simdata.STAND if trot else 0.0
    was = np.ones(4, bool)
    for k in range(int(T / dt)):
        t = t0 + k * dt
        trunk = com - R @ COM_B
        rpy = rpy_from_rot(R)
        Rz = exp_so3(np.array([0.0, 0.0, rpy[2]]))
        vh, wh = Rz.T @ v, Rz.T @ w
        stance = simdata.scheduled_stance(t)[0] if trot else np.ones(4, bool)
        for leg in np.flatnonzero(stance & ~was):    # touchdown: Module 8's foot placement
            hip = trunk + Rz @ np.array([nom[leg, 0], nom[leg, 1], 0.0])
            step = vh[:2] * t_half + 0.06 * (vh[:2] - np.array(cmd[:2]))
            feet[leg] = hip + Rz @ np.array([step[0], step[1], 0.0])
            feet[leg, 2] = 0.0
        was = stance
        q = np.tile(STAND_Q, 4)
        for leg in np.flatnonzero(stance):
            q[3 * leg:3 * leg + 3] = leg_ik(leg, R.T @ (feet[leg] - trunk))
        fb = (R.T @ v, rpy, R.T @ w, trunk[2])
        f_h, _ = plan(stance, q, fb, cmd, i_err, g)
        err = np.array([cmd[0] - vh[0], cmd[1] - vh[1], cmd[2] - wh[2]])
        i_err = np.clip(i_err + dt * err, -g.i_max, g.i_max)
        # dynamics: forces back into the world frame, then Newton and Euler
        F, tq = np.zeros(3), np.zeros(3)
        for leg in np.flatnonzero(stance):
            Fi = Rz @ f_h[leg]
            F += Fi
            tq += np.cross(feet[leg] - com, Fi)
        acc = F / MASS + G_W - drag / MASS * np.array([v[0], v[1], 0.0])
        Iw = R @ INERTIA_B @ R.T
        wdot = np.linalg.solve(Iw, tq - np.cross(w, Iw @ w))
        v = v + acc * dt
        com = com + v * dt
        w = w + wdot * dt
        R = exp_so3(w * dt) @ R
        fs = f_h[stance]
        log["t"].append(t - t0)
        log["vx"].append(vh[0])
        log["vy"].append(vh[1])
        log["roll"].append(rpy[0])
        log["pitch"].append(rpy[1])
        log["h"].append(trunk[2])
        log["fz"].append(f_h[:, 2].copy())
        log["ix"].append(i_err[0])
        ok = fs[:, 2] > 1e-6
        log["max_slip"].append(np.max((np.abs(fs[ok, 0]) + np.abs(fs[ok, 1])) / fs[ok, 2]) if ok.any() else 0.0)
    return {k: np.array(val) for k, val in log.items()}


def demo():
    stand = np.ones(4, bool)
    q = np.tile(STAND_Q, 4)
    fb0 = (np.zeros(3), np.zeros(3), np.zeros(3), H_DES)
    f, tau = plan(stand, q, fb0, (0, 0, 0), np.zeros(3))
    print(f"1) Standing on four feet: f_z = {np.round(f[:, 2], 1)} N, sum {f[:, 2].sum():.1f} N = m g."
          " Not equal: the CoM is 2 mm behind centre")
    print(f"   FR joint torques (abduction, hip, knee) = {np.round(tau[:3], 2)} N m, from tau = J^T (-f)")
    f, _ = plan(np.array([True, False, False, True]), q, fb0, (0, 0, 0), np.zeros(3))
    print(f"2) Trot, FR + RL planted: f_z = {np.round(f[:, 2], 1)} N. Two feet carry it all")
    f, _ = plan(stand, q, fb0, (0.5, 0, 0), np.zeros(3))
    print(f"3) Told 0.5 m/s while still: a_des = kv x 0.5 = 2.0 m/s^2, so the feet push back:"
          f" sum f_x = {f[:, 0].sum():.1f} N = m a")
    f, _ = plan(stand, q, fb0, (3.0, 0, 0), np.zeros(3))
    print(f"4) Told 3 m/s (a_des 12 m/s^2): sum f_x = {f[:, 0].sum():.1f} N = mu x sum f_z ="
          f" 0.6 x {f[:, 2].sum():.1f}. Friction caps it at {f[:, 0].sum() / MASS:.1f} m/s^2")
    tilt = (np.zeros(3), np.array([0.0, np.radians(5), 0.0]), np.zeros(3), H_DES)
    f, _ = plan(stand, q, tilt, (0, 0, 0), np.zeros(3))
    print(f"5) Pitched 5 deg nose-down: front feet {np.round(f[:2, 2], 1)} N, rear {np.round(f[2:, 2], 1)} N."
          " Pushing harder at the front lifts the nose")
    s = simulate(T=1.0, cmd=(0, 0, 0), tilt_deg=(0.0, 5.0), trot=False)
    k = np.argmax(np.abs(np.degrees(s["pitch"])) < 0.5)
    print(f"6) Rigid-body toy, standing, starts 5 deg nose-down: under 0.5 deg after {s['t'][k]:.2f} s")
    print("7) Rigid-body toy trotting, told 0.4 m/s, mean speed over the last 2 of 6 s:")
    for drag, ki in ((0.0, 0.0), (20.0, 0.0), (20.0, 4.0)):
        s = simulate(T=6.0, drag=drag, ki_v=ki)
        print(f"   unmodelled drag {drag:2.0f} N s/m, ki_v = {ki:.0f}: {s['vx'][-2000:].mean():.3f} m/s,"
              f" worst |f_x|+|f_y| = {s['max_slip'].max():.2f} f_z (limit 0.6)")


if __name__ == "__main__":
    demo()
