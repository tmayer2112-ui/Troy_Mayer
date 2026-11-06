"""Auto-checker for the rebuild.

    python check.py 1          check Module 1 (also 2-8, 10, 11; Module 9 is C++, see README)
    python check.py all        check everything you've written so far
    python check.py 4 --solutions    run a check against the reference solutions instead of your code

A check either PASSes, FAILs with the numbers it expected, or says the TODO isn't written yet.
Modules 4 and 5 test your filter's Jacobians with YOUR fd_jacobian from Module 3, the same way the
main project's GoogleTests do (cpp/tests/test_jacobians.cpp).
"""
import importlib
import sys
import time
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
USE_SOLUTIONS = "--solutions" in sys.argv
if USE_SOLUTIONS:
    sys.path.insert(0, str(HERE / "solutions"))
sys.path.insert(1 if USE_SOLUTIONS else 0, str(HERE))

RESULTS = []


class NotWritten(Exception):
    pass


def load(name):
    return importlib.import_module(name)


def check(label):
    """Decorator: run one check, record PASS / FAIL / TODO."""
    def wrap(fn):
        def run():
            try:
                fn()
                RESULTS.append(("PASS", label, ""))
                print(f"  PASS  {label}")
            except NotImplementedError:
                RESULTS.append(("TODO", label, ""))
                print(f"  TODO  {label}  (not written yet)")
            except NotWritten as e:
                RESULTS.append(("TODO", label, str(e)))
                print(f"  TODO  {label}  ({e})")
            except AssertionError as e:
                RESULTS.append(("FAIL", label, str(e)))
                print(f"  FAIL  {label}\n        {e}")
            except Exception:
                RESULTS.append(("FAIL", label, "crashed"))
                print(f"  FAIL  {label}  (crashed)")
                print("        " + traceback.format_exc().strip().replace("\n", "\n        "))
        return run
    return wrap


def close(got, want, tol, what):
    got, want = np.asarray(got, float), np.asarray(want, float)
    assert got.shape == want.shape, f"{what}: shape {got.shape}, expected {want.shape}"
    diff = np.abs(got - want)
    err = diff.max() if got.size else 0.0
    if err <= tol:
        return
    if got.size <= 12:
        raise AssertionError(f"{what}: got {np.round(got, 6).tolist()}, expected {np.round(want, 6).tolist()}"
                             f" (max difference {err:.2e}, allowed {tol:.0e})")
    idx = np.unravel_index(np.argmax(diff), diff.shape)
    raise AssertionError(f"{what}: worst entry {list(map(int, idx))}: got {got[idx]:.6g}, expected {want[idx]:.6g}"
                         f" (difference {err:.2e}, allowed {tol:.0e})")


# ---------------------------------------------------------------------------------- Module 1
def module1():
    m = load("m1_kalman_1d")

    @check("kalman_gain(P=4, R=1) = 0.8")
    def _():
        close(m.kalman_gain(4.0, 1.0), 0.8, 1e-12, "K")

    @check("predict_1d(x=1, P=0.5, u=0.2, q=0.1) = (1.2, 0.6)")
    def _():
        close(m.predict_1d(1.0, 0.5, 0.2, 0.1), (1.2, 0.6), 1e-12, "(x, P)")

    @check("update_1d(x=10, P=4, z=12, R=1) = (11.6, 0.8)   (worksheet 1.2)")
    def _():
        close(m.update_1d(10.0, 4.0, 12.0, 1.0), (11.6, 0.8), 1e-12, "(x, P)")

    @check("an update never makes P bigger, and never moves past the measurement")
    def _():
        rng = np.random.default_rng(0)
        for _ in range(100):
            x, P, z, R = rng.normal(), rng.uniform(0.01, 5), rng.normal(), rng.uniform(0.01, 5)
            x2, P2 = m.update_1d(x, P, z, R)
            assert P2 <= P + 1e-12, f"P went from {P} to {P2}"
            assert min(x, z) - 1e-12 <= x2 <= max(x, z) + 1e-12, f"x={x}, z={z}, but the update gave {x2}"


def run_module(fn):
    """Each moduleN() defines its checks with @check; this runs them in order."""
    checks = []
    original = check

    def collecting(label):
        dec = original(label)

        def wrap(f):
            r = dec(f)
            checks.append(r)
            return r
        return wrap
    globals()["check"] = collecting
    try:
        fn()
    finally:
        globals()["check"] = original
    for c in checks:
        c()


# ---------------------------------------------------------------------------------- Module 2
def module2():
    m = load("m2_kalman_2d")

    @check("make_F(0.1) = [[1, 0.1], [0, 1]]")
    def _():
        close(m.make_F(0.1), [[1, 0.1], [0, 1]], 1e-12, "F")

    @check("predict: P = I, Q = diag(0, 0.01), dt = 0.1  ->  [[1.01, 0.1], [0.1, 1.01]]   (worksheet 2.2)")
    def _():
        x, P = m.predict(np.array([0.0, 1.0]), np.eye(2), m.make_F(0.1), np.diag([0.0, 0.01]))
        close(x, [0.1, 1.0], 1e-12, "x")
        close(P, [[1.01, 0.1], [0.1, 1.01]], 1e-12, "P")

    @check("update: position-only measurement corrects velocity too (worksheet 2.3)")
    def _():
        P = np.array([[1.01, 0.1], [0.1, 1.01]])
        x, P2, K = m.update(np.zeros(2), P, np.array([1.0]), np.array([[1.0, 0.0]]), np.array([[0.25]]))
        close(np.ravel(K), [1.01 / 1.26, 0.1 / 1.26], 1e-9, "K")
        close(x, [1.01 / 1.26, 0.1 / 1.26], 1e-9, "x after a 1 m residual")
        close(P2, [[0.2003968, 0.0198413], [0.0198413, 1.0020635]], 1e-6, "P after the update")

    @check("the cart demo: velocity is estimated to within 0.15 m/s from position alone")
    def _():
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            x_true, x = m.demo(seed=0)
        assert abs(x[1] - x_true[1]) < 0.15, f"velocity error {abs(x[1] - x_true[1]):.3f} m/s"


# ---------------------------------------------------------------------------------- Module 3
def module3():
    m = load("m3_rotations_fd")

    @check("fd_jacobian matches a Jacobian worked out by hand")
    def _():
        f = lambda x: np.array([x[0] ** 2, x[0] * x[1], np.sin(x[1])])
        x0 = np.array([3.0, 0.5])
        want = np.array([[6.0, 0.0], [0.5, 3.0], [0.0, np.cos(0.5)]])
        close(m.fd_jacobian(f, x0), want, 1e-6, "J")

    @check("correcting a quaternion by adding breaks unit length (1.005)")
    def _():
        qa = m.correct_by_adding(np.array([1.0, 0, 0, 0]), np.array([0.2, 0, 0]))
        close(qa, [1.0, 0.1, 0.0, 0.0], 1e-12, "q after adding")
        close(np.linalg.norm(qa), np.sqrt(1.01), 1e-12, "length")

    @check("correcting by multiplying keeps unit length")
    def _():
        qm = m.correct_by_multiplying(np.array([1.0, 0, 0, 0]), np.array([0.2, 0, 0]))
        close(qm, [np.cos(0.1), np.sin(0.1), 0.0, 0.0], 1e-12, "q after multiplying")

    @check("gravity leak, level robot: entry (v_x, tilt_y) = g dt = 0.00980665")
    def _():
        A = m.gravity_leak_block(np.eye(3), np.array([0.0, 0.0, 9.80665]), 1e-3)
        close(A[0, 1], 9.80665e-3, 1e-12, "A[0, 1]")
        close(A[1, 0], -9.80665e-3, 1e-12, "A[1, 0]")

    @check("gravity leak block matches finite differences for random orientations and readings")
    def _():
        from so3 import exp_so3
        rng = np.random.default_rng(1)
        for _ in range(5):
            R, f, dt = exp_so3(rng.normal(0, 1, 3)), rng.normal(0, 5, 3), 1e-3
            # independent FD here (not yours), so this checks the block itself
            J = np.zeros((3, 3))
            for j in range(3):
                e = np.zeros(3)
                e[j] = 1e-6
                J[:, j] = (m.velocity_after_one_step(e, R, f, dt) - m.velocity_after_one_step(-e, R, f, dt)) / 2e-6
            close(m.gravity_leak_block(R, f, dt), J, 1e-9, "block")


# ---------------------------------------------------------------------------------- shared
def their_fd():
    """Your fd_jacobian from Module 3 (Modules 4-5 test with it)."""
    m3 = load("m3_rotations_fd")
    try:
        m3.fd_jacobian(lambda x: x, np.zeros(1))
    except NotImplementedError:
        raise NotWritten("finish fd_jacobian in Module 3 first: this check uses it")
    return m3.fd_jacobian


def random_filter(E, seed, feet=(0, 3)):
    from so3 import quat_exp
    rng = np.random.default_rng(seed)
    f = E.Eskf(E.Params(), rng.normal(0, 1, 3), rng.normal(0, 0.5, 3), quat_exp(rng.normal(0, 0.3, 3)),
               rng.normal(0, 0.05, 3), rng.normal(0, 0.01, 3), np.eye(E.NB) * 1e-4)
    for i in feet:
        f.touchdown(i, np.array([0.2, -0.13, -0.27]) + rng.normal(0, 0.03, 3))
    acc = rng.normal(0, 3, 3) + np.array([0, 0, 9.8])
    gyro = rng.normal(0, 2, 3)
    return f, acc, gyro


# ---------------------------------------------------------------------------------- Module 4
def module4():
    E = load("eskf")
    import simdata

    @check("noise-free, bias-free IMU dead reckoning reproduces the truth for 30 s (< 1e-6 m)")
    def _():
        d = simdata.make_walk(seed=0, noise=False, biases=False)
        f = E.Eskf(E.Params(), d["p"][0], d["v"][0], d["q"][0], np.zeros(3), np.zeros(3), np.eye(E.NB))
        worst = 0.0
        for k in range(len(d["t"]) - 1):
            f.propagate_nominal(d["acc"][k], d["gyro"][k], d["dt"])
            worst = max(worst, np.max(np.abs(f.p - d["p"][k + 1])))
        assert worst < 1e-6, f"position drifted {worst:.2e} m from the truth with perfect sensors"

    @check("F matches finite differences of propagate_nominal (3 random states, tolerance 1e-8)")
    def _():
        fd = their_fd()
        for seed in range(3):
            f, acc, gyro = random_filter(E, seed)
            dt = 1e-3
            F = f.error_transition(acc, gyro, dt)
            ref = E.boxplus(f, np.zeros(E.N))
            ref.propagate_nominal(acc, gyro, dt)

            def step(dx):
                g = E.boxplus(f, dx)
                g.propagate_nominal(acc, gyro, dt)
                return E.boxminus(g, ref)
            Ffd = fd(step, np.zeros(E.N))
            err = np.abs(F - Ffd)
            i, j = np.unravel_index(np.argmax(err), err.shape)
            names = ["p", "v", "theta", "ba", "bg"] + [f"foot{n}" for n in range(4)]
            assert err.max() < 1e-8, (f"worst entry F[{i}, {j}] (row: {names[min(i // 3, 8)]}, column: "
                                      f"{names[min(j // 3, 8)]}): yours {F[i, j]:.6g}, nudged {Ffd[i, j]:.6g}")

    @check("Q has the right diagonal")
    def _():
        f, _, _ = random_filter(E, 0)
        p, dt = f.prm, 1e-3
        want = np.concatenate([np.zeros(3), [p.acc_noise ** 2 * dt] * 3, [p.gyro_noise ** 2 * dt] * 3,
                               [p.acc_rw ** 2 * dt] * 3, [p.gyro_rw ** 2 * dt] * 3, [p.foot_noise ** 2 * dt] * 12])
        Q = f.process_noise(dt)
        close(np.diag(Q), want, 1e-18, "diag(Q)")
        close(Q - np.diag(np.diag(Q)), np.zeros((E.N, E.N)), 0.0, "off-diagonal of Q")

    @check("predict: P = F P F^T + Q with F taken BEFORE the state moves, and P stays symmetric")
    def _():
        f, acc, gyro = random_filter(E, 5)
        dt = 1e-3
        F = f.error_transition(acc, gyro, dt)
        want = F @ f.P @ F.T + f.process_noise(dt)
        f.predict(acc, gyro, dt)
        close(f.P, 0.5 * (want + want.T), 1e-12, "P")
        close(f.P, f.P.T, 0.0, "P - P^T")


# ---------------------------------------------------------------------------------- Module 5
def module5():
    E = load("eskf")

    @check("measurement: residual is z - R^T (d - p)")
    def _():
        f, _, _ = random_filter(E, 0)
        z = np.array([0.15, -0.12, -0.28])
        r, H = f.measurement(0, z)
        close(r, z - f.R.T @ (f.d[0] - f.p), 1e-12, "r")
        assert H.shape == (3, E.N), f"H shape {H.shape}, expected (3, {E.N})"

    @check("H matches finite differences of the residual (tolerance 1e-8)")
    def _():
        fd = their_fd()
        for seed in range(3):
            f, _, _ = random_filter(E, seed)
            z = np.array([0.15, -0.12, -0.28])
            _, H = f.measurement(3, z)
            Hfd = -fd(lambda dx: E.boxplus(f, dx).measurement(3, z)[0], np.zeros(E.N))   # r = z - h(x)
            close(H, Hfd, 1e-8, "H")

    @check("inject: moves the state by dx, keeps q unit length, resets P only in the tilt rows/columns")
    def _():
        f, _, _ = random_filter(E, 1)
        rng = np.random.default_rng(2)
        dx = rng.normal(0, 1e-2, E.N)
        p0, v0, q0, P0 = f.p.copy(), f.v.copy(), f.q.copy(), f.P.copy()
        d0 = f.d.copy()
        f.inject(dx)
        close(f.p, p0 + dx[:3], 1e-12, "p")
        close(f.v, v0 + dx[3:6], 1e-12, "v")
        close(np.linalg.norm(f.q), 1.0, 1e-12, "|q|")
        from so3 import quat_exp, quat_mul
        qw = quat_mul(q0, quat_exp(dx[6:9]))
        close(f.q, qw / np.linalg.norm(qw), 1e-12, "q")
        close(f.d[0], d0[0] + dx[E.foot_index(0):E.foot_index(0) + 3], 1e-12, "foot 0 (active)")
        G = np.eye(E.N)
        from so3 import skew
        G[6:9, 6:9] -= 0.5 * skew(dx[6:9])
        close(f.P, G @ P0 @ G.T, 1e-12, "P after the reset")

    @check("update: P shrinks, stays symmetric and positive definite")
    def _():
        f, _, _ = random_filter(E, 3)
        f.P = f.P + np.eye(E.N) * 1e-4
        legs = np.full((4, 3), np.nan)
        legs[0] = np.array([0.18, -0.12, -0.27])
        legs[3] = np.array([0.21, -0.14, -0.26])
        tr = np.trace(f.P)
        f.update([0, 3], legs)
        assert np.trace(f.P) < tr, f"trace(P) went from {tr:.4g} to {np.trace(f.P):.4g}"
        close(f.P, f.P.T, 1e-12, "P - P^T")
        assert np.linalg.eigvalsh(f.P).min() > 0, "P has a negative eigenvalue"

    @check("full filter on the synthetic walk: velocity RMSE below 0.02 m/s (reference: about 0.004)")
    def _():
        import run
        t0 = time.time()
        res = run.run_filter(run.make_data(seed=0), E)
        assert res["vel_rmse"] < 0.02, f"velocity RMSE {res['vel_rmse']:.4f} m/s"
        print(f"        (your velocity RMSE {res['vel_rmse']:.4f} m/s, {time.time() - t0:.0f} s)")


# ---------------------------------------------------------------------------------- Module 8
def module8():
    m = load("m8_walking_control")

    @check("leg_phase 0.1 s after walking starts = [0.25, 0.75, 0.75, 0.25]   (worksheet 8.1)")
    def _():
        close(m.leg_phase(2.1), [0.25, 0.75, 0.75, 0.25], 1e-9, "phase")
        close(m.leg_phase(1.0), [0.0, 0.5, 0.5, 0.0], 1e-12, "phase before the clock starts")

    @check("scheduled_stance: one diagonal pair, then the other, all four 20% of the time")
    def _():
        want = {1.0: [1, 1, 1, 1], 2.10: [1, 0, 0, 1], 2.22: [1, 1, 1, 1], 2.30: [0, 1, 1, 0]}
        for t, w in want.items():
            close(np.asarray(m.scheduled_stance(t), float), w, 0, f"stance at t = {t}")
        import simdata
        ts = np.arange(2.0, 4.0, 0.001)
        got = np.array([m.scheduled_stance(t) for t in ts])
        close(got.astype(float), simdata.scheduled_stance(ts).astype(float), 0, "stance over 2-4 s (vs simdata)")
        close(got.all(1).mean(), 0.2, 0.002, "fraction of time with all four planted")

    @check("raibert_offset = neutral point + k (v - v_des)   (worksheet 8.2)")
    def _():
        close(m.raibert_offset(0.5, 0.5, 0.06), m.neutral_point(0.5), 1e-12, "on speed: just the neutral point")
        close(m.raibert_offset(0.6, 0.5, 0.06), m.neutral_point(0.6) + 0.006, 1e-12, "0.1 m/s too fast")

    @check("Raibert walk: k = 0.06 reaches the command, k = 0 never changes speed, k = 0.25 blows up")
    def _():
        v, cmd = m.walk_steps(0.06)
        assert abs(v[9] - 0.5) < 0.01 and abs(v[39] - 0.2) < 0.01, f"k = 0.06: speeds {v[9]:.3f}, {v[39]:.3f}"
        v0, _ = m.walk_steps(0.0)
        close(v0, np.zeros_like(v0), 1e-12, "k = 0 speeds")
        vbig, _ = m.walk_steps(0.25)
        assert abs(vbig[15] - 0.5) > 10, "k = 0.25 should over-correct more every step"

    @check("level_command is -kp*angle - kd*rate - ki*integral")
    def _():
        close(m.level_command(0.1, 0.2, 0.3, 150.0, 20.0, 300.0), -15.0 - 4.0 - 90.0, 1e-12, "u")

    @check("levelling loop: an estimate error becomes a real lean; an integral fixes a push but not that")
    def _():
        e = np.radians(1.0)
        close(m.level_loop(meas_error=e), -e, 1e-5, "PD with the estimate off by +1 deg")
        close(m.level_loop(push=3.0), 3.0 / 150.0, 1e-5, "PD with a 3 rad/s^2 push (= push / kp)")
        close(m.level_loop(push=3.0, ki=300.0), 0.0, 1e-5, "PID with the push")
        close(m.level_loop(meas_error=e, ki=300.0), -e, 1e-5, "PID with the estimate off")


# ---------------------------------------------------------------------------------- Module 6
def module6():
    m = load("m6_legs")
    import legs

    @check("foot_position of a straight leg = hip + (0, -0.08, -0.426)   (worksheet 6.1)")
    def _():
        close(m.foot_position(0, np.zeros(3)), [0.1881, -0.12675, -0.426], 1e-12, "FR foot, all joints 0")
        close(m.foot_position(1, np.zeros(3)), [0.1881, 0.12675, -0.426], 1e-12, "FL foot, all joints 0")

    @check("foot_position undoes the provided inverse kinematics (all legs, random poses)")
    def _():
        rng = np.random.default_rng(0)
        for _ in range(40):
            leg = int(rng.integers(4))
            p = legs.HIP_OFFSETS[leg] + np.array([rng.uniform(-0.08, 0.08), legs.SIDE[leg] * 0.08 + rng.uniform(-0.03, 0.03),
                                                  rng.uniform(-0.33, -0.22)])
            close(m.foot_position(leg, legs.leg_ik(leg, p)), p, 1e-9, f"FK(IK(p)) for leg {leg}")

    @check("foot_jacobian matches finite differences of your foot_position (tolerance 1e-8)")
    def _():
        fd = their_fd()
        rng = np.random.default_rng(1)
        for leg in range(4):
            q = np.array([0.0, 0.8, -1.6]) + rng.normal(0, 0.2, 3)
            close(m.foot_jacobian(leg, q), fd(lambda qq: m.foot_position(leg, qq), q), 1e-8, f"J for leg {leg}")

    @check("leg_noise_cov = J (step^2/12) J^T + model_noise^2 I; 14-bit rounding is far below 3 mm")
    def _():
        q = np.array([0.05, 0.75, -1.55])
        J = m.foot_jacobian(2, q)
        want = J @ J.T * legs.encoder_step(10) ** 2 / 12 + np.eye(3) * 0.002 ** 2
        close(m.leg_noise_cov(2, q, bits=10, model_noise=0.002), want, 1e-15, "covariance")
        enc = m.leg_noise_cov(2, q, bits=14, model_noise=0.0)
        assert np.sqrt(np.linalg.eigvalsh(enc).max()) < 1e-4, "14-bit rounding should give well under 0.1 mm"


# ---------------------------------------------------------------------------------- Module 7
def module7():
    m = load("m7_contacts")
    sched = np.array([0, 1, 1, 1, 1, 1, 0, 0], bool)[:, None].repeat(4, 1)
    force = np.array([0, 2, 10, 20, 20, 10, 3, 0], float)[:, None].repeat(4, 1)

    def col(c):
        return np.asarray(c)[:, 0].astype(int).tolist()

    @check("force only: planted while the force is above 5 N   (worksheet 7.1)")
    def _():
        assert col(m.detect_contacts(force, sched, use_schedule=False)) == [0, 0, 1, 1, 1, 1, 0, 0], \
            f"got {col(m.detect_contacts(force, sched, use_schedule=False))}"

    @check("latency: a force signal 1 sample late keeps the foot planted 1 sample too long")
    def _():
        got = col(m.detect_contacts(force, sched, latency=1, use_schedule=False))
        assert got == [0, 0, 0, 1, 1, 1, 1, 0], f"got {got}"

    @check("schedule with advance = 2: contact ends 2 samples before the scheduled liftoff")
    def _():
        got = col(m.detect_contacts(force, sched, advance=2, use_force=False))
        assert got == [0, 1, 1, 1, 0, 0, 0, 0], f"got {got}"

    @check("settle = 2: a foot only counts after more than 2 samples in contact")
    def _():
        got = col(m.detect_contacts(force, sched, settle=2, use_force=False))
        assert got == [0, 0, 0, 1, 1, 1, 0, 0], f"got {got}"

    @check("force AND schedule (advance 1): both have to agree   (worksheet 7.2)")
    def _():
        got = col(m.detect_contacts(force, sched, advance=1))
        assert got == [0, 0, 1, 1, 1, 0, 0, 0], f"got {got}"


# ---------------------------------------------------------------------------------- Module 10
def module10():
    m = load("m10_consistency")

    @check("nees([1, 2, 0], diag(1, 4, 9)) = 2   (worksheet 10.1)")
    def _():
        close(m.nees(np.array([1.0, 2.0, 0.0]), np.diag([1.0, 4.0, 9.0])), 2.0, 1e-12, "NEES")

    @check("Module 2's cart: measuring position sees everything; measuring only velocity can't see position")
    def _():
        F = np.array([[1.0, 0.1], [0.0, 1.0]])
        O = m.observability_matrix([F, F], [np.array([[1.0, 0.0]])] * 3)
        close(O, [[1, 0], [1, 0.1], [1, 0.2]], 1e-12, "O with H = [1, 0]")
        assert m.count_unobservable(O) == 0, "position-only measurements should see both states"
        O2 = m.observability_matrix([F, F], [np.array([[0.0, 1.0]])] * 3)
        assert m.count_unobservable(O2) == 1, "velocity-only measurements can't see position (1 direction)"

    @check("your filter on the walk: exactly 4 unobservable directions (3 position + yaw)")
    def _():
        import simdata
        E = load("eskf")
        d = simdata.make_walk(1, noise=False, biases=False, duration=10.0)
        c = d["contact"]
        k0 = next(k for k in range(8000, 9000) if (c[k:k + 100] == c[k]).all() and c[k].sum() == 2)
        Fs, Hs, _ = m.linearize_window(d, E, k0, 100)
        n = m.count_unobservable(m.observability_matrix(Fs, Hs))
        assert n == 4, f"found {n} unobservable directions, expected 4"


# ---------------------------------------------------------------------------------- Module 11
def module11():
    m = load("m11_mujoco")

    @check("specific_force: resting level reads +g; a rotated sensor sees it rotated   (worksheet 11.1)")
    def _():
        close(m.specific_force(np.eye(3), np.zeros(3)), [0, 0, 9.80665], 1e-12, "level, at rest")
        from so3 import rot_from_rpy
        R = rot_from_rpy(0.0, np.pi / 2, 0.0)
        close(m.specific_force(R, np.zeros(3)), R.T @ np.array([0, 0, 9.80665]), 1e-12, "pitched 90 deg, at rest")
        close(m.specific_force(np.eye(3), np.array([0, 0, -9.80665])), np.zeros(3), 1e-12, "free fall")

    @check("analyze_drop on a 6-sample example: touch at 3 ms, sinks 2 mm more, settled from 5 ms")
    def _():
        t = np.arange(1, 7) * 1e-3
        z = np.array([0.010, 0.009, 0.006, 0.004, 0.005, 0.005])
        force = np.array([0, 0, 6, 20, 15, 15], float)
        close(m.analyze_drop(t, z, force), (0.003, 2.0, 0.005), 1e-12, "(t_touch, max_sink_mm, t_settled)")

    @check("in MuJoCo: your specific_force matches the simulated accelerometer for the whole drop")
    def _():
        try:
            import mujoco  # noqa: F401
        except ImportError:
            raise NotWritten("MuJoCo isn't installed; Module 11 is optional (README, Module 11)")
        lg = m.simulate()
        mine = np.array([m.specific_force(R, a) for R, a in zip(lg["R"], lg["a_world"])])
        close(mine, lg["accel"], 1e-9, "specific force vs accelerometer")


def module9():
    print("  Module 9 is C++. Build and run it as the README says (section \"Module 9: C++\"):")
    print("    cd cpp && cmake -S . -B build && cmake --build build && ./build/rebuild_tests")


MODULES = {"1": module1, "2": module2, "3": module3, "4": module4, "5": module5, "6": module6, "7": module7,
           "8": module8, "9": module9, "10": module10, "11": module11}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    which = args[0] if args else "all"
    keys = [k for k in MODULES if k != "9"] if which == "all" else [which]
    if any(k not in MODULES for k in keys):
        print(__doc__)
        sys.exit(1)
    if USE_SOLUTIONS:
        print("(checking the REFERENCE SOLUTIONS, not your code)")
    for k in keys:
        print(f"\nModule {k}")
        run_module(MODULES[k])
    n = {s: sum(r[0] == s for r in RESULTS) for s in ("PASS", "FAIL", "TODO")}
    print(f"\n{n['PASS']} passed, {n['FAIL']} failed, {n['TODO']} not written yet")
    sys.exit(1 if n["FAIL"] else 0)


if __name__ == "__main__":
    main()
