"""Run your filter on the synthetic walk (provided).

    python run.py imu-only     Module 4: the IMU alone (dead reckoning). Watch it drift.
    python run.py full         Module 5: IMU + legs. Compare with imu-only.
    python run.py legs         Module 6: forward kinematics of rounded joint angles (14, 8, 6-bit encoders)
    python run.py contacts     Module 7: contact policies, your bug #4, and a late force signal
    python run.py report       Module 10: every experiment, and your interview numbers
Options:
    --seed N       which random sensor errors (default 1: its accelerometer bias, 0.175 m/s^2,
                   is close to the typical size; seed 3 has a big one)
    --solutions    run the reference solutions instead of your code
    --plot         also save report.png (needs matplotlib; skipped if it isn't installed)
"""
import importlib
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))
import simdata  # noqa: E402
from so3 import quat_to_rot, rot_to_quat, rpy_from_rot, wrap  # noqa: E402


def make_data(seed=1, turn=True, slip=0.0, foot_motion=False, duration=30.0):
    return simdata.make_walk(seed=seed, turn=turn, slip=slip, foot_motion=foot_motion, duration=duration)


def start_filter(d, E, cls=None, params=None):
    """Calibrate on the standing window, then start the filter at t = 2 s (as the main project does)."""
    dt = d["dt"]
    k0 = int(round(simdata.STAND / dt))
    cal = slice(int(0.25 / dt), k0 - int(0.25 / dt))
    yaw0 = rpy_from_rot(quat_to_rot(d["q"][k0]))[2]     # yaw can't be measured; start it from the truth
    R0, ba0, bg0 = E.static_alignment(d["acc"][cal], d["gyro"][cal], yaw0)
    P0 = E.initial_covariance(d["spec"], cal.stop - cal.start)
    f = (cls or E.Eskf)(params or E.Params(), d["p"][k0], np.zeros(3), rot_to_quat(R0), ba0, bg0, P0)
    return f, k0


def run_filter(d, E, use_legs=True, cls=None, contact=None, params=None):
    """Run one filter over one walk. Returns the errors over time and the headline numbers.
    contact: (n, 4) bool to use instead of the schedule (Module 7). params: an E.Params to use (Module 10)."""
    f, k0 = start_filter(d, E, cls, params)
    contact = d["contact"] if contact is None else contact
    n = len(d["t"])
    m = n - k0
    est_p, est_v, rpy_est = np.zeros((m, 3)), np.zeros((m, 3)), np.zeros((m, 3))
    sig = np.zeros((m, 15))
    P_v = np.zeros((m, 3, 3))
    corr = np.zeros((m, 2))
    for j, k in enumerate(range(k0, n)):
        p, v, q, ba, bg, P = f.step(d["acc"][k], d["gyro"][k], contact[k], d["legs"][k], d["dt"], use_legs)
        est_p[j], est_v[j], rpy_est[j] = p, v, rpy_from_rot(quat_to_rot(q))
        P_v[j] = P[3:6, 3:6]
        sig[j] = np.sqrt(np.clip(np.diag(P), 0, None))
        # correlation between tilt and horizontal accel bias: theta_y with ba_x, theta_x with ba_y
        corr[j] = [P[7, 9] / (sig[j, 7] * sig[j, 9]), P[6, 10] / (sig[j, 6] * sig[j, 10])]
    rpy_true = np.array([rpy_from_rot(quat_to_rot(q)) for q in d["q"][k0:]])
    e_rpy = rpy_est - rpy_true
    e_rpy[:, 2] = wrap(e_rpy[:, 2])
    e_rpy = np.degrees(e_rpy)
    v_err_vec = est_v - d["v"][k0:]
    v_err = np.linalg.norm(v_err_vec, axis=1)
    return dict(t=d["t"][k0:], vel_err=v_err, vel_rmse=float(np.sqrt(np.mean(v_err ** 2))),
                vel_err_vec=v_err_vec, P_v=P_v, z_err=est_p[:, 2] - d["p"][k0:, 2],
                pos_err=np.linalg.norm(est_p - d["p"][k0:], axis=1), roll_err=e_rpy[:, 0],
                pitch_err=e_rpy[:, 1], yaw_err=e_rpy[:, 2], tilt_err=np.hypot(e_rpy[:, 0], e_rpy[:, 1]),
                sig=sig, corr=corr)


def at(res, key, t):
    return res[key][np.searchsorted(res["t"], t)]


def window_mean(res, key, a, b):
    s = (res["t"] >= a) & (res["t"] < b)
    return float(np.mean(np.abs(res[key][s])))


def load_eskf(solutions):
    if solutions:
        sys.path.insert(0, str(HERE / "solutions"))
    return importlib.import_module("eskf")


def timed(label, fn):
    print(f"  running {label} ...", end="", flush=True)
    t0 = time.time()
    out = fn()
    print(f" {time.time() - t0:.0f} s")
    return out


def cmd_imu_only(E, seed):
    d = make_data(seed)
    r = timed("IMU only", lambda: run_filter(d, E, use_legs=False))
    print("\nIMU only (dead reckoning): velocity error grows without limit")
    print("   t (s)   velocity error (m/s)   position error (m)   tilt error (deg)")
    for t in (3, 5, 10, 15, 20, 25, 29.9):
        print(f"   {t:5.1f}   {at(r, 'vel_err', t):18.3f}   {at(r, 'pos_err', t):18.3f}   {at(r, 'tilt_err', t):14.3f}")
    print(f"\n   velocity RMSE over the walk: {r['vel_rmse']:.3f} m/s")
    return r


def cmd_full(E, seed):
    d = make_data(seed)
    imu = timed("IMU only", lambda: run_filter(d, E, use_legs=False))
    full = timed("IMU + legs", lambda: run_filter(d, E))
    print(f"\nVelocity RMSE over the same walk:  IMU only {imu['vel_rmse']:.3f} m/s,  IMU + legs {full['vel_rmse']:.4f} m/s"
          f"  ({imu['vel_rmse'] / full['vel_rmse']:.0f}x better)")
    return imu, full


def cmd_legs(E, L, seed):
    """Module 6: feed the filter forward kinematics of rounded joint angles instead of foot vectors."""
    import legs
    d = make_data(seed)
    q = legs.joint_angles(d["legs"])                       # what perfect encoders would read
    n = len(d["t"])
    worst = max(np.abs(L.foot_position(i, q[k, i]) - d["legs"][k, i]).max() for k in range(0, n, 97) for i in range(4))
    print(f"FK(IK(foot)) round trip: worst error {worst:.2e} m (your foot_position undoes the provided IK)\n")
    print("   encoder bits   one count (deg)   velocity RMSE (m/s)")
    for bits in (14, 8, 6):
        qe = legs.quantize(q, bits)
        z = np.array([[L.foot_position(i, qe[k, i]) for i in range(4)] for k in range(n)])
        r = timed(f"{bits}-bit encoders", lambda: run_filter(dict(d, legs=z), E))
        print(f"   {bits:12d}   {np.degrees(legs.encoder_step(bits)):15.4f}   {r['vel_rmse']:19.4f}")
    print("\n14 bits (the real project) changes nothing: rounding moves the foot ~0.04 mm against 3 mm of model")
    print("error. At 6 bits the rounding is several mm and the filter, still assuming 3 mm, does worse.")


def cmd_contacts(E, C, seed):
    """Module 7: contact detection on a walk whose feet sink and rise like the real ones (DEBUG_LOG #4)."""
    d = make_data(seed, foot_motion=True, duration=20.0)
    sched = d["contact"]
    print("A) Your bug #4: feet sink 2 mm after touchdown and rise 3.4 mm before liftoff; contact from the schedule")
    print("   policy                               height error at 20 s (mm)   velocity RMSE (m/s)")
    for name, kw in (("use the whole scheduled stance", {}),
                     ("wait 20 ms after touchdown (settle)", dict(settle=20)),
                     ("end contact 5 ms before liftoff", dict(advance=5)),
                     ("end contact 10 ms before liftoff", dict(advance=10)),
                     ("end contact 20 ms before liftoff", dict(advance=20))):
        c = C.detect_contacts(d["force"], sched, use_force=False, **kw)
        r = timed(name, lambda: run_filter(d, E, contact=c))
        print(f"   {name:36s} {1000 * r['z_err'][-1]:+18.1f} {r['vel_rmse']:24.4f}")
    print("\nB) Late force signal (the E4 experiment): force only vs force AND schedule (5 ms early)")
    print("   latency (ms)   velocity RMSE, force only   velocity RMSE, force AND schedule")
    for lat in (0, 10, 30):
        rf = timed(f"force only, {lat} ms", lambda: run_filter(d, E, contact=C.detect_contacts(d["force"], sched, latency=lat, use_schedule=False)))
        ra = timed(f"force AND schedule, {lat} ms", lambda: run_filter(d, E, contact=C.detect_contacts(d["force"], sched, latency=lat, advance=5)))
        print(f"   {lat:12d}   {rf['vel_rmse']:21.4f} m/s   {ra['vel_rmse']:27.4f} m/s")
    print("   With no latency the two match: the force here drops below 5 N about 7 ms before liftoff anyway.")
    print("   Your real project also saw yaw drift from late contact (6.6 deg at 10 ms over 60 s, E4). This toy's")
    print("   feet don't reproduce that part, so quote the yaw number from the project, not from here.")


def cmd_report(E, seed, plot):
    bias = np.random.default_rng(seed).normal(0, simdata.SensorSpec().acc_bias_sigma, 3)
    print(f"Seed {seed}: horizontal accelerometer bias {np.hypot(*bias[:2]):.3f} m/s^2 "
          f"(= {np.degrees(np.hypot(*bias[:2]) / simdata.G):.2f} deg of tilt if it were mistaken for tilt)\n")
    d_turn, d_straight = make_data(seed), make_data(seed, turn=False)
    imu = timed("IMU only", lambda: run_filter(d_turn, E, use_legs=False))
    full = timed("IMU + legs (turns at 12-16 s)", lambda: run_filter(d_turn, E))
    straight = timed("IMU + legs, same walk without the turn", lambda: run_filter(d_straight, E))
    slip2 = timed("IMU + legs, feet slipping 2 cm/s", lambda: run_filter(make_data(seed, slip=0.02), E))
    slip10 = timed("IMU + legs, feet slipping 10 cm/s", lambda: run_filter(make_data(seed, slip=0.10), E))

    class Broken(E.Eskf):
        def error_transition(self, acc, gyro, dt):
            F = super().error_transition(acc, gyro, dt)
            F[3:6, 6:9] *= -1.0          # flip the sign of the gravity-leak block
            return F
    broken = timed("IMU + legs with one sign flipped in F", lambda: run_filter(d_turn, E, cls=Broken))

    print("\n1. The headline: legs vs IMU only (same data)")
    print(f"   velocity RMSE   IMU only {imu['vel_rmse']:.3f} m/s   IMU + legs {full['vel_rmse']:.4f} m/s")

    T = full["t"][-1]
    print(f"\n2. What drifts and what doesn't (IMU + legs; error / the filter's own 1-sigma)")
    print("               at 10 s                at the end (30 s)")
    rows = [("velocity (m/s)", "vel_err", np.linalg.norm, slice(3, 6), 1.0),
            ("roll (deg)", "roll_err", None, 6, np.degrees(1)),
            ("pitch (deg)", "pitch_err", None, 7, np.degrees(1)),
            ("yaw (deg)", "yaw_err", None, 8, np.degrees(1)),
            ("position (m)", "pos_err", np.linalg.norm, slice(0, 3), 1.0)]
    for name, key, agg, idx, scale in rows:
        cells = []
        for t in (10.0, T):
            j = np.searchsorted(full["t"], t)
            s = full["sig"][j, idx]
            s = np.linalg.norm(s) if agg else s
            cells.append(f"{abs(full[key][j]):8.4f} / {s * scale:<8.4f}")
        print(f"   {name:14s} {cells[0]}    {cells[1]}")
    print("   Velocity, roll and pitch stay small. Yaw and position only grow: nothing measures them.")

    print("\n3. Tilt vs accelerometer bias: the lean question (mean |tilt error|, degrees)")
    print("                          before the turn (8-12 s)   after it (20-30 s)   |corr(tilt, bias)| at 30 s")
    for name, r in (("with the turn", full), ("without the turn", straight)):
        print(f"   {name:22s} {window_mean(r, 'tilt_err', 8, 12):14.3f} {window_mean(r, 'tilt_err', 20, 30):22.3f}"
              f" {np.abs(r['corr'][-1]).max():22.3f}")
    j = np.searchsorted(full["t"], 11.0)
    print(f"   |corr(tilt, bias)| just before the turn: {np.abs(full['corr'][j]).max():.3f}"
          "  (1.000 = the filter can't tell them apart)")

    print("\n4. A limit: foot slip (the filter assumes a planted foot doesn't move)")
    print(f"   velocity RMSE   no slip {full['vel_rmse']:.4f}   2 cm/s {slip2['vel_rmse']:.4f}   10 cm/s {slip10['vel_rmse']:.4f} m/s")

    print("\n5. What a wrong Jacobian does (one sign flipped in F)")
    print(f"   velocity RMSE   correct F {full['vel_rmse']:.4f} m/s   broken F {broken['vel_rmse']:.4f} m/s"
          "   (python check.py 4 would catch it)")

    M = importlib.import_module("m10_consistency")
    print("\n6. Is the filter honest about its uncertainty? (velocity NEES: 3 means honest)")
    print("   Q noise densities   velocity RMSE   mean NEES   reading")
    nees_rows = {}
    for scale, reading in ((0.1, "overconfident: P says it knows more than it does"),
                           (1.0, "close to honest"), (10.0, "underconfident: P too big")):
        base = E.Params()
        prm = E.Params(acc_noise=base.acc_noise * scale, gyro_noise=base.gyro_noise * scale,
                       acc_rw=base.acc_rw * scale, gyro_rw=base.gyro_rw * scale)
        r = full if scale == 1.0 else timed(f"Q densities x{scale}", lambda: run_filter(d_turn, E, params=prm))
        n = np.mean([M.nees(e, P) for e, P in zip(r["vel_err_vec"][::10], r["P_v"][::10])])
        nees_rows[scale] = n
        print(f"   x{scale:<17g} {r['vel_rmse']:13.4f} {n:11.2f}   {reading}")

    print("\n7. What can it never know? Observability over a 0.1 s window of straight walking")
    d0 = simdata.make_walk(seed, noise=False, biases=False, duration=10.0)
    c = d0["contact"]
    k_win = next(k for k in range(8000, 9000) if (c[k:k + 100] == c[k]).all() and c[k].sum() == 2)
    Fs, Hs, names = M.linearize_window(d0, E, k_win, 100)
    O = M.observability_matrix(Fs, Hs)
    n_null = M.count_unobservable(O)
    _, S, Vt = np.linalg.svd(O)
    print(f"   unobservable directions: {n_null}  (3 position + yaw)")
    for j in (n_null + 1, n_null + 2):
        v = Vt[-j]
        tilt, bias = np.linalg.norm(v[6:8]), np.linalg.norm(v[9:11])   # roll/pitch part, horizontal bias part
        print(f"   next weakest (singular value {S[-j] / S[0]:.1e} of the largest): horizontal accel bias with a"
              f" tilt part, tilt/bias = {tilt / bias:.3f}, about 1/g = {1 / simdata.G:.3f}")

    imu_ratio = imu["vel_rmse"] / full["vel_rmse"]
    print("\nYour interview lines, with these numbers (synthetic data: white noise, perfect contact)")
    print(f'  * "Legs cut velocity error from {imu["vel_rmse"]:.2f} to {full["vel_rmse"]:.3f} m/s, about {imu_ratio:.0f}x."')
    print(f'  * "Velocity, roll and pitch stay bounded; after {T:.0f} s yaw was off {abs(full["yaw_err"][-1]):.2f} deg and'
          f' position {full["pos_err"][-1]:.2f} m, and the filter\'s own sigma for them only grows."')
    print(f'  * "Walking straight, tilt and accel bias stay fully correlated and the tilt error sits at about'
          f' {window_mean(full, "tilt_err", 8, 12):.2f} deg; after a 90 deg turn it drops to {window_mean(full, "tilt_err", 20, 30):.3f}."')
    print(f'  * "Slip breaks the fixed-foot assumption: 10 cm/s of slip gives {slip10["vel_rmse"]:.3f} m/s of error."')
    print(f'  * "With the right Q the velocity NEES is about {nees_rows[1.0]:.1f} (3 is honest); with Q 100x too small'
          f' it is {nees_rows[0.1]:.0f}, overconfident."')
    print(f'  * "The observability matrix has exactly {n_null} null directions, 3 position and yaw; the next weakest'
          ' pair tilt with accelerometer bias at a ratio of 1/g."')

    if plot:
        save_plot(imu, full, straight)


def save_plot(imu, full, straight):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("\n(matplotlib isn't available, so no plot. The numbers above are the whole report.)")
        return
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].semilogy(imu["t"], imu["vel_err"], label="IMU only")
    ax[0].semilogy(full["t"], full["vel_err"], label="IMU + legs")
    ax[0].set(xlabel="time (s)", ylabel="velocity error (m/s)", title="Legs stop the drift", ylim=(1e-4, None))
    ax[0].legend()
    ax[1].plot(full["t"], full["tilt_err"], label="turns at 12-16 s")
    ax[1].plot(straight["t"], straight["tilt_err"], label="never turns")
    ax[1].axvspan(12, 16, alpha=0.15)
    ax[1].set(xlabel="time (s)", ylabel="tilt error (deg)", title="Turning separates tilt from bias")
    ax[1].legend()
    fig.tight_layout()
    out = HERE / "report.png"
    fig.savefig(out, dpi=120)
    print(f"\nSaved {out}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    solutions = "--solutions" in sys.argv
    seed = 1
    if "--seed" in sys.argv:
        seed = int(sys.argv[sys.argv.index("--seed") + 1])
        args = [a for a in args if a != str(seed)]
    if not args or args[0] not in ("imu-only", "full", "legs", "contacts", "report"):
        print(__doc__)
        sys.exit(1)
    E = load_eskf(solutions)
    if solutions:
        print("(running the REFERENCE SOLUTIONS, not your code)")
    try:
        if args[0] == "imu-only":
            cmd_imu_only(E, seed)
        elif args[0] == "full":
            cmd_full(E, seed)
        elif args[0] == "legs":
            cmd_legs(E, importlib.import_module("m6_legs"), seed)
        elif args[0] == "contacts":
            cmd_contacts(E, importlib.import_module("m7_contacts"), seed)
        else:
            cmd_report(E, seed, "--plot" in sys.argv)
    except NotImplementedError:
        print("\nA function this needs still says NotImplementedError. Run python check.py 4 (or 5) to see which.")
        sys.exit(1)


if __name__ == "__main__":
    main()
