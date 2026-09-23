"""Dataset generation, filter runs and metrics. Everything the scripts need, no plotting."""
import json
import platform
import subprocess
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from . import kinematics as K
from .eskf import NB, Eskf, EskfParams, initial_covariance, static_alignment
from .gait import GaitParams, command_profile
from .imu_model import EncoderSpec, ImuSpec, quantize_encoders, synthesize_imu  # noqa: F401 (re-exported)
from .sim import simulate
from .so3 import log_so3, quat_to_rot, rpy_from_rot, wrap

ROOT = Path(__file__).resolve().parents[2]
CAL_WINDOW = (0.5, 2.0)   # s, robot standing still (settles from the keyframe during the first 0.5 s)
T_START = 2.0             # s, filter starts here; evaluation window is [T_START, T_START + duration)
CONTACT_FORCE_N = 5.0     # N, contact force threshold (a force-sensing foot / torque-based estimate)


# ---------------------------------------------------------------------- dataset
def make_dataset(duration=60.0, seed=0, profile=command_profile, gait=GaitParams(), imu=ImuSpec(),
                 enc=EncoderSpec(), floor_friction=None, push=None, sim_log=None):
    """Simulate (or reuse sim_log) and synthesize sensors. The sim is deterministic; seed only drives sensor noise."""
    t0 = time.time()
    if sim_log is None:
        log = simulate(T_START + duration, profile=profile, gait=gait, floor_friction=floor_friction,
                       contact_threshold=CONTACT_FORCE_N, push=push)
    else:
        log = {k: v for k, v in sim_log.items() if k not in ("acc", "gyro", "ba_true", "bg_true", "qenc", "meta")}
    rng = np.random.default_rng(seed)
    acc, gyr, ba, bg = synthesize_imu(log["acc_true"], log["gyro_true"], imu, rng)
    log.update(acc=acc, gyro=gyr, ba_true=ba, bg_true=bg, qenc=quantize_encoders(log["qj"], enc))
    log["meta"] = dict(duration=duration, seed=seed, imu=imu.summary(), encoder_bits=enc.bits,
                       gait=asdict(gait), floor_friction=floor_friction, sim_wall_s=time.time() - t0,
                       profile=profile.__name__)
    return log


def _shift(c, n):
    """Delay a boolean (N,4) signal by n samples (n > 0 later)."""
    if n <= 0:
        return c
    return np.vstack([np.repeat(c[:1], n, 0), c[:-n]])


def _trim_end(c, n):
    """Drop the last n samples of every True window (end the window early)."""
    if n <= 0:
        return c
    c = c.copy()
    for leg in range(c.shape[1]):
        off = np.where(np.diff(c[:, leg].astype(int)) == -1)[0] + 1
        for b in off:
            c[max(b - n, 0):b, leg] = False
    return c


def estimator_contacts(log, delay_ms=0, use_schedule=True, liftoff_advance_ms=0):
    """Contact flags the estimator sees.

    force:     normal force > CONTACT_FORCE_N (a force/torque-based detector), delayed by delay_ms
    schedule:  the gait planner's stance window, ended liftoff_advance_ms early. The planner knows
               its own schedule ahead of time, so ending it early is causal on a real robot.
    use_schedule=True -> force AND schedule (default); False -> force only.
    """
    dt_ms = log["dt"] * 1000.0
    force = _shift(log["contact"].astype(bool), int(round(delay_ms / dt_ms)))
    if not use_schedule:
        return _trim_end(force, int(round(liftoff_advance_ms / dt_ms)))
    sched = _trim_end(log["sched"].astype(bool), int(round(liftoff_advance_ms / dt_ms)))
    return force & sched


def slip_injection(log, contact, v_slip, seed=1):
    """Add a synthetic foot slide of speed v_slip (m/s) during every stance to the kinematics.

    The slide points along -v_world (horizontal), like a foot skating backwards on a
    low-friction floor while the trunk pushes forward. Returns body-frame offsets
    (N,4,3) to add to FK(q). Equivalent to the foot moving in the world while the
    trunk motion is unchanged.
    """
    n = len(log["t"])
    off = np.zeros((n, 4, 3))
    if v_slip == 0:
        return off
    dt = log["dt"]
    vh = log["v"].copy()
    vh[:, 2] = 0
    for leg in range(4):
        s = np.zeros(3)
        for k in range(n):
            if contact[k, leg]:
                sp = np.linalg.norm(vh[k])
                if sp > 0.05:
                    s = s - v_slip * vh[k] / sp * dt
                R = quat_to_rot(log["quat"][k])
                off[k, leg] = R.T @ s
            else:
                s = np.zeros(3)
    return off


def save_dataset(log, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {k: v for k, v in log.items() if isinstance(v, np.ndarray)}
    np.savez_compressed(path, **arrays, dt=log["dt"], meta=json.dumps(log["meta"], default=float))


def load_dataset(path):
    z = np.load(path, allow_pickle=False)
    log = {k: z[k] for k in z.files if k not in ("meta", "dt")}
    log["dt"] = float(z["dt"])
    log["meta"] = json.loads(str(z["meta"]))
    return log


def export_csv(log, contact, path, k0=None, k1=None, slip_off=None):
    """Sensor stream for the C++ replay: t, acc(3), gyro(3), qenc(12), contact(4)."""
    k0 = int(round(T_START / log["dt"])) if k0 is None else k0
    k1 = len(log["t"]) if k1 is None else k1
    qenc = log["qenc"]
    cols = [log["t"][k0:k1, None], log["acc"][k0:k1], log["gyro"][k0:k1], qenc[k0:k1],
            contact[k0:k1].astype(float)]
    hdr = "t,ax,ay,az,gx,gy,gz," + ",".join(f"q{i}" for i in range(12)) + ",c0,c1,c2,c3"
    np.savetxt(path, np.hstack(cols), delimiter=",", header=hdr, comments="", fmt="%.17g")


# ---------------------------------------------------------------------- filter runs
def init_filter(log, imu: ImuSpec, params: EskfParams):
    dt = log["dt"]
    k_cal = slice(int(CAL_WINDOW[0] / dt), int(CAL_WINDOW[1] / dt))
    k0 = int(round(T_START / dt))
    # The navigation frame is defined by the true pose at k0: position and yaw are
    # unobservable, so any odometry is evaluated relative to its starting pose.
    yaw0 = rpy_from_rot(quat_to_rot(log["quat"][k0]))[2]
    R0, ba0, bg0 = static_alignment(log["acc"][k_cal], log["gyro"][k_cal], yaw0)
    from .so3 import rot_to_quat
    P0 = initial_covariance(imu, n_cal=k_cal.stop - k_cal.start)
    f = Eskf(params, p=log["p"][k0], v=np.zeros(3), q=rot_to_quat(R0), ba=ba0, bg=bg0, P0=P0)
    return f, k0


def run_filter(log, imu: ImuSpec = None, params: EskfParams = None, mode="contact", contact=None,
               slip_off=None, record_P=False):
    """mode: 'contact' (IMU + leg kinematics) or 'imu_only' (dead reckoning, same init, same integrator)."""
    imu = imu or ImuSpec(**{k: v for k, v in log["meta"]["imu"].items() if k in ImuSpec.__dataclass_fields__})
    params = params or EskfParams.from_imu_spec(imu)
    contact = estimator_contacts(log) if contact is None else contact
    f, k0 = init_filter(log, imu, params)
    n = len(log["t"]) - k0
    out = dict(p=np.zeros((n, 3)), v=np.zeros((n, 3)), q=np.zeros((n, 4)), ba=np.zeros((n, 3)),
               bg=np.zeros((n, 3)), Pdiag=np.zeros((n, NB)), nfeet=np.zeros(n, int))
    dt = log["dt"]
    use_kin = mode == "contact"
    t0 = time.perf_counter()
    for j in range(n):
        k = k0 + j
        p, v, q, ba, bg, Pd = f.step(log["acc"][k], log["gyro"][k], log["qenc"][k], contact[k], dt,
                                     use_kinematics=use_kin,
                                     fk_offset=None if slip_off is None else slip_off[k])
        out["p"][j], out["v"][j], out["q"][j], out["ba"][j], out["bg"][j], out["Pdiag"][j] = p, v, q, ba, bg, Pd
        out["nfeet"][j] = len(f.x.order)
    out["wall_s"] = time.perf_counter() - t0
    out["stats"] = dict(f.stats)
    out["k0"] = k0
    return out


# ---------------------------------------------------------------------- metrics
def errors(log, est):
    """Per-sample errors over the evaluation window (estimate minus truth)."""
    k0 = est["k0"]
    n = len(est["v"])
    sl = slice(k0, k0 + n)
    Rg = np.array([quat_to_rot(q) for q in log["quat"][sl]])
    Re = np.array([quat_to_rot(q) for q in est["q"]])
    rpy_g = np.array([rpy_from_rot(R) for R in Rg])
    rpy_e = np.array([rpy_from_rot(R) for R in Re])
    # local attitude error, same convention as the filter: R_true = R_hat Exp(dtheta)
    dth = np.array([log_so3(Re[i].T @ Rg[i]) for i in range(n)])
    vb_g = np.einsum("nji,nj->ni", Rg, log["v"][sl])
    vb_e = np.einsum("nji,nj->ni", Re, est["v"])
    return dict(
        t=log["t"][sl] - log["t"][k0],
        dp=est["p"] - log["p"][sl],
        dv=est["v"] - log["v"][sl],
        dvb=vb_e - vb_g,
        drpy=wrap(rpy_e - rpy_g),
        dth=dth,
        dba=est["ba"] - log["ba_true"][sl],
        dbg=est["bg"] - log["bg_true"][sl],
    )


def _rms(x, axis=0):
    return np.sqrt(np.mean(np.square(x), axis=axis))


def summarize(log, est):
    e = errors(log, est)
    t = e["t"]
    dist = np.sum(np.linalg.norm(np.diff(log["p"][est["k0"]:est["k0"] + len(t), :2], axis=0), axis=1))
    s = dict(
        duration_s=float(t[-1] - t[0] + log["dt"]),
        vel_rmse_world_3d=float(np.sqrt(np.mean(np.sum(e["dv"] ** 2, 1)))),   # HEADLINE metric
        vel_rmse_world_xyz=_rms(e["dv"]).tolist(),
        vel_rmse_body_xyz=_rms(e["dvb"]).tolist(),
        vel_rmse_body_3d=float(np.sqrt(np.mean(np.sum(e["dvb"] ** 2, 1)))),
        roll_rmse_deg=float(np.degrees(_rms(e["drpy"][:, 0]))),
        pitch_rmse_deg=float(np.degrees(_rms(e["drpy"][:, 1]))),
        yaw_rmse_deg=float(np.degrees(_rms(e["drpy"][:, 2]))),
        yaw_final_err_deg=float(np.degrees(e["drpy"][-1, 2])),
        pos_rmse_xyz=_rms(e["dp"]).tolist(),
        pos_final_err_m=float(np.linalg.norm(e["dp"][-1])),
        pos_final_err_xy_m=float(np.linalg.norm(e["dp"][-1, :2])),
        distance_m=float(dist),
        drift_pct_distance=float(100 * np.linalg.norm(e["dp"][-1, :2]) / max(dist, 1e-9)),
        ba_rmse=_rms(e["dba"]).tolist(),
        bg_rmse_dps=np.degrees(_rms(e["dbg"])).tolist(),
        filter_wall_s=est["wall_s"],
        stats=est.get("stats", {}),
    )
    # time until the world-velocity error norm first exceeds thresholds (dead-reckoning drift)
    vn = np.linalg.norm(e["dv"], axis=1)
    for thr in (0.1, 0.5, 1.0):
        idx = np.argmax(vn > thr) if np.any(vn > thr) else None
        s[f"t_vel_err_gt_{thr}"] = None if idx is None else float(t[idx])
    # velocity error growth rate from a linear fit over the first 10 s (m/s per s)
    m = t <= 10.0
    s["vel_err_rate_first10s"] = float(np.polyfit(t[m], vn[m], 1)[0])
    return s


def nees(log, est, block):
    """Normalized estimation error squared for a 3-dof block using the logged marginal variances (diag only)."""
    e = errors(log, est)
    idx = {"v": (3, e["dv"]), "th": (6, e["dth"])}[block]
    var = est["Pdiag"][:, idx[0]:idx[0] + 3]
    return np.sum(idx[1] ** 2 / var, axis=1)


def provenance():
    def sh(cmd):
        try:
            return subprocess.check_output(cmd, cwd=ROOT, stderr=subprocess.DEVNULL, text=True).strip()
        except Exception:
            return "unknown"
    import mujoco
    return dict(git_commit=sh(["git", "rev-parse", "HEAD"]), git_dirty=bool(sh(["git", "status", "--porcelain", "."])),
                python=platform.python_version(), numpy=np.__version__, mujoco=mujoco.__version__,
                machine=platform.machine(), cpu=_cpu_name(), time_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))


def _cpu_name():
    try:
        for line in open("/proc/cpuinfo"):
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


# ---------------------------------------------------------------------- C++ bridge
BUILD = ROOT / "build"


def write_cpp_config(path, f, params: EskfParams, dt, extra=None):
    """Parameters + the exact initial state/P0 the Python filter starts from."""
    lines = [f"{k} {float(getattr(params, k))!r}" for k in
             ("acc_noise", "gyro_noise", "acc_rw", "gyro_rw", "foot_noise", "kin_noise", "enc_lsb", "gate_chi2",
              "settle_steps")]
    lines += [f"dt {dt!r}",
              "p0 " + " ".join(repr(float(x)) for x in f.x.p),
              "v0 " + " ".join(repr(float(x)) for x in f.x.v),
              "q0 " + " ".join(repr(float(x)) for x in f.x.q),
              "ba0 " + " ".join(repr(float(x)) for x in f.x.ba),
              "bg0 " + " ".join(repr(float(x)) for x in f.x.bg),
              "P0 " + " ".join(repr(float(x)) for x in f.P.ravel())]
    for k, v in (extra or {}).items():
        lines.append(f"{k} {float(v)!r}")
    Path(path).write_text("\n".join(lines) + "\n")


def run_cpp(log, imu: ImuSpec = None, params: EskfParams = None, mode="contact", contact=None, workdir=None,
            binary="qeskf_replay"):
    """Same inputs/outputs as run_filter, executed by the C++ replay binary."""
    imu = imu or ImuSpec(**{k: v for k, v in log["meta"]["imu"].items() if k in ImuSpec.__dataclass_fields__})
    params = params or EskfParams.from_imu_spec(imu)
    contact = estimator_contacts(log) if contact is None else contact
    f, k0 = init_filter(log, imu, params)
    wd = Path(workdir or (ROOT / "data" / "cpp_run"))
    wd.mkdir(parents=True, exist_ok=True)
    write_cpp_config(wd / "config.txt", f, params, log["dt"])
    export_csv(log, contact, wd / "sensors.csv", k0=k0)
    cmd = [str(BUILD / binary), str(wd / "config.txt"), str(wd / "sensors.csv"), str(wd / "est.csv")]
    if mode == "imu_only":
        cmd.append("--imu-only")
    subprocess.run(cmd, check=True, capture_output=True)
    return read_cpp_estimates(wd / "est.csv", k0)


def read_cpp_estimates(path, k0):
    a = np.loadtxt(path, delimiter=",", skiprows=1)
    return dict(p=a[:, 1:4], v=a[:, 4:7], q=a[:, 7:11], ba=a[:, 11:14], bg=a[:, 14:17], Pdiag=a[:, 17:32],
                nfeet=a[:, 32].astype(int), step_ns=a[:, 33], wall_s=float(a[:, 33].sum() * 1e-9), k0=k0,
                stats={})
