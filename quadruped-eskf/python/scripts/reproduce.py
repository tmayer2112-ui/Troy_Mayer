"""Headline results. Every number quoted in the README / resume comes out of this script.

  1. load results/tuned_params.json (chosen on the tuning walk by scripts/tune.py)
  2. headline walk, sensor seed 0: IMU-only vs contact-aided (Python reference) + C++ run + parity
  3. Monte Carlo over sensor seeds 0..9 (same walk, fresh IMU/encoder noise), C++
  4. write results/metrics.json, results/rmse_table.md, results/logs/reproduce.log, figures

Metric definitions (fixed before the first run, see README "Metrics"):
  velocity RMSE = sqrt(mean_k ||v_hat_k - v_k||^2), world frame, 3D, 60 s after a 2 s standing
  calibration, 1 kHz samples.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402
from qeskf.so3 import log_so3, quat_to_rot  # noqa: E402

RES = pl.ROOT / "results"
N_SEEDS = 10


def contacts(log, policy):
    return pl.estimator_contacts(log, delay_ms=policy.delay_ms, use_schedule=policy.use_schedule,
                                 liftoff_advance_ms=policy.liftoff_advance_ms)


def pitch_by_segment(log, est):
    """Tilt RMSE in windows before/after the first commanded turn (README, 'The two rows that got worse')."""
    e = pl.errors(log, est)
    out = {}
    for a, b in ((0, 5), (5, 12), (12, 20), (20, 30), (30, 40), (40, 50), (50, 60)):
        m = (e["t"] >= a) & (e["t"] < b)
        out[f"{a}-{b}s"] = dict(pitch_deg=float(np.degrees(np.sqrt(np.mean(e["drpy"][m, 1] ** 2)))),
                                roll_deg=float(np.degrees(np.sqrt(np.mean(e["drpy"][m, 0] ** 2)))))
    first_turn = float(e["t"][np.argmax(np.abs(log["cmd"][est["k0"]:, 2]) > 0.05)])
    return dict(first_commanded_turn_s=first_turn, windows=out)


def foot_motion(log, contact):
    """How far a 'planted' foot-sphere centre actually moves during the estimator's stance windows."""
    fw = log["foot_w"]
    k0 = int(round(pl.T_START / log["dt"]))
    dxy, dz, dur = [], [], []
    for leg in range(4):
        c = contact[:, leg].astype(int)
        on = np.where(np.diff(c) == 1)[0] + 1
        off = np.where(np.diff(c) == -1)[0] + 1
        for a in on[on > k0]:
            b = off[off > a]
            if not len(b):
                continue
            b = b[0]
            dxy.append(np.linalg.norm(fw[b - 1, leg, :2] - fw[a, leg, :2]))
            dz.append(abs(fw[b - 1, leg, 2] - fw[a, leg, 2]))
            dur.append((b - a) * log["dt"])
    dxy, dz, dur = map(np.array, (dxy, dz, dur))
    return dict(stance_windows=int(len(dur)), mean_stance_s=float(dur.mean()),
                xy_displacement_mm_mean=float(1e3 * dxy.mean()), z_displacement_mm_mean=float(1e3 * dz.mean()),
                implied_random_walk_xy_mm_per_sqrt_s=float(1e3 * np.mean(dxy / np.sqrt(dur))),
                implied_random_walk_z_mm_per_sqrt_s=float(1e3 * np.mean(dz / np.sqrt(dur))))


def per_state_table(s_imu, s_con):
    rows = [
        ("Velocity, world 3D (m/s)", s_imu["vel_rmse_world_3d"], s_con["vel_rmse_world_3d"]),
        ("Velocity x, world (m/s)", s_imu["vel_rmse_world_xyz"][0], s_con["vel_rmse_world_xyz"][0]),
        ("Velocity y, world (m/s)", s_imu["vel_rmse_world_xyz"][1], s_con["vel_rmse_world_xyz"][1]),
        ("Velocity z, world (m/s)", s_imu["vel_rmse_world_xyz"][2], s_con["vel_rmse_world_xyz"][2]),
        ("Velocity, body 3D (m/s)", s_imu["vel_rmse_body_3d"], s_con["vel_rmse_body_3d"]),
        ("Roll (deg)", s_imu["roll_rmse_deg"], s_con["roll_rmse_deg"]),
        ("Pitch (deg)", s_imu["pitch_rmse_deg"], s_con["pitch_rmse_deg"]),
        ("Yaw (deg)", s_imu["yaw_rmse_deg"], s_con["yaw_rmse_deg"]),
        ("Position x (m)", s_imu["pos_rmse_xyz"][0], s_con["pos_rmse_xyz"][0]),
        ("Position y (m)", s_imu["pos_rmse_xyz"][1], s_con["pos_rmse_xyz"][1]),
        ("Position z (m)", s_imu["pos_rmse_xyz"][2], s_con["pos_rmse_xyz"][2]),
        ("Accel bias x (m/s²)", s_imu["ba_rmse"][0], s_con["ba_rmse"][0]),
        ("Accel bias y (m/s²)", s_imu["ba_rmse"][1], s_con["ba_rmse"][1]),
        ("Accel bias z (m/s²)", s_imu["ba_rmse"][2], s_con["ba_rmse"][2]),
        ("Gyro bias x (deg/s)", s_imu["bg_rmse_dps"][0], s_con["bg_rmse_dps"][0]),
        ("Gyro bias y (deg/s)", s_imu["bg_rmse_dps"][1], s_con["bg_rmse_dps"][1]),
        ("Gyro bias z (deg/s)", s_imu["bg_rmse_dps"][2], s_con["bg_rmse_dps"][2]),
    ]
    return rows


def fmt(x):
    if x == 0:
        return "0"
    if abs(x) >= 100:
        return f"{x:.0f}"
    if abs(x) >= 1:
        return f"{x:.2f}"
    return f"{x:.3g}"


def figures(log, e_imu, e_con, est_imu, est_con, s_imu, s_con):
    ps.setup()
    t = e_con["t"]
    import matplotlib.pyplot as plt
    # --- 1. velocity tracking: full run (left) + 3 s zoom (right), one row per world axis ---
    k0 = est_con["k0"]
    v_gt = log["v"][k0:k0 + len(t)]
    fig = plt.figure(figsize=(11.5, 7.4))
    gs = fig.add_gridspec(3, 2, width_ratios=[2.3, 1], hspace=0.35, wspace=0.12)
    z0, z1 = 20.0, 23.0
    zm = (t >= z0) & (t <= z1)
    for i, lab in enumerate("xyz"):
        axl = fig.add_subplot(gs[i, 0])
        axr = fig.add_subplot(gs[i, 1], sharey=axl)
        axl.plot(t, v_gt[:, i], color=ps.TRUTH, lw=0.6)
        axl.plot(t, est_con["v"][:, i], color=ps.S1, lw=0.6)
        axr.plot(t[zm], v_gt[zm, i], color=ps.TRUTH, lw=1.4, label="MuJoCo ground truth")
        axr.plot(t[zm], est_con["v"][zm, i], color=ps.S1, lw=1.4, label="Contact-aided ESKF")
        axl.set_ylabel(f"v{lab}, world (m/s)")
        axr.tick_params(labelleft=False)
        axr.set_title(f"v{lab} RMSE {s_con['vel_rmse_world_xyz'][i]:.3f} m/s (full run)", fontsize=10,
                      fontweight="normal", color=ps.INK2)
        if i == 0:
            axl.set_title("Full 60 s", fontsize=10, fontweight="normal", color=ps.INK2)
            axl.axvspan(z0, z1, color=ps.WASH, lw=0, zorder=0)
        if i == 2:
            axl.set_xlabel("time since filter start (s)")
            axr.set_xlabel(f"zoom: t = {z0:.0f}–{z1:.0f} s")
        else:
            axl.tick_params(labelbottom=False)
            axr.tick_params(labelbottom=False)
    h, l = axr.get_legend_handles_labels()
    fig.legend(h, l, loc="upper right", ncol=2, bbox_to_anchor=(0.99, 1.0))
    fig.suptitle(f"Base velocity vs ground truth, 60 s walk: 3D RMSE {s_con['vel_rmse_world_3d']:.4f} m/s",
                 x=0.01, y=1.0, ha="left", fontsize=12, fontweight="bold", color=ps.INK)
    step = 10
    ps.save(fig, "velocity_tracking", dict(t=t[::step], vx_true=v_gt[::step, 0], vy_true=v_gt[::step, 1],
                                           vz_true=v_gt[::step, 2], vx_est=est_con["v"][::step, 0],
                                           vy_est=est_con["v"][::step, 1], vz_est=est_con["v"][::step, 2]))
    # --- 2. drift: velocity error norm, IMU-only vs contact-aided, log scale ---
    fig, ax = plt.subplots(figsize=(10, 4.2))
    ni = np.linalg.norm(e_imu["dv"], axis=1)
    nc = np.linalg.norm(e_con["dv"], axis=1)
    ax.semilogy(t, ni, color=ps.S2, label="IMU only (no corrections)")
    ax.semilogy(t, nc, color=ps.S1, lw=1.0, label="Contact-aided ESKF")
    ax.set_ylim(1e-4, 20)
    ax.set_ylabel("|velocity error| (m/s)")
    ax.set_xlabel("time since filter start (s)")
    ax.set_title("Dead reckoning falls apart in seconds; leg kinematics bound it")
    for thr in (0.1, 1.0):
        tt = s_imu[f"t_vel_err_gt_{thr}"]
        if tt is not None:
            ax.axhline(thr, color=ps.AXIS, lw=0.8, zorder=0)
            ax.annotate(f"IMU-only error passes {thr} m/s at {tt:.1f} s", xy=(tt, thr), xytext=(tt + 2.5, thr * 0.3),
                        color=ps.INK2, fontsize=9.5, arrowprops=dict(arrowstyle="-", color=ps.MUTED, lw=0.8))
    ax.legend(loc="lower right")
    ps.save(fig, "imu_only_drift", dict(t=t[::step], err_imu_only=ni[::step], err_contact=nc[::step]))
    # --- 3. top-down trajectory ---
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    p_gt = log["p"][k0:k0 + len(t)]
    ax.plot(p_gt[:, 0], p_gt[:, 1], color=ps.TRUTH, lw=1.2, label="Ground truth")
    ax.plot(est_con["p"][:, 0], est_con["p"][:, 1], color=ps.S1, label="Contact-aided ESKF")
    inside = (np.abs(est_imu["p"][:, 0] - p_gt[:, 0]) < 3) & (np.abs(est_imu["p"][:, 1] - p_gt[:, 1]) < 3)
    last_in = np.argmin(inside) if not inside.all() else len(t) - 1
    ax.plot(est_imu["p"][:last_in, 0], est_imu["p"][:last_in, 1], color=ps.S2, label="IMU only")
    ax.plot(p_gt[0, 0], p_gt[0, 1], "o", color=ps.TRUTH, ms=6)
    ax.annotate(f"IMU only leaves the plot at t = {t[last_in]:.0f} s\n(final error {s_imu['pos_final_err_m']:.0f} m)",
                xy=(est_imu["p"][last_in - 1, 0], est_imu["p"][last_in - 1, 1]), xytext=(8, -4),
                textcoords="offset points", color=ps.INK2, fontsize=9.5, va="top")
    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Top-down path, {s_con['distance_m']:.1f} m walked; contact-aided final "
                 f"xy error {s_con['pos_final_err_xy_m']:.2f} m")
    ax.legend(loc="upper right")
    ps.save(fig, "trajectory_xy", dict(t=t[::step], x_true=p_gt[::step, 0], y_true=p_gt[::step, 1],
                                       x_est=est_con["p"][::step, 0], y_est=est_con["p"][::step, 1]))
    # --- 3b. card thumbnail for the portfolio page (16:10, no title; the page supplies the caption) ---
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(p_gt[:, 0], p_gt[:, 1], color=ps.TRUTH, lw=1.4, label="ground truth")
    ax.plot(est_con["p"][:, 0], est_con["p"][:, 1], color=ps.S1, lw=2.0, label="contact-aided ESKF")
    ax.plot(est_imu["p"][:last_in, 0], est_imu["p"][:last_in, 1], color=ps.S2, lw=2.0, label="IMU only")
    ax.set_aspect("equal", "datalim")
    ax.tick_params(labelbottom=False, labelleft=False, length=0)
    ax.legend(loc="upper right", fontsize=11)
    fig.tight_layout(pad=0.4)
    ps.save(fig, "card_trajectory")
    # --- 4. tilt vs accel bias: separates only after the robot turns ---
    fig, axs = plt.subplots(2, 1, figsize=(10, 5.8), sharex=True)
    cmd = log["cmd"][k0:k0 + len(t)]
    turning = np.abs(cmd[:, 2]) > 0.05
    sig_roll = np.degrees(np.sqrt(est_con["Pdiag"][:, 6]))
    sig_bay = np.sqrt(est_con["Pdiag"][:, 10])
    for ax in axs:
        ax.fill_between(t, 0, 1, where=turning, transform=ax.get_xaxis_transform(), color=ps.WASH, lw=0,
                        label="commanded yaw rate ≠ 0")
    win = 500  # 0.5 s rolling RMS: |error| crosses zero every gait cycle, which is unreadable on a log axis
    ker = np.ones(win) / win
    rms_roll = np.sqrt(np.convolve(np.degrees(e_con["drpy"][:, 0]) ** 2, ker, mode="same"))
    rms_bay = np.sqrt(np.convolve(e_con["dba"][:, 1] ** 2, ker, mode="same"))
    axs[0].plot(t, rms_roll, color=ps.S1, label="roll error (0.5 s RMS)")
    axs[0].plot(t, sig_roll, color=ps.S2, label="filter 1σ")
    axs[0].set_yscale("log")
    axs[0].set_ylim(3e-3, 3)
    axs[0].set_ylabel("roll (deg)")
    axs[1].plot(t, rms_bay, color=ps.S1, label="error (0.5 s RMS)")
    axs[1].plot(t, sig_bay, color=ps.S2, label="filter 1σ")
    axs[1].set_yscale("log")
    axs[1].set_ylim(3e-4, 0.5)
    axs[1].set_ylabel("accel bias y (m/s²)")
    axs[1].set_xlabel("time since filter start (s)")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper left", ncol=3, bbox_to_anchor=(0.01, 0.955))
    fig.suptitle("Tilt and horizontal accel bias separate only once the body yaws", x=0.01, y=1.0, ha="left",
                 fontsize=12, fontweight="bold", color=ps.INK)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    ps.save(fig, "tilt_bias_separation", dict(t=t[::step], roll_err_deg=np.degrees(e_con["drpy"][::step, 0]),
                                              roll_err_rms_deg=rms_roll[::step], bay_err_rms=rms_bay[::step],
                                              roll_sigma_deg=sig_roll[::step], bay_err=e_con["dba"][::step, 1],
                                              bay_sigma=sig_bay[::step], turning=turning[::step]))


def main():
    t_start = time.time()
    prm, policy = load_tuned()
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    c = contacts(log, policy)

    est_imu = pl.run_filter(log, params=prm, mode="imu_only", contact=c)
    est_con = pl.run_filter(log, params=prm, mode="contact", contact=c)
    est_cpp = pl.run_cpp(log, params=prm, contact=c)
    s_imu, s_con, s_cpp = (pl.summarize(log, e) for e in (est_imu, est_con, est_cpp))
    e_imu, e_con = pl.errors(log, est_imu), pl.errors(log, est_con)
    dth = max(np.linalg.norm(log_so3(quat_to_rot(a).T @ quat_to_rot(b))) for a, b in zip(est_cpp["q"], est_con["q"]))
    parity = dict(max_abs_dp_m=float(np.abs(est_cpp["p"] - est_con["p"]).max()),
                  max_abs_dv_mps=float(np.abs(est_cpp["v"] - est_con["v"]).max()),
                  max_dtheta_rad=float(dth),
                  max_rel_dPdiag=float((np.abs(est_cpp["Pdiag"] - est_con["Pdiag"]) / est_con["Pdiag"]).max()),
                  vel_rmse_python=s_con["vel_rmse_world_3d"], vel_rmse_cpp=s_cpp["vel_rmse_world_3d"])
    nees_v = pl.nees(log, est_con, "v")

    mc = []
    for seed in range(N_SEEDS):
        lg = log if seed == 0 else pl.load_dataset(pl.ROOT / "data" / f"headline_seed{seed}.npz")
        cc = contacts(lg, policy)
        si = pl.summarize(lg, pl.run_cpp(lg, params=prm, mode="imu_only", contact=cc))
        sc = pl.summarize(lg, pl.run_cpp(lg, params=prm, contact=cc))
        mc.append(dict(seed=seed, vel_rmse_contact=sc["vel_rmse_world_3d"], vel_rmse_imu_only=si["vel_rmse_world_3d"],
                       pos_final_err_contact=sc["pos_final_err_m"], yaw_final_err_contact=sc["yaw_final_err_deg"],
                       roll_rmse_contact=sc["roll_rmse_deg"], pitch_rmse_contact=sc["pitch_rmse_deg"],
                       t_imu_only_err_gt_0p1=si["t_vel_err_gt_0.1"]))
    v_mc = np.array([m["vel_rmse_contact"] for m in mc])

    metrics = dict(
        headline=dict(
            vel_rmse_world_3d_mps=s_con["vel_rmse_world_3d"],
            trajectory_duration_s=s_con["duration_s"],
            estimator_rate_hz=1.0 / log["dt"],
            definition="sqrt(mean ||v_hat - v_true||^2), world frame, 3D, 1 kHz samples, 60 s walk after 2 s "
                       "standing calibration; sensor seed 0; parameters from results/tuned_params.json",
            dataset="data/headline_seed0.npz (python/scripts/generate_data.py)",
        ),
        monte_carlo=dict(n_seeds=N_SEEDS, vel_rmse_mean=float(v_mc.mean()), vel_rmse_std=float(v_mc.std(ddof=1)),
                         vel_rmse_min=float(v_mc.min()), vel_rmse_max=float(v_mc.max()), runs=mc),
        imu_only=s_imu, contact_aided=s_con, cpp_contact_aided=s_cpp, parity_python_vs_cpp=parity,
        consistency=dict(nees_velocity_median=float(np.median(nees_v)), nees_velocity_mean=float(nees_v.mean()),
                         expected="3.0 for a consistent 3-dof estimate"),
        tilt_by_segment=pitch_by_segment(log, est_con),
        foot_motion_in_stance=foot_motion(log, c),
        tuned=json.loads((RES / "tuned_params.json").read_text()),
        imu_spec=log["meta"]["imu"],
        provenance=pl.provenance(),
        wall_time_s=time.time() - t_start,
    )
    RES.mkdir(exist_ok=True)
    (RES / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float))

    rows = per_state_table(s_imu, s_con)
    md = ["| State (RMSE) | IMU-only | Contact-aided | Contact-aided vs IMU-only |", "|---|---:|---:|---:|"]
    def ratio(a, b):
        r = a / b
        if r >= 10:
            return f"{r:.0f}× better"
        if r >= 1:
            return f"{r:.1f}× better"
        return f"**{1 / r:.1f}× worse**"
    for name, a, b in rows:
        md.append(f"| {name} | {fmt(a)} | {fmt(b)} | {ratio(a, b)} |")
    md.append("")
    md.append(f"60 s walk, sensor seed 0, 1 kHz. RMSE over the full window. Distance walked {s_con['distance_m']:.1f} m. "
              f"Final position error: IMU-only {s_imu['pos_final_err_m']:.1f} m, contact-aided "
              f"{s_con['pos_final_err_m']:.2f} m ({s_con['pos_final_err_xy_m']:.2f} m horizontal). "
              f"Final yaw error: IMU-only {s_imu['yaw_final_err_deg']:+.2f}°, contact-aided {s_con['yaw_final_err_deg']:+.2f}°.")
    md.append("")
    md.append(f"Monte Carlo, {N_SEEDS} sensor-noise seeds on the same walk: contact-aided velocity RMSE "
              f"{v_mc.mean():.4f} ± {v_mc.std(ddof=1):.4f} m/s (min {v_mc.min():.4f}, max {v_mc.max():.4f}).")
    (RES / "rmse_table.md").write_text("\n".join(md) + "\n")

    figures(log, e_imu, e_con, est_imu, est_con, s_imu, s_con)

    lines = [
        "# reproduce.py run log",
        f"command: PYTHONPATH=python python3 python/scripts/reproduce.py",
        f"provenance: {json.dumps(metrics['provenance'])}",
        f"tuned params: {json.dumps(metrics['tuned']['eskf'])} contact={json.dumps(metrics['tuned']['contact'])}",
        f"dataset: {metrics['headline']['dataset']}  duration {s_con['duration_s']:.3f} s at {1 / log['dt']:.0f} Hz",
        "",
        f"HEADLINE vel RMSE (world 3D)      contact-aided {s_con['vel_rmse_world_3d']:.4f} m/s   "
        f"IMU-only {s_imu['vel_rmse_world_3d']:.4f} m/s",
        f"C++ replay vel RMSE               {s_cpp['vel_rmse_world_3d']:.4f} m/s  "
        f"(parity max |dp| {parity['max_abs_dp_m']:.1e} m, |dv| {parity['max_abs_dv_mps']:.1e} m/s)",
        f"Monte Carlo ({N_SEEDS} seeds)            {v_mc.mean():.4f} +- {v_mc.std(ddof=1):.4f} m/s "
        f"[{v_mc.min():.4f}, {v_mc.max():.4f}]",
        f"IMU-only |v err| > 0.1 m/s at     {s_imu['t_vel_err_gt_0.1']} s;  > 1.0 m/s at {s_imu['t_vel_err_gt_1.0']} s",
        f"IMU-only |v err| growth, first 10 s: {s_imu['vel_err_rate_first10s']:.4f} m/s per s",
        f"velocity NEES median              {np.median(nees_v):.2f} (3.0 = consistent)",
        "",
        "per-state RMSE:",
        *[f"  {n:28s} IMU-only {fmt(a):>10s}   contact-aided {fmt(b):>10s}" for n, a, b in rows],
        "",
        f"wall time {metrics['wall_time_s']:.0f} s",
    ]
    (RES / "logs").mkdir(exist_ok=True)
    (RES / "logs" / "reproduce.log").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
