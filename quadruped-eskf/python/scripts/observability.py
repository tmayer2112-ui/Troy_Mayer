"""Observability: covariance split over a long walk + numerical null-space check.

  results/observability.json
  results/figures/covariance_observability.png   (+ .csv)
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import observability as ob  # noqa: E402
from qeskf import pipeline as pl  # noqa: E402
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402

RES = pl.ROOT / "results"


def slope_loglog(t, y, t0):
    m = t >= t0
    return float(np.polyfit(np.log(t[m]), np.log(y[m]), 1)[0])


def yaw_information_check(every=10):
    """Does the standard EKF gain information along the unobservable global-yaw direction?

    For a consistent estimator N^T P^-1 N (N = analytic yaw direction) can't grow. Split each
    kinematic update into (a) the update at a fixed linearization point and (b) the shift of the
    linearization point caused by the correction (the textbook EKF inconsistency; FEJ / OC-EKF fix it).
    """
    prm, pol = load_tuned()
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    c = pl.estimator_contacts(log, liftoff_advance_ms=pol.liftoff_advance_ms)
    f, k0 = pl.init_filter(log, pl.ImuSpec(), prm)
    hn, d_upd, d_lin, info_raw = [], [], [], []
    for j in range(len(log["t"]) - k0):
        k = k0 + j
        if j % 100 == 0:   # unnormalised N: metres and radians stay separate, so the direction is stable
            n = ob.analytic_nullspace(f)[:, 3]
            info_raw.append(float(n @ np.linalg.solve(f.P, n)))
        new = f.handle_contacts(c[k], log["qenc"][k])
        if j % every == 0 and len(f.x.order) > len(new):
            legs = [leg for leg in f.x.order if leg not in new]
            n0 = ob.analytic_nullspace(f)[:, 3]
            _, H, _ = f.measurement(legs, log["qenc"][k])
            hn.append(np.abs(H @ n0).max())
            i1 = n0 @ np.linalg.solve(f.P, n0)
            f.update(log["qenc"][k], skip=new)
            i2 = n0 @ np.linalg.solve(f.P, n0)
            n1 = ob.analytic_nullspace(f)[:, 3]
            i3 = n1 @ np.linalg.solve(f.P, n1)
            d_upd.append((i2 - i1) / i1)
            d_lin.append((i3 - i2) / i1)
        else:
            f.update(log["qenc"][k], skip=new)
        f.predict(log["acc"][k], log["gyro"][k], log["dt"])
    d_upd, d_lin, ir = np.array(d_upd), np.array(d_lin), np.array(info_raw)
    rises = np.diff(ir)
    return dict(info_along_yaw_unnormalised=dict(start=float(ir[0]), at_10s=float(ir[100]), at_30s=float(ir[300]),
                                                 end=float(ir[-1]), sum_of_rises_over_start=float(rises[rises > 0].sum() / ir[0]),
                                                 largest_rise_relative=float((rises / ir[:-1]).max())),
                dataset="data/headline_seed0.npz", sampled_every_n_updates=every, samples=int(len(d_upd)),
                max_abs_HN=float(max(hn)),
                update_fixed_point_rel_info_change=dict(median=float(np.median(d_upd)), max=float(d_upd.max())),
                linearization_shift_rel_info_gain=dict(median=float(np.median(d_lin)), mean=float(d_lin.mean()),
                                                       max=float(d_lin.max()), sum_over_samples=float(d_lin.sum())))


def main():
    prm, pol = load_tuned()
    log = pl.load_dataset(pl.ROOT / "data" / "longwalk.npz")
    c = pl.estimator_contacts(log, liftoff_advance_ms=pol.liftoff_advance_ms)
    est = pl.run_cpp(log, params=prm, contact=c)
    e = pl.errors(log, est)
    t = e["t"] + log["dt"]
    sig = np.sqrt(est["Pdiag"])
    deg = np.degrees
    s = pl.summarize(log, est)

    # --- numerical check over every usable stance window of this walk (every ~10 s) ---
    checks = []
    for ks in range(int(4.0 / log["dt"]), len(log["t"]) - 2000, 10000):
        try:
            r = ob.analyze(log, c, ks, 100)
        except ValueError:
            continue
        checks.append(dict(t=r["k0"] * log["dt"], zero_svs=r["n_numerically_zero"], rel_ON_max=max(r["rel_ON"]),
                           fifth_smallest_sv=r["singular_values"][-5]))
    # weak direction at one window, for the README
    k0, legs = ob.stance_window(c, int(30 / log["dt"]), 100)
    O, N = ob.observability_matrix(log, k0, legs, 100)
    sc = np.linalg.norm(O, axis=0)
    _, S, Vt = np.linalg.svd(O / sc)
    names = ["px", "py", "pz", "vx", "vy", "vz", "th_x", "th_y", "th_z", "ba_x", "ba_y", "ba_z", "bg_x", "bg_y",
             "bg_z"] + [f"foot{j}_{a}" for j in range(len(legs)) for a in "xyz"]
    weak = []
    for idx in (-5, -6):
        v = Vt[idx] / sc
        v = v / np.abs(v).max()
        top = np.argsort(-np.abs(v))[:4]
        weak.append({names[i]: float(v[i]) for i in top})

    out = dict(
        dataset="data/longwalk.npz (300 s, long_walk_profile, sensor seed 1)",
        final_sigma=dict(pos_xyz_m=sig[-1, 0:3].tolist(), vel_xyz_mps=sig[-1, 3:6].tolist(),
                         roll_pitch_yaw_deg=deg(sig[-1, 6:9]).tolist()),
        final_abs_error=dict(pos_xyz_m=np.abs(e["dp"][-1]).tolist(), vel_xyz_mps=np.abs(e["dv"][-1]).tolist(),
                             roll_pitch_yaw_deg=np.abs(deg(e["drpy"][-1])).tolist()),
        growth_exponent_after_30s=dict(  # sigma ~ t^alpha ; ~0.5 = random walk, ~0 = bounded
            pos_x=slope_loglog(t, sig[:, 0], 30), pos_z=slope_loglog(t, sig[:, 2], 30),
            yaw=slope_loglog(t, sig[:, 8], 30), vel_x=slope_loglog(t, sig[:, 3], 30),
            roll=slope_loglog(t, sig[:, 6], 30), pitch=slope_loglog(t, sig[:, 7], 30)),
        rmse=dict(vel_world_3d=s["vel_rmse_world_3d"], roll_deg=s["roll_rmse_deg"], pitch_deg=s["pitch_rmse_deg"],
                  yaw_final_deg=s["yaw_final_err_deg"], pos_final_m=s["pos_final_err_m"], distance_m=s["distance_m"]),
        nullspace_checks=checks,
        nullspace_summary=dict(windows=len(checks), all_have_exactly_4_zero_svs=all(x["zero_svs"] == 4 for x in checks),
                               max_rel_ON=max(x["rel_ON_max"] for x in checks)),
        weakest_observable_directions=weak,
        sigma_at=dict((f"{a}s", dict(pos_xyz_m=sig[min(int(a / log["dt"]), len(t) - 1), 0:3].tolist(),
                                     yaw_deg=float(deg(sig[min(int(a / log["dt"]), len(t) - 1), 8])),
                                     roll_pitch_deg=deg(sig[min(int(a / log["dt"]), len(t) - 1), 6:8]).tolist(),
                                     vel_xyz_mps=sig[min(int(a / log["dt"]), len(t) - 1), 3:6].tolist()))
                      for a in (10, 30, 100, 200, 300)),
        bounded_after_30s=dict(vel_sigma_max_mps=float(sig[int(30 / log["dt"]):, 3:6].max()),
                               vel_sigma_min_mps=float(sig[int(30 / log["dt"]):, 3:6].min()),
                               tilt_sigma_max_deg=float(deg(sig[int(30 / log["dt"]):, 6:8]).max()),
                               tilt_sigma_min_deg=float(deg(sig[int(30 / log["dt"]):, 6:8]).min())),
        yaw_information=yaw_information_check(),
        provenance=pl.provenance(),
    )
    (RES / "observability.json").write_text(json.dumps(out, indent=2, default=float))

    # --- figure: 2x2 small multiples, top = unobservable (grows), bottom = observable (bounded) ---
    import matplotlib.pyplot as plt
    ps.setup()
    fig, axs = plt.subplots(2, 2, figsize=(11.5, 7.2), sharex=True)
    step = 50
    tt = t[::step]
    win = 20  # 1 s moving average of the downsampled sigma (gait-rate ripple otherwise fills the panel)

    def smooth(x):
        xp = np.pad(x, (win // 2, win - 1 - win // 2), mode="edge")   # no droop at the ends
        return np.convolve(xp, np.ones(win) / win, mode="valid")

    b = out["bounded_after_30s"]
    s300 = out["sigma_at"]["300s"]
    panels = [
        (axs[0, 0], "Position 1σ (m) — unobservable, grows", [(0, "x", ps.S1), (1, "y", ps.S2), (2, "z", ps.S3)],
         1.0, False, f"at 300 s: x {s300['pos_xyz_m'][0]:.2f}, y {s300['pos_xyz_m'][1]:.3f}, z {s300['pos_xyz_m'][2]:.3f} m"),
        (axs[0, 1], "Yaw 1σ (deg) — unobservable, nothing to correct it", [(8, "yaw", ps.S1)], 180 / np.pi, False,
         f"{out['sigma_at']['10s']['yaw_deg']:.2f}° at 10 s → {s300['yaw_deg']:.2f}° at 300 s"),
        (axs[1, 0], "Velocity 1σ (m/s) — observable, bounded", [(3, "x", ps.S1), (4, "y", ps.S2), (5, "z", ps.S3)],
         1.0, True, f"{b['vel_sigma_min_mps'] * 1000:.1f}–{b['vel_sigma_max_mps'] * 1000:.1f} mm/s after 30 s"),
        (axs[1, 1], "Roll / pitch 1σ (deg) — observable, bounded", [(6, "roll", ps.S1), (7, "pitch", ps.S2)],
         180 / np.pi, True, f"{b['tilt_sigma_min_deg']:.3f}–{b['tilt_sigma_max_deg']:.3f}° after 30 s"),
    ]
    table = dict(t=tt)
    for ax, title, series, k, sm, txt in panels:
        for idx, lab, col in series:
            y = sig[::step, idx] * k
            ax.plot(tt, smooth(y) if sm else y, color=col, label=lab)
            table[f"sigma_{idx}"] = y
        ax.set_title(title, fontsize=11)
        ax.set_ylim(bottom=0)
        ps.note(ax, txt, xy=(0.02, 0.97), ha="left")
    axs[0, 0].legend(loc="center left", ncol=3)
    axs[1, 0].set_ylim(0, 0.0095)
    axs[1, 0].legend(loc="upper right", ncol=3)
    axs[1, 1].legend(loc="upper right", ncol=2)
    axs[1, 1].set_ylim(0, 0.12)
    axs[1, 1].text(0.99, 0.03, "starts at 1.1° (standing calibration); off-scale for the first ~15 s",
                   transform=axs[1, 1].transAxes, ha="right", color=ps.INK2, fontsize=8.5)
    for ax in axs[1]:
        ax.set_xlabel("time (s)")
    fig.suptitle("300 s walk, filter covariance: gravity pins roll, pitch and velocity; nothing pins position or yaw",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", color=ps.INK)
    fig.tight_layout()
    ps.save(fig, "covariance_observability", table)
    print(json.dumps({k: out[k] for k in ("sigma_at", "bounded_after_30s", "final_abs_error", "rmse",
                                          "nullspace_summary", "weakest_observable_directions", "yaw_information")},
                     indent=1, default=float))


if __name__ == "__main__":
    main()
