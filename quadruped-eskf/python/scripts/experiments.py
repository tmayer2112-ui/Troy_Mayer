"""Design reversals and deliberate breakage. Baseline = tuned params on the headline walk, seed 0.

E1 hard_vs_soft   sweep foot random-walk noise from ~hard (1e-4) to very soft (1e-1), clean and with slip
E2 slip           synthetic foot slide at v_slip during every stance; hard / tuned / soft / tuned+chi2 gate
E3 friction       physical slip: re-simulate the walk with lower foot friction (mu 0.8 -> 0.12)
E4 contact_delay  contact detection latency 0..60 ms; force-only detector vs force AND gait schedule; +gate
E5 process_noise  datasheet Q vs "tutorial" Q = 0.01 I per step vs 10x under/over; thermal gyro drift
Writes results/experiments.json and figures/*.png (+ .csv twins).
"""
import argparse
import json
import os
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402
from qeskf.imu_model import ImuSpec  # noqa: E402
from qeskf.gait import command_profile  # noqa: E402

RES = pl.ROOT / "results"
CHI2_3DOF_999 = 16.266  # chi-square, 3 dof, 99.9 %
_LOGS = {}


def _log(name):
    if name not in _LOGS:
        _LOGS[name] = pl.load_dataset(pl.ROOT / "data" / f"{name}.npz")
    return _LOGS[name]


def _metrics(log, est):
    s = pl.summarize(log, est)
    nv = pl.nees(log, est, "v")
    e = pl.errors(log, est)
    late = e["t"] >= 12.0   # after the first turn (see README, observability)
    return dict(vel_rmse=s["vel_rmse_world_3d"], vel_rmse_z=s["vel_rmse_world_xyz"][2],
                pos_final_err_m=s["pos_final_err_m"], yaw_final_err_deg=s["yaw_final_err_deg"],
                roll_rmse_deg=s["roll_rmse_deg"], pitch_rmse_deg=s["pitch_rmse_deg"],
                tilt_rmse_after_12s_deg=float(np.degrees(np.sqrt(np.mean(e["drpy"][late, :2] ** 2)))),
                nees_v_median=float(np.median(nv)), gated=s["stats"].get("gated", None))


def job(spec):
    """One filter run. spec: dict(dataset, overrides, contact kwargs, slip, engine)."""
    prm, pol = load_tuned()
    prm = replace(prm, **spec.get("overrides", {}))
    log = _log(spec["dataset"])
    ck = dict(liftoff_advance_ms=pol.liftoff_advance_ms, use_schedule=pol.use_schedule)
    ck.update(spec.get("contact", {}))
    c = pl.estimator_contacts(log, **ck)
    if spec.get("slip", 0) > 0 or spec.get("engine") == "python":
        off = pl.slip_injection(log, c, spec.get("slip", 0.0))
        est = pl.run_filter(log, params=prm, contact=c, slip_off=off)
    else:
        with tempfile.TemporaryDirectory() as wd:
            est = pl.run_cpp(log, params=prm, contact=c, workdir=wd)
    out = dict(spec)
    out.update(_metrics(log, est))
    return out


def run_all(specs):
    with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
        return list(ex.map(job, specs))


def make_friction_datasets(mus):
    base = _log("headline_seed0")
    todo = [mu for mu in mus if not (pl.ROOT / "data" / f"friction_{mu}.npz").exists()]
    with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
        for mu, lg in zip(todo, ex.map(_sim_friction, todo)):
            pl.save_dataset(lg, pl.ROOT / "data" / f"friction_{mu}.npz")
    del base


def _sim_friction(mu):
    return pl.make_dataset(60.0, seed=0, profile=command_profile, floor_friction=mu)


# --------------------------------------------------------------------------- experiments
def e1_hard_vs_soft():
    feet = [1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 1e-1]
    specs = [dict(exp="E1", dataset="headline_seed0", overrides=dict(foot_noise=f), slip=s, foot_noise=f,
                  engine="python" if s else "cpp")
             for s in (0.0, 0.1) for f in feet]
    return run_all(specs)


def e2_slip():
    slips = [0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5]
    variants = {"hard (1e-4 m/√s)": dict(foot_noise=1e-4),
                "tuned (1e-3 m/√s)": dict(),
                "soft (3e-2 m/√s)": dict(foot_noise=3e-2),
                "tuned + χ² gate": dict(gate_chi2=CHI2_3DOF_999)}
    specs = [dict(exp="E2", dataset="headline_seed0", overrides=o, slip=s, variant=v, engine="python")
             for v, o in variants.items() for s in slips]
    return run_all(specs)


def e3_friction():
    mus = [0.8, 0.5, 0.35, 0.25, 0.18, 0.12]
    make_friction_datasets(mus)
    specs = []
    for mu in mus:
        for v, o in {"tuned": dict(), "tuned + χ² gate": dict(gate_chi2=CHI2_3DOF_999),
                     "soft (3e-2 m/√s)": dict(foot_noise=3e-2)}.items():
            specs.append(dict(exp="E3", dataset=f"friction_{mu}", mu=mu, overrides=o, variant=v, engine="cpp"))
    rows = run_all(specs)
    for r in rows:   # measured slip on that floor, for the x-axis
        lg = _log(r["dataset"])
        c = pl.estimator_contacts(lg)
        vf = np.gradient(lg["foot_w"], lg["dt"], axis=0)
        xy = np.concatenate([np.linalg.norm(vf[4000:, i, :2], axis=1)[c[4000:, i]] for i in range(4)])
        r["stance_foot_speed_median_mps"] = float(np.median(xy))
        r["stance_foot_speed_p90_mps"] = float(np.percentile(xy, 90))
    return rows


def e4_contact_delay():
    delays = [0, 2, 5, 10, 15, 20, 30, 40, 60]
    variants = {"force only": dict(use_schedule=False, liftoff_advance_ms=0),
                "force AND schedule": dict(use_schedule=True),
                "force only + χ² gate": dict(use_schedule=False, liftoff_advance_ms=0, _gate=True)}
    specs = []
    for v, ck in variants.items():
        ck = dict(ck)
        gate = ck.pop("_gate", False)
        for d in delays:
            specs.append(dict(exp="E4", dataset="headline_seed0", variant=v, delay_ms=d,
                              contact=dict(ck, delay_ms=d),
                              overrides=dict(gate_chi2=CHI2_3DOF_999) if gate else {}, engine="cpp"))
    return run_all(specs)


def e5_process_noise():
    imu = ImuSpec()
    dt = 1e-3
    tut = np.sqrt(0.01 / dt)  # continuous PSD whose per-step variance is 0.01 -> Q_d = 0.01 I
    variants = {
        "datasheet (BMI088)": dict(),
        "tutorial Q = 0.01·I per step": dict(acc_noise=tut, gyro_noise=tut, acc_rw=tut, gyro_rw=tut),
        "datasheet ÷ 10 (overconfident)": dict(acc_noise=imu.acc_noise_density / 10,
                                                gyro_noise=imu.gyro_noise_density / 10,
                                                acc_rw=imu.acc_rrw / 10, gyro_rw=imu.gyro_rrw / 10),
        "datasheet × 10 (timid)": dict(acc_noise=imu.acc_noise_density * 10, gyro_noise=imu.gyro_noise_density * 10,
                                       acc_rw=imu.acc_rrw * 10, gyro_rw=imu.gyro_rrw * 10),
    }
    specs = [dict(exp="E5", dataset="headline_seed0", variant=v, overrides=o, engine="cpp") for v, o in variants.items()]
    # the datasheet Q against an IMU that also has warm-up drift (5 K, tempco 0.015 deg/s/K)
    if not (pl.ROOT / "data" / "thermal.npz").exists():
        base = _log("headline_seed0")
        pl.save_dataset(pl.make_dataset(60.0, seed=0, imu=ImuSpec(thermal_dT=5.0, thermal_tau=60.0), sim_log=base),
                        pl.ROOT / "data" / "thermal.npz")
    specs.append(dict(exp="E5", dataset="thermal", variant="datasheet Q, IMU with 5 K warm-up drift", overrides={},
                      engine="cpp"))
    return run_all(specs)


# --------------------------------------------------------------------------- figures
def figures(R):
    import matplotlib.pyplot as plt
    ps.setup()
    # E1
    rows = R["E1"]
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for slip, col, lab in ((0.0, ps.S1, "no slip"), (0.1, ps.S2, "0.1 m/s injected slip")):
        rr = sorted([r for r in rows if r["slip"] == slip], key=lambda r: r["foot_noise"])
        x = [r["foot_noise"] for r in rr]
        axs[0].loglog(x, [r["vel_rmse"] for r in rr], "-o", color=col, ms=5, label=lab)
        axs[1].loglog(x, [r["nees_v_median"] for r in rr], "-o", color=col, ms=5, label=lab)
    axs[1].axhline(3.0, color=ps.MUTED, lw=0.9)
    axs[1].text(1.1e-4, 3.3, "consistent = 3", color=ps.INK2, fontsize=9)
    axs[0].set_title("Hard vs soft contact: accuracy")
    axs[1].set_title("…and honesty of the covariance")
    axs[0].set_ylabel("velocity RMSE (m/s)")
    axs[1].set_ylabel("median velocity NEES")
    for ax in axs:
        ax.set_xlabel("foot random-walk noise (m/√s)   ← hard · soft →")
        ax.legend(loc="center left" if ax is axs[0] else "upper right")
    ps.save(fig, "exp_hard_vs_soft", dict(
        foot_noise=[r["foot_noise"] for r in rows], slip=[r["slip"] for r in rows],
        vel_rmse=[r["vel_rmse"] for r in rows], nees_v_median=[r["nees_v_median"] for r in rows]))
    # E2
    rows = R["E2"]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    variants = list(dict.fromkeys(r["variant"] for r in rows))
    for v, col in zip(variants, (ps.S2, ps.S1, ps.S3, ps.S4)):
        rr = sorted([r for r in rows if r["variant"] == v], key=lambda r: r["slip"])
        ax.plot([r["slip"] for r in rr], [r["vel_rmse"] for r in rr], "-o", color=col, ms=5, label=v)
    xs = np.linspace(0.005, 0.5, 200)   # dense: a 2-point segment is not y = x on a log axis
    ax.plot(xs, xs, color=ps.MUTED, lw=0.9, zorder=0)
    ax.text(0.40, 0.27, "velocity error = slip speed", color=ps.INK2, fontsize=9)
    ps.note(ax, "hard, tuned and tuned + gate lie on top of each other:\n"
                "a smooth slide is 0.1 mm of innovation per step, so the χ² gate never fires",
            xy=(0.99, 0.06), ha="right")
    ax.texts[-1].set_va("bottom")
    ax.set_yscale("log")
    ax.set_xlabel("injected foot slip speed during stance (m/s)")
    ax.set_ylabel("velocity RMSE (m/s)")
    ax.set_title("Where the planted-foot assumption breaks")
    ax.legend(loc="upper left")
    ps.save(fig, "exp_slip", dict(variant=[variants.index(r["variant"]) for r in rows], slip=[r["slip"] for r in rows],
                                  vel_rmse=[r["vel_rmse"] for r in rows]))
    # E3
    rows = R["E3"]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    variants = list(dict.fromkeys(r["variant"] for r in rows))
    for v, col in zip(variants, (ps.S1, ps.S4, ps.S3)):
        rr = sorted([r for r in rows if r["variant"] == v], key=lambda r: -r["mu"])
        ax.plot([r["mu"] for r in rr], [r["vel_rmse"] for r in rr], "-o", color=col, ms=5, label=v)
    ax.invert_xaxis()
    ax.set_yscale("log")
    ax.set_ylabel("velocity RMSE (m/s)")
    ax.set_title("Physical slip: same walk re-simulated on slipperier floors")
    rr = sorted([r for r in rows if r["variant"] == variants[0]], key=lambda r: -r["mu"])
    ax.set_xticks([r["mu"] for r in rr])
    ax.set_xticklabels([f"{r['mu']:g}\n{r['stance_foot_speed_median_mps'] * 1000:.0f}" for r in rr])
    ax.set_xlabel("foot friction μ  /  measured median stance-foot slide speed (mm/s)")
    ax.legend(loc="upper left")
    ps.save(fig, "exp_friction", dict(mu=[r["mu"] for r in rows], variant=[variants.index(r["variant"]) for r in rows],
                                      vel_rmse=[r["vel_rmse"] for r in rows],
                                      foot_speed=[r["stance_foot_speed_median_mps"] for r in rows]))
    # E4
    rows = R["E4"]
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.4))
    variants = list(dict.fromkeys(r["variant"] for r in rows))
    for v, col in zip(variants, (ps.S2, ps.S1, ps.S4)):
        rr = sorted([r for r in rows if r["variant"] == v], key=lambda r: r["delay_ms"])
        d = [r["delay_ms"] for r in rr]
        axs[0].plot(d, [r["vel_rmse"] for r in rr], "-o", color=col, ms=5, label=v)
        axs[1].plot(d, [max(abs(r["yaw_final_err_deg"]), 1e-2) for r in rr], "-o", color=col, ms=5, label=v)
    for ax in axs:
        ax.set_yscale("log")
        ax.set_xlabel("contact detection latency (ms), touchdown and liftoff")
    axs[0].set_ylabel("velocity RMSE (m/s)")
    axs[1].set_ylabel("|yaw error| after 60 s (deg)")
    axs[0].set_title("Velocity degrades steadily…", fontsize=11)
    axs[1].set_title("…yaw is where force-only detection falls off", fontsize=11)
    axs[1].legend(loc="center right")
    fig.suptitle("Contact-detection latency: ANDing with the gait schedule removes the failure", x=0.01, ha="left",
                 fontsize=12, fontweight="bold", color=ps.INK)
    fig.tight_layout()
    ps.save(fig, "exp_contact_delay", dict(variant=[variants.index(r["variant"]) for r in rows],
                                           delay_ms=[r["delay_ms"] for r in rows], vel_rmse=[r["vel_rmse"] for r in rows],
                                           yaw_final_deg=[r["yaw_final_err_deg"] for r in rows]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None, help="subset, e.g. E1 E4")
    ap.add_argument("--figures-only", action="store_true", help="re-render figures from results/experiments.json")
    a = ap.parse_args()
    if a.figures_only:
        figures(json.loads((RES / "experiments.json").read_text()))
        return
    path = RES / "experiments.json"
    R = json.loads(path.read_text()) if path.exists() else {}
    fns = dict(E1=e1_hard_vs_soft, E2=e2_slip, E3=e3_friction, E4=e4_contact_delay, E5=e5_process_noise)
    for k, fn in fns.items():
        if a.only is None or k in a.only:
            print(f"running {k} ...", flush=True)
            R[k] = fn()
    R["provenance"] = pl.provenance()
    path.write_text(json.dumps(R, indent=1, default=float))
    if all(k in R for k in ("E1", "E2", "E3", "E4")):
        figures(R)
    for k in ("E1", "E2", "E3", "E4", "E5"):
        for r in R.get(k, []):
            keys = [x for x in ("variant", "foot_noise", "slip", "mu", "delay_ms") if x in r]
            print(k, " ".join(f"{x}={r[x]}" for x in keys),
                  f"vel_rmse={r['vel_rmse']:.4f} nees={r['nees_v_median']:.1f} tilt>12s={r['tilt_rmse_after_12s_deg']:.3f}"
                  f" yaw_final={r['yaw_final_err_deg']:+.2f} pos_final={r['pos_final_err_m']:.2f}")


if __name__ == "__main__":
    main()
