"""Closed loop: does the walking controller still work when it is fed the estimate instead of ground truth?

CL1 headline   the 60 s headline walk; controller fed ground truth vs the estimate, sensor seeds 0-4
CL2 turn_first the same walk preceded by a 4 s left/right turn, so tilt is observable before the
               long straight segment; ground truth vs estimate, seeds 0-4
CL3 push       largest lateral push (0.1 s on the trunk) survived while walking straight at 0.4 m/s
               (after the 4 s turn), found by bisection; ground truth vs estimate, seed 0

The controller (qeskf/gait.py) and estimator (tuned params, results/tuned_params.json) are unchanged;
only what the controller reads changes. Writes results/closed_loop.json, results/logs/closed_loop.log
and results/figures/closed_loop_lean.png (+ .csv).
"""
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import closed_loop as cl  # noqa: E402
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402
from qeskf.gait import command_profile  # noqa: E402
from qeskf.pipeline import ROOT, T_START, provenance  # noqa: E402
from qeskf.so3 import quat_to_rot, rpy_from_rot  # noqa: E402

RES = ROOT / "results"
SEEDS = range(5)
DURATION = 60.0
T_TURN = 4.0
PUSH_T = 9.0            # s, push starts here (straight walking at 0.4 m/s by then)
PUSH_LEN = 0.1          # s
PUSH_HI = 20.0          # N*s, upper bracket for the bisection
PUSH_RES = 0.25         # N*s


def _profile(name):
    if name == "headline":
        return command_profile
    if name == "turn_first":
        return cl.turn_first(command_profile, T_TURN)
    return cl.turn_first(cl.straight_profile(0.4), T_TURN)


def _walk(args):
    name, feedback, seed, *ctrl = args       # optional 4th element: controller ("scripted" or "qp")
    controller = ctrl[0] if ctrl else "scripted"
    prm, pol = load_tuned()
    t0 = time.time()
    r = cl.run(DURATION, feedback, seed, profile=_profile(name), params=prm, policy=pol, controller=controller)
    t_walk = T_START + (T_TURN if name == "turn_first" else 0.0)
    m = cl.metrics(r, t_from=t_walk + 3.0, straight=(t_walk + 3.0, t_walk + 10.0))
    m.update(exp=name, wall_s=time.time() - t0)
    if controller != "scripted":
        m["controller"] = controller
    trace = None
    if seed == 0:   # 50 Hz trace for the figure
        L = r.log
        k = np.arange(0, len(L["t"]), 20)
        rt = np.degrees([rpy_from_rot(quat_to_rot(q))[:2] for q in L["quat"][k]])
        re = np.degrees([rpy_from_rot(quat_to_rot(q))[:2] if np.all(np.isfinite(q)) else (np.nan, np.nan)
                         for q in L["est_q"][k]])
        vx = [(quat_to_rot(q).T @ v)[0] for q, v in zip(L["quat"][k], L["v"][k])]
        trace = dict(t=L["t"][k].tolist(), roll=rt[:, 0].tolist(), pitch=rt[:, 1].tolist(),
                     roll_err=(re[:, 0] - rt[:, 0]).tolist(), pitch_err=(re[:, 1] - rt[:, 1]).tolist(),
                     vx=vx, cmd_vx=L["cmd"][k, 0].tolist())
    return m, trace


def _survives(feedback, impulse, controller="scripted"):
    prm, pol = load_tuned()
    f = impulse / PUSH_LEN
    r = cl.run(PUSH_T + 3.0 - T_START, feedback, 0, profile=_profile("push"), params=prm, policy=pol,
               push=(PUSH_T, PUSH_T + PUSH_LEN, np.array([0.0, f, 0.0])), controller=controller)
    return not r.fell


def _push(feedback, controller="scripted"):
    lo, hi, tried = 0.0, PUSH_HI, []
    assert not _survives(feedback, hi, controller), "upper bracket survived; raise PUSH_HI"
    while hi - lo > PUSH_RES:
        mid = 0.5 * (lo + hi)
        ok = _survives(feedback, mid, controller)
        tried.append((mid, ok))
        lo, hi = (mid, hi) if ok else (lo, mid)
    out = dict(feedback=feedback, max_survived_Ns=lo, min_fell_Ns=hi, tried=tried)
    if controller != "scripted":
        out["controller"] = controller
    return out


def figure(rows, traces):
    ps.setup()
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.4), sharey=True)
    for ax, name, title in ((axs[0], "headline", "Headline walk: first turn at t ≈ 12 s"),
                            (axs[1], "turn_first", "Same walk after a 4 s turn at startup")):
        for fb, col, lab in (("truth", ps.S1, "controller fed ground truth"),
                             ("estimate", ps.S2, "controller fed the estimate")):
            tr = traces[(name, fb)]
            ax.plot(tr["t"], tr["roll"], color=col, lw=1.2, label=lab)
        ax.axhline(0, color=ps.AXIS, lw=0.8)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("time (s)")
        ax.set_xlim(0, 30)
    axs[0].set_ylabel("true trunk roll (deg)")
    axs[0].legend(loc="lower right")
    fig.suptitle("Closing the loop: the estimate's tilt error becomes a real lean until a turn makes tilt observable",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", color=ps.INK)
    fig.tight_layout()
    cols = {}
    for (name, fb), tr in traces.items():
        cols[f"{name}_{fb}_t"] = tr["t"]
        cols[f"{name}_{fb}_roll"] = tr["roll"]
        cols[f"{name}_{fb}_roll_err"] = tr["roll_err"]
    n = min(len(v) for v in cols.values())
    ps.save(fig, "closed_loop_lean", {k: v[:n] for k, v in cols.items()})


def _mean_sd(rows, key):
    x = np.array([r[key] for r in rows], float)
    return f"{x.mean():.4g} ± {x.std(ddof=1):.2g}" if len(x) > 1 else f"{x[0]:.4g}"


def main():
    t0 = time.time()
    jobs = [(n, fb, s) for n in ("headline", "turn_first") for fb in ("truth", "estimate") for s in SEEDS]
    with ProcessPoolExecutor() as ex:
        out = list(ex.map(_walk, jobs))
        push = list(ex.map(_push, ("truth", "estimate")))
    rows = [m for m, _ in out]
    traces = {(m["exp"], m["feedback"]): tr for m, tr in out if tr is not None}
    figure(rows, traces)

    summary = {}
    for name in ("headline", "turn_first"):
        for fb in ("truth", "estimate"):
            rr = [r for r in rows if r["exp"] == name and r["feedback"] == fb]
            summary[f"{name}/{fb}"] = {k: float(np.mean([r[k] for r in rr])) for k in rr[0]
                                       if isinstance(rr[0][k], float) and k != "wall_s"}
            summary[f"{name}/{fb}"]["falls"] = int(sum(r["fell"] for r in rr))
            summary[f"{name}/{fb}"]["n"] = len(rr)
    result = dict(rows=rows, summary=summary, push=push,
                  setup=dict(duration_s=DURATION, seeds=list(SEEDS), turn_first_s=T_TURN, push_t=PUSH_T,
                             push_len_s=PUSH_LEN, push_resolution_Ns=PUSH_RES,
                             contact="force sensed 1 ms earlier AND gait schedule ended 5 ms early"),
                  provenance=provenance(), wall_time_s=time.time() - t0)
    (RES / "closed_loop.json").write_text(json.dumps(result, indent=1, default=float))

    lines = [f"closed loop, {DURATION:.0f} s walks, seeds {list(SEEDS)}, mean ± sd over seeds", ""]
    keys = [("fell", None), ("track_vxy_rmse", "velocity tracking RMSE vs command (m/s)"),
            ("track_wz_rmse", "yaw-rate tracking RMSE (rad/s)"),
            ("straight_lean_deg", "mean lean on the first straight segment (deg)"),
            ("straight_est_tilt_err_deg", "estimate tilt error, same segment (deg)"),
            ("roll_rms_deg", "true roll RMS (deg)"), ("pitch_rms_deg", "true pitch RMS (deg)"),
            ("height_rms_err_mm", "height error RMS (mm)"), ("est_vel_rmse", "estimator velocity RMSE (m/s)")]
    for name in ("headline", "turn_first"):
        lines.append(f"[{name}]")
        for fb in ("truth", "estimate"):
            rr = [r for r in rows if r["exp"] == name and r["feedback"] == fb]
            lines.append(f"  controller fed {fb}: falls {sum(r['fell'] for r in rr)}/{len(rr)}")
            for k, lab in keys[1:]:
                lines.append(f"    {lab:52s} {_mean_sd(rr, k)}")
        lines.append("")
    for p in push:
        lines.append(f"[push] controller fed {p['feedback']}: survives {p['max_survived_Ns']:.2f} N·s, "
                     f"falls at {p['min_fell_Ns']:.2f} N·s (lateral, {PUSH_LEN} s)")
    lines.append(f"\nwall time {time.time() - t0:.0f} s")
    text = "\n".join(lines)
    (RES / "logs").mkdir(exist_ok=True)
    (RES / "logs" / "closed_loop.log").write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
