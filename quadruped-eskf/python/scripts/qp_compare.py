"""Scripted trot vs QP force-balance controller, both closed loop on the estimator, same metrics as closed_loop.py.

QP1 headline   the 60 s headline walk, controller fed ground truth vs the estimate, sensor seeds 0-4
QP2 turn_first the same walk after a 4 s left/right turn at startup, seeds 0-4
QP3 push       largest lateral push (0.1 s) survived walking straight at 0.4 m/s, bisection, seed 0

The QP controller is qeskf/qp_balance.py with its default QPGains. The scripted numbers are read from
results/closed_loop.json (run `make closed-loop` first), so they are exactly the ones in the README; only
the two seed-0 headline traces for the figure are re-simulated. Writes results/qp_compare.json,
results/logs/qp_compare.log and results/figures/qp_compare_velocity.png (+ .csv).
"""
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import closed_loop as CL  # noqa: E402  (python/scripts/closed_loop.py: the CL1-CL3 experiment)
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.pipeline import ROOT, provenance  # noqa: E402
from qeskf.qp_balance import QPGains  # noqa: E402

RES = ROOT / "results"
EXPS = ("headline", "turn_first")
FBS = ("truth", "estimate")
KEYS = [("track_vxy_rmse", "velocity tracking RMSE vs command (m/s)", "{:.3f}"),
        ("track_wz_rmse", "yaw-rate tracking RMSE (rad/s)", "{:.3f}"),
        ("straight_lean_deg", "mean lean on the first straight segment (deg)", "{:.2f}"),
        ("straight_est_tilt_err_deg", "estimate tilt error, same segment (deg)", "{:.2f}"),
        ("roll_rms_deg", "true roll RMS (deg)", "{:.2f}"),
        ("pitch_rms_deg", "true pitch RMS (deg)", "{:.2f}"),
        ("height_rms_err_mm", "height error RMS (mm)", "{:.2f}"),
        ("est_vel_rmse", "estimator velocity RMSE (m/s)", "{:.4f}")]


def _push_qp(feedback):
    return CL._push(feedback, "qp")


def _summary(rows):
    out = {}
    for name in EXPS:
        for fb in FBS:
            rr = [r for r in rows if r["exp"] == name and r["feedback"] == fb]
            ok = [r for r in rr if not r["fell"]]
            s = {k: float(np.mean([r[k] for r in ok])) for k, _, _ in KEYS} if ok else {}
            for k, _, _ in KEYS:
                if len(ok) > 1:
                    s[k + "_sd"] = float(np.std([r[k] for r in ok], ddof=1))
                    s[k + "_min"], s[k + "_max"] = float(min(r[k] for r in ok)), float(max(r[k] for r in ok))
            s.update(falls=int(sum(r["fell"] for r in rr)), n=len(rr))
            out[f"{name}/{fb}"] = s
    return out


def figure(traces):
    ps.setup()
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 1, figsize=(11.5, 6.4), sharex=True, sharey=True)
    for ax, ctrl, title in ((axs[0], "scripted", "Scripted trot (position targets, stiff servos)"),
                            (axs[1], "qp", "QP force balance on the stance legs")):
        tr = traces[(ctrl, "truth")]
        ax.plot(tr["t"], tr["cmd_vx"], color=ps.AXIS, lw=1.0, ls="--", label="command")
        for fb, col, lab in (("truth", ps.S1, "fed ground truth"), ("estimate", ps.S2, "fed the estimate")):
            tr = traces[(ctrl, fb)]
            ax.plot(tr["t"], np.convolve(tr["vx"], np.ones(20) / 20, "same"), color=col, lw=1.2, label=lab)
        ax.set_title(title, fontsize=11, loc="left")
        ax.set_ylabel("forward speed (m/s)\n0.4 s moving average")
    axs[1].set_xlabel("time (s)")
    axs[0].legend(loc="upper right", ncols=3)
    fig.suptitle("Headline walk, seed 0, in simulation: the QP controller gets closer to the commanded speed",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", color=ps.INK)
    fig.tight_layout()
    cols = {}
    for (ctrl, fb), tr in traces.items():
        cols[f"{ctrl}_{fb}_t"] = tr["t"]
        cols[f"{ctrl}_{fb}_cmd_vx"] = tr["cmd_vx"]
        cols[f"{ctrl}_{fb}_vx"] = tr["vx"]
    n = min(len(v) for v in cols.values())
    ps.save(fig, "qp_compare_velocity", {k: v[:n] for k, v in cols.items()})


def main():
    t0 = time.time()
    src = RES / "closed_loop.json"
    if not src.exists():
        sys.exit("results/closed_loop.json missing: run `make closed-loop` first")
    scripted = json.loads(src.read_text())
    jobs = [(n, fb, s, "qp") for n in EXPS for fb in FBS for s in CL.SEEDS]
    jobs += [("headline", fb, 0, "scripted") for fb in FBS]   # traces for the figure only
    with ProcessPoolExecutor() as ex:
        out = list(ex.map(CL._walk, jobs))
        push = list(ex.map(_push_qp, FBS))
    qp_rows = [m for m, _ in out if m.get("controller") == "qp"]
    traces = {(m.get("controller", "scripted"), m["feedback"]): tr
              for m, tr in out if tr is not None and m["exp"] == "headline"}
    figure(traces)

    result = dict(qp=dict(rows=qp_rows, summary=_summary(qp_rows), push=push),
                  scripted=dict(summary=_summary(scripted["rows"]), push=scripted["push"],
                                source="results/closed_loop.json"),
                  qp_gains=asdict(QPGains()),
                  setup=dict(scripted["setup"], controller_qp="qeskf/qp_balance.py, default QPGains"),
                  provenance=provenance(), wall_time_s=time.time() - t0)
    (RES / "qp_compare.json").write_text(json.dumps(result, indent=1, default=float))

    lines = [f"scripted trot vs QP force balance, in simulation, {CL.DURATION:.0f} s walks, seeds {list(CL.SEEDS)}",
             "mean (min-max over seeds that did not fall); scripted numbers from results/closed_loop.json", ""]
    for name in EXPS:
        lines.append(f"[{name}]")
        for fb in FBS:
            lines.append(f"  controller fed {fb}")
            ss, sq = result["scripted"]["summary"][f"{name}/{fb}"], result["qp"]["summary"][f"{name}/{fb}"]
            lines.append(f"    {'falls':52s} scripted {ss['falls']}/{ss['n']:<16d} qp {sq['falls']}/{sq['n']}")
            for k, lab, fmt in KEYS:
                def cell(s):
                    if k not in s:
                        return "-"
                    c = fmt.format(s[k])
                    return c + (f" ({fmt.format(s[k + '_min'])}-{fmt.format(s[k + '_max'])})" if k + "_min" in s else "")
                lines.append(f"    {lab:52s} scripted {cell(ss):18s} qp {cell(sq)}")
        lines.append("")
    for ctrl, pp in (("scripted", scripted["push"]), ("qp", push)):
        for p in pp:
            lines.append(f"[push] {ctrl:8s} fed {p['feedback']:8s}: survives {p['max_survived_Ns']:.2f} N·s, "
                         f"falls at {p['min_fell_Ns']:.2f} N·s (lateral, {CL.PUSH_LEN} s)")
    lines.append(f"\nwall time {time.time() - t0:.0f} s")
    text = "\n".join(lines)
    (RES / "logs").mkdir(exist_ok=True)
    (RES / "logs" / "qp_compare.log").write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
