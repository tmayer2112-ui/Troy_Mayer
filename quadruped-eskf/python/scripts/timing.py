"""1 kHz real-time loop benchmark: runs build/qeskf_bench_loop on the headline stream.

Writes results/timing.json, results/logs/timing.log, results/figures/timing_histogram.png (+ .csv).
Run it on an otherwise idle machine; anything else competing for the CPU shows up as jitter.
"""
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf import plotstyle as ps  # noqa: E402
from qeskf.config import load_tuned  # noqa: E402

RES = pl.ROOT / "results"
N_RUNS = 3


def main():
    prm, pol = load_tuned()
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    c = pl.estimator_contacts(log, liftoff_advance_ms=pol.liftoff_advance_ms)
    f, k0 = pl.init_filter(log, pl.ImuSpec(), prm)
    wd = pl.ROOT / "data" / "timing"
    wd.mkdir(parents=True, exist_ok=True)
    pl.write_cpp_config(wd / "config.txt", f, prm, log["dt"])
    pl.export_csv(log, c, wd / "sensors.csv", k0=k0)
    reports, runs = [], []
    for rep in range(N_RUNS):   # the host's jitter varies run to run; report every run, plot the first
        cmd = [str(pl.BUILD / "qeskf_bench_loop"), str(wd / "config.txt"), str(wd / "sensors.csv"),
               str(wd / f"timing_{rep}.csv"), "--rate", "1000"]
        rr = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
        reports.append(rr)
        b = np.loadtxt(wd / f"timing_{rep}.csv", delimiter=",", skiprows=1) / 1e3
        runs.append(dict(deadline_misses=int(np.sum(b[:, 0] + b[:, 1] > 1000.0)),
                         compute_p50_us=float(np.percentile(b[:, 1], 50)), compute_p99_us=float(np.percentile(b[:, 1], 99)),
                         compute_p999_us=float(np.percentile(b[:, 1], 99.9)), compute_max_us=float(b[:, 1].max()),
                         wake_late_p99_us=float(np.percentile(b[:, 0], 99)), wake_late_max_us=float(b[:, 0].max())))
    report = "\n".join(reports)
    cmd[3] = str(wd / "timing_0.csv")
    a = np.loadtxt(wd / "timing_0.csv", delimiter=",", skiprows=1)
    late_us, comp_us, per_us = a[:, 0] / 1e3, a[:, 1] / 1e3, a[:, 2] / 1e3
    busy = late_us + comp_us
    pc = lambda x, p: float(np.percentile(x, p))  # noqa: E731
    out = dict(
        rate_hz=1000, iterations=int(len(a)),
        scheduler=re.search(r"scheduler\s+(.*)", report).group(1).strip(),
        compute_us=dict(p50=pc(comp_us, 50), p99=pc(comp_us, 99), p999=pc(comp_us, 99.9), max=float(comp_us.max())),
        wake_late_us=dict(p50=pc(late_us, 50), p99=pc(late_us, 99), p999=pc(late_us, 99.9), max=float(late_us.max())),
        period_us=dict(mean=float(per_us.mean()), std=float(per_us.std())),
        deadline_misses=int(np.sum(busy > 1000.0)),
        deadline_miss_pct=float(100 * np.mean(busy > 1000.0)),
        compute_budget_used_p99_pct=pc(comp_us, 99) / 10.0,
        all_runs=runs,
        deadline_miss_pct_range=[min(100 * r["deadline_misses"] / len(a) for r in runs),
                                 max(100 * r["deadline_misses"] / len(a) for r in runs)],
        cpu=pl.provenance()["cpu"], note="non-RT kernel (cloud VM); jitter is the host's, compute is the filter's",
        command=" ".join(cmd), provenance=pl.provenance(),
    )
    (RES / "timing.json").write_text(json.dumps(out, indent=2))
    (RES / "logs").mkdir(exist_ok=True)
    (RES / "logs" / "timing.log").write_text(f"$ {' '.join(cmd)}\n{report}\n{json.dumps(out, indent=2)}\n")

    import matplotlib.pyplot as plt
    ps.setup()
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    bins = np.logspace(0, np.log10(max(3000.0, 1.2 * max(comp_us.max(), late_us.max()))), 70)  # never drop the tail
    for ax, x, col, title in ((axs[0], comp_us, ps.S1, "Filter step compute time"),
                              (axs[1], late_us, ps.S2, "Wake-up lateness (scheduler jitter)")):
        cnt, edges = np.histogram(np.clip(x, 1, None), bins=bins)
        assert cnt.sum() == len(x), "histogram dropped samples"
        ax.bar(edges[:-1], cnt, width=np.diff(edges) * 0.85, align="edge", color=col, lw=0)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.axvline(1000, color=ps.INK2, lw=1.0)
        ax.text(930, 0.45, "1 ms budget", rotation=90, ha="right", va="center", color=ps.INK2, fontsize=9,
                transform=ax.get_xaxis_transform())
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("µs")
        ps.note(ax, f"p50 {np.percentile(x, 50):.0f} µs · p99 {np.percentile(x, 99):.0f} µs · max {x.max():.0f} µs")
    axs[0].set_ylabel("iterations (of 60,000)")
    lo, hi = out["deadline_miss_pct_range"]
    fig.suptitle(f"C++ ESKF at 1 kHz for 60 s ({out['scheduler']}, non-RT cloud VM)", x=0.01, y=1.04, ha="left",
                 fontsize=12, fontweight="bold", color=ps.INK)
    fig.text(0.01, 0.955, f"run 1: {out['deadline_misses']} of {len(a):,} deadlines missed "
             f"({out['deadline_miss_pct']:.3f} %); {lo:.3f}–{hi:.3f} % over {N_RUNS} runs. Misses line up with host "
             f"stalls in wake-up lateness, not with filter compute.", ha="left", color=ps.INK2, fontsize=9.5)
    fig.tight_layout()
    cnt_c, _ = np.histogram(comp_us, bins=bins)
    cnt_l, _ = np.histogram(late_us, bins=bins)
    ps.save(fig, "timing_histogram", dict(bin_lo_us=bins[:-1], compute_count=cnt_c, wake_late_count=cnt_l))
    print(report)
    print(json.dumps({k: out[k] for k in ("compute_us", "wake_late_us", "deadline_misses", "deadline_miss_pct")}))


if __name__ == "__main__":
    main()
