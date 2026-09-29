"""Figures for firmware/README.md from the CSVs sim_demo writes.

    python sim/plot.py results
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"   # validated categorical slots 1-3

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
    "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False, "legend.fontsize": 8, "lines.linewidth": 1.6,
})


def read(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def label_end(ax, x, y, text, color):
    ax.annotate(text, (x[-1], y[-1]), xytext=(4, 0), textcoords="offset points",
                va="center", fontsize=8, color=INK)
    ax.plot([x[-1]], [y[-1]], "o", ms=4, color=color)


def step_load(d, out):
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    ax.plot(d["t"], d["ref"], "--", color=INK_2, lw=1.2, label="speed command")
    ax.plot(d["t"], d["vel_ff_only"], color=ORANGE, label="feed-forward only")
    ax.plot(d["t"], d["vel_closed"], color=BLUE, label="PI + feed-forward (this firmware)")
    label_end(ax, d["t"], d["vel_closed"], "closed loop", BLUE)
    label_end(ax, d["t"], d["vel_ff_only"], "feed-forward only", ORANGE)
    ax.set(xlabel="time (s)", ylabel="joint speed (deg/s)", xlim=(0, 1.85))
    ax.set_title("Lifting the forearm at 15 deg/s: feedback removes the load droop")
    ax.legend(loc="upper left", bbox_to_anchor=(0.30, 0.78))
    fig.tight_layout()
    fig.savefig(out / "step_load.png", dpi=150)


def emg_end_to_end(d, out):
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 6.2), sharex=True)
    t = d["t"]
    a = axes[0]
    a.plot(t, d["act_flex"], color=BLUE, label="flexor")
    a.plot(t, d["act_ext"], color=ORANGE, label="extensor")
    a.set(ylabel="activation (0-1)", ylim=(-0.05, 1.1))
    a.set_title("Synthetic EMG through the whole controller")
    a.legend(loc="upper right", ncol=2)
    for x0, x1, txt in [(1, 3, "flex"), (4, 5, "co-contract = stop"), (6, 8, "extend")]:
        a.annotate(txt, ((x0 + x1) / 2, 1.04), ha="center", fontsize=8, color=INK_2)
    b = axes[1]
    b.plot(t, d["cmd"], "--", color=INK_2, lw=1.2, label="command from EMG")
    b.plot(t, d["vel"], color=BLUE, label="joint speed")
    b.set(ylabel="deg/s")
    b.legend(loc="upper right", ncol=2)
    c = axes[2]
    c.plot(t, d["pos"], color=BLUE)
    c.set(ylabel="joint angle (deg)", xlabel="time (s)", xlim=(0, 10))
    fig.tight_layout()
    fig.savefig(out / "emg_end_to_end.png", dpi=150)


def estimator(d, out):
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    ax.plot(d["t"], d["count_diff_100hz"], color=ORANGE, lw=1.2, label="count difference at 100 Hz (old)")
    ax.plot(d["t"], d["edge_timing"], color=BLUE, label="edge timing over one quadrature cycle (new)")
    ax.plot(d["t"], d["true"], "--", color=INK_2, lw=1.2, label="true speed (model)")
    ax.set(xlabel="time (s)", ylabel="joint speed (deg/s)", xlim=(0, 1.5))
    ax.set_title("Speed estimate from a 28-count encoder during a slow ramp")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(out / "estimator.png", dpi=150)


if __name__ == "__main__":
    res = Path(sys.argv[1] if len(sys.argv) > 1 else "results")
    figs = res / "figures"
    figs.mkdir(parents=True, exist_ok=True)
    step_load(read(res / "step_load.csv"), figs)
    emg_end_to_end(read(res / "emg_end_to_end.csv"), figs)
    estimator(read(res / "estimator.csv"), figs)
    print("wrote", ", ".join(p.name for p in sorted(figs.glob("*.png"))))
