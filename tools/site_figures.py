"""Draw the plots shown on the website, in plain matplotlib style.

Every number comes from the CSV each project saved next to its own figure
(plus the summary JSON for the values printed on a plot). Nothing is
re-simulated, so these show exactly the data behind the project figures.

    python tools/site_figures.py        # writes figures/*.png

Needs numpy and matplotlib.
"""
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
ESKF = ROOT / "quadruped-eskf" / "results"
FW = ROOT / "firmware" / "results"
OUT = ROOT / "figures"

plt.rcParams.update(matplotlib.rcParamsDefault)   # ignore any machine-wide matplotlibrc
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "figure.dpi": 100,
    "savefig.dpi": 200,
    "savefig.facecolor": "white",
    "axes.grid": True,
    "grid.alpha": 0.3,
    "font.size": 9.5,
    "axes.titlesize": 9.5,
    "legend.fontsize": 8,
})


def read(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def save(fig, name):
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / f"{name}.png", bbox_inches="tight")
    plt.close(fig)
    print("wrote", f"figures/{name}.png")


def velocity_tracking():
    d = read(ESKF / "figures" / "velocity_tracking.csv")
    rmse = json.load(open(ESKF / "metrics.json"))["contact_aided"]["vel_rmse_world_xyz"]
    t = d["t"]
    z0, z1 = 20.0, 23.0
    zm = (t >= z0) & (t <= z1)
    fig = plt.figure(figsize=(7.2, 6.0))
    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1], hspace=0.38, wspace=0.08)
    left = right = None
    for i, ax_name in enumerate("xyz"):
        axl = fig.add_subplot(gs[i, 0], sharex=left)
        axr = fig.add_subplot(gs[i, 1], sharex=right, sharey=axl)
        left, right = left or axl, right or axr
        true, est = d[f"v{ax_name}_true"], d[f"v{ax_name}_est"]
        axl.plot(t, true, "k", lw=0.5)
        axl.plot(t, est, "C0", lw=0.5)
        axl.axvspan(z0, z1, color="0.6", alpha=0.15, lw=0)
        axr.plot(t[zm], true[zm], "k", lw=1.1, label="ground truth (MuJoCo)")
        axr.plot(t[zm], est[zm], "C0", lw=1.1, label="contact-aided ESKF")
        axl.set_ylabel(f"v{ax_name} (m/s)")
        axl.set_title(f"v{ax_name}: RMSE {rmse[i]:.3f} m/s over the full 60 s", loc="left")
        axr.tick_params(labelleft=False)
        if i < 2:
            axl.tick_params(labelbottom=False)
            axr.tick_params(labelbottom=False)
    right.set_title(f"zoom: {z0:.0f}–{z1:.0f} s")
    right.legend(loc="upper center", fontsize=7, handlelength=1.5)
    fig.axes[-2].set_xlabel("time (s)")
    fig.axes[-1].set_xlabel("time (s)")
    save(fig, "velocity_tracking")


def covariance_observability():
    d = read(ESKF / "figures" / "covariance_observability.csv")
    t = d["t"]
    win = 20  # 1 s moving average (samples are 0.05 s apart), as in the project figure

    def smooth(x):
        xp = np.pad(x, (win // 2, win - 1 - win // 2), mode="edge")
        return np.convolve(xp, np.ones(win) / win, mode="valid")

    fig, axs = plt.subplots(2, 2, figsize=(7.2, 5.4), sharex=True)
    panels = [   # (axes, title, y label, (CSV column index, label) pairs, smooth?)
        (axs[0, 0], "Position", "1σ (m)", [(0, "x"), (1, "y"), (2, "z")], False),
        (axs[0, 1], "Yaw", "1σ (deg)", [(8, "yaw")], False),
        (axs[1, 0], "Velocity", "1σ (m/s)", [(3, "x"), (4, "y"), (5, "z")], True),
        (axs[1, 1], "Roll and pitch", "1σ (deg)", [(6, "roll"), (7, "pitch")], True),
    ]
    for ax, title, ylabel, series, sm in panels:   # CSV columns are already in m, m/s and deg
        for j, (idx, lab) in enumerate(series):
            y = d[f"sigma_{idx}"]
            ax.plot(t, smooth(y) if sm else y, f"C{j}", lw=1.2, label=lab)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_ylim(bottom=0)
    axs[0, 0].legend(loc="center left", fontsize=7.5)
    axs[1, 0].set_ylim(0, 0.0095)
    axs[1, 0].legend(loc="upper right", ncol=3, fontsize=7.5, handlelength=1.5)
    axs[1, 1].set_ylim(0, 0.12)
    axs[1, 1].legend(loc="upper right", ncol=2, fontsize=7.5, handlelength=1.5)
    axs[1, 1].text(0.97, 0.05, "starts at 1.1° (standing calibration),\noff scale for the first ~15 s",
                   transform=axs[1, 1].transAxes, ha="right", fontsize=7, color="0.35")
    for ax in axs[1]:
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    save(fig, "covariance_observability")


def contact_delay():
    d = read(ESKF / "figures" / "exp_contact_delay.csv")
    rows = json.load(open(ESKF / "experiments.json"))["E4"]
    names = list(dict.fromkeys(r["variant"] for r in rows))
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.3))
    for i, name in enumerate(names):
        m = d["variant"] == i
        order = np.argsort(d["delay_ms"][m])
        delay = d["delay_ms"][m][order]
        axs[0].plot(delay, d["vel_rmse"][m][order], "o-", ms=4, color=f"C{i}", label=name)
        axs[1].plot(delay, np.maximum(np.abs(d["yaw_final_deg"][m][order]), 1e-2), "o-", ms=4, color=f"C{i}",
                    label=name)
    for ax in axs:
        ax.set_yscale("log")
        ax.set_xlabel("contact detection latency (ms)")
    axs[0].set_ylabel("velocity RMSE (m/s)")
    axs[1].set_ylabel("|yaw error| after 60 s (deg)")
    axs[1].legend(loc="upper left", fontsize=7.5)
    fig.tight_layout()
    save(fig, "exp_contact_delay")


def closed_loop_lean():
    d = read(ESKF / "figures" / "closed_loop_lean.csv")
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.3), sharey=True)
    for ax, run, title in ((axs[0], "headline", "headline walk (first turn at t ≈ 12 s)"),
                           (axs[1], "turn_first", "same walk, 4 s turn at startup")):
        for j, (fb, lab) in enumerate((("truth", "controller fed ground truth"),
                                        ("estimate", "controller fed the estimate"))):
            ax.plot(d[f"{run}_{fb}_t"], d[f"{run}_{fb}_roll"], f"C{j}", lw=1.0, label=lab)
        ax.axhline(0, color="0.5", lw=0.8)
        ax.set_title(title)
        ax.set_xlabel("time (s)")
        ax.set_xlim(0, 30)
    axs[0].set_ylabel("true trunk roll (deg)")
    axs[0].legend(loc="lower right", fontsize=7.5)
    fig.tight_layout()
    save(fig, "closed_loop_lean")


def qp_compare():
    d = read(ESKF / "figures" / "qp_compare_velocity.csv")
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True, sharey=True)
    for ax, ctrl, title in ((axs[0], "scripted", "scripted trot (position targets)"),
                            (axs[1], "qp", "QP force balance on the stance legs")):
        ax.plot(d[f"{ctrl}_truth_t"], d[f"{ctrl}_truth_cmd_vx"], "--", color="0.45", lw=1.0, label="command")
        for j, (fb, lab) in enumerate((("truth", "fed ground truth"), ("estimate", "fed the estimate"))):
            vx = np.convolve(d[f"{ctrl}_{fb}_vx"], np.ones(20) / 20, "same")   # 0.4 s moving average
            ax.plot(d[f"{ctrl}_{fb}_t"], vx, f"C{j}", lw=1.0, label=lab)
        ax.set_title(title)
        ax.set_ylabel("forward speed (m/s)")
    axs[1].set_xlabel("time (s); speeds are 0.4 s moving averages")
    axs[0].legend(loc="upper right", ncol=3, fontsize=7.5)
    fig.tight_layout()
    save(fig, "qp_compare_velocity")


def emg_end_to_end():
    d = read(FW / "emg_end_to_end.csv")
    t = d["t"]
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.8), sharex=True)
    axs[0].plot(t, d["act_flex"], "C0", lw=1.0, label="flexor")
    axs[0].plot(t, d["act_ext"], "C1", lw=1.0, label="extensor")
    axs[0].set(ylabel="activation (0–1)", ylim=(-0.05, 1.18))
    for x0, x1, txt in ((1, 3, "flex"), (4, 5, "both = stop"), (6, 8, "extend")):
        axs[0].text((x0 + x1) / 2, 1.08, txt, ha="center", fontsize=8)
    axs[0].legend(loc="center right")
    axs[1].plot(t, d["cmd"], "--", color="0.45", lw=1.0, label="speed command from EMG")
    axs[1].plot(t, d["vel"], "C0", lw=1.0, label="joint speed")
    axs[1].set(ylabel="joint speed (deg/s)")
    axs[1].legend(loc="upper right")
    axs[2].plot(t, d["pos"], "C0", lw=1.0)
    axs[2].set(ylabel="joint angle (deg)", xlabel="time (s)", xlim=(0, 10))
    fig.tight_layout()
    save(fig, "emg_end_to_end")


def card_trajectory():
    """Thumbnail for the project card: the walked path, truth against the estimate (16:10, no title)."""
    d = read(ESKF / "figures" / "trajectory_xy.csv")
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(d["x_est"], d["y_est"], "C0", lw=3.0, label="contact-aided ESKF")
    ax.plot(d["x_true"], d["y_true"], "k", lw=1.0, label="ground truth")
    ax.set_aspect("equal", "datalim")
    ax.set(xlabel="x (m)", ylabel="y (m)")
    ax.legend(loc="best", fontsize=10)
    fig.tight_layout(pad=0.6)
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "card_trajectory.png")   # fixed 16:10 canvas, so it fills the card frame
    plt.close(fig)
    print("wrote figures/card_trajectory.png")


if __name__ == "__main__":
    card_trajectory()
    velocity_tracking()
    covariance_observability()
    contact_delay()
    closed_loop_lean()
    qp_compare()
    emg_end_to_end()
