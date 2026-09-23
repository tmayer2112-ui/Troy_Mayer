"""One look for every figure. Palette = validated categorical order (blue, orange, aqua, yellow);
hairline solid grid; text in ink colors, never series colors. Every figure also writes a CSV with
the plotted numbers next to the PNG, so no value is readable only from a picture."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"   # blue, orange, aqua, yellow
TRUTH = "#0b0b0b"
WASH = "#f0efec"

FIG_DIR = Path(__file__).resolve().parents[2] / "results" / "figures"


def setup():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "axes.titlesize": 12, "axes.titleweight": "semibold", "axes.titlelocation": "left",
        "axes.labelsize": 10.5, "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 9.5,
        "ytick.labelsize": 9.5, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
        "grid.linestyle": "-", "axes.spines.top": False, "axes.spines.right": False,
        "lines.linewidth": 1.6, "lines.solid_capstyle": "round", "legend.frameon": False,
        "legend.fontsize": 9.5, "legend.labelcolor": INK2, "font.size": 10.5, "figure.dpi": 110,
        "savefig.dpi": 160, "font.family": "DejaVu Sans",
    })


def save(fig, name, table: dict = None):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{name}.png", bbox_inches="tight")
    plt.close(fig)
    if table:
        cols = list(table.keys())
        arr = np.column_stack([np.asarray(table[c], dtype=float) for c in cols])
        np.savetxt(FIG_DIR / f"{name}.csv", arr, delimiter=",", header=",".join(cols), comments="", fmt="%.6g")


def note(ax, text, xy=(0.99, 0.96), ha="right"):
    ax.text(*xy, text, transform=ax.transAxes, ha=ha, va="top", color=INK2, fontsize=9.5)
