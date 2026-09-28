"""Check the analog numbers the emulation depends on. Needs no dataset.

Writes results/board.json and results/figures/filter_response.png.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import freqs, freqz

from emgdec import board as B
from emgdec.data import FS

OUT = Path("results")


def cascade_analog(f):
    h = np.ones_like(f, dtype=complex)
    for f0, kind in ((B.HPF_F0, "high"), (B.LPF_F0, "low")):
        h *= freqs(*B.sallen_key_analog(f0, kind), worN=2 * np.pi * f)[1]
    return h


def cascade_digital(f, fs=FS):
    h = np.ones_like(f, dtype=complex)
    for f0, kind in ((B.HPF_F0, "high"), (B.LPF_F0, "low")):
        h *= freqz(*B.sallen_key(f0, kind, fs), worN=f, fs=fs)[1]
    return h


def minus_3db(f, h):
    mag = np.abs(h) / np.abs(h).max()
    above = f[mag >= 10 ** (-3 / 20)]
    return float(above.min()), float(above.max())


def main():
    f = np.logspace(0, np.log10(FS / 2 * 0.999), 4000)
    ha, hd = cascade_analog(f), cascade_digital(f)
    lo_a, hi_a = minus_3db(f, ha)
    lo_d, hi_d = minus_3db(f, hd)

    v1_rows = []
    for z in (0, 5e3, 20e3, 50e3, 100e3):
        b = B.BoardV1(electrodes=B.Electrodes(z, 0.25))
        v1_rows.append({"z_electrode_ohm": z, "diff_gain": round(b.front_gain, 2),
                        "cmrr_db": round(-20 * np.log10(b.cm_to_diff), 1),
                        "input_for_0.4V_at_max_pot_uV":
                            round(0.4 / (b.front_gain * B.K_SK ** 2 * 148) * 1e6, 1),
                        "input_for_0.4V_at_min_pot_uV":
                            round(0.4 / (b.front_gain * B.K_SK ** 2 * 48) * 1e6, 1)})
    v2 = B.BoardV2()
    res = {
        "sallen_key": {"K": round(B.K_SK, 4), "Q": round(B.Q_SK, 4),
                       "hpf_f0_hz": round(B.HPF_F0, 2), "lpf_f0_hz": round(B.LPF_F0, 1)},
        "cascade_passband_gain": round(float(np.abs(ha).max()), 3),
        "cascade_minus3db_hz_analog": [round(lo_a, 1), round(hi_a, 1)],
        "cascade_minus3db_hz_digital_fs2048": [round(lo_d, 1), round(hi_d, 1)],
        "max_digital_vs_analog_error_db_below_400hz": round(float(np.max(np.abs(
            20 * np.log10(np.abs(hd[f < 400]) / np.abs(ha[f < 400]))))), 3),
        "v1_front_end_vs_electrode_impedance": v1_rows,
        "v2": {"ina_gain": round(v2.ina_gain, 3),
               "effective_cmrr_db_incl_rld": round(-20 * np.log10(v2.cm_to_diff), 1),
               "gain_range": [round(v2.ina_gain * B.K_SK ** 2 * g, 0) for g in (11, 111)]},
    }
    OUT.mkdir(exist_ok=True)
    (OUT / "board.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))

    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.semilogx(f, 20 * np.log10(np.abs(ha)), label="analog circuit", lw=2)
    ax.semilogx(f, 20 * np.log10(np.abs(hd)), "--", label=f"emulation, fs = {FS} Hz")
    for x in (lo_a, hi_a):
        ax.axvline(x, color="0.6", lw=0.8)
    ax.set(xlabel="Hz", ylabel="gain, dB", xlim=(1, FS / 2), ylim=(-40, 12),
           title=f"HPF + LPF: -3 dB at {lo_a:.0f} Hz and {hi_a:.0f} Hz, passband x{np.abs(ha).max():.2f}")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    (OUT / "figures").mkdir(exist_ok=True)
    fig.savefig(OUT / "figures" / "filter_response.png", dpi=150)


if __name__ == "__main__":
    main()
