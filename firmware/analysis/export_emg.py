"""Export real forearm EMG, as the v2 board's ESP32 would see it, for the latency study.

Runs the LibEMG MultiDay recordings (see orthosis-emg-decoder/) through the v2
board model, resamples to the firmware's 1 kHz and converts to ADC counts.
For every (day, gesture) it keeps the channel with the most activity, so each
trial is one real sustained contraction.

    python analysis/export_emg.py            # writes analysis/data/contractions.npz
    python analysis/export_emg.py --subset   # writes sim/data/real_contractions.bin
"""
import sys
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "orthosis-emg-decoder"))
from emgdec.data import FS, load_all          # noqa: E402
from emgdec.pipeline import run_chain         # noqa: E402

ADC_FS_V, ADC_BITS = 3.1, 12


def main():
    sig, info = run_chain(load_all(), "v2")
    out = {}
    for (day, cls), x in sorted(sig.items()):
        if cls == 0:                       # "no motion" is not rest in this dataset
            continue
        ch = int(np.argmax(x.std(axis=0)))
        v = resample_poly(x[:, ch].astype(np.float64), 1000, FS)
        counts = np.clip(np.round((v + 1.65) / ADC_FS_V * 2 ** ADC_BITS), 0, 4095).astype(np.int16)
        out[f"d{day}_c{cls}"] = counts
    dest = Path(__file__).with_name("data")
    dest.mkdir(exist_ok=True)
    np.savez_compressed(dest / "contractions.npz", **out)
    print(f"{len(out)} contractions, pots {info['pot_ohms']}")


if __name__ == "__main__" and len(sys.argv) == 1:
    main()


def export_subset(n=24, seconds=2.0, seed=7):
    """A small fixed subset for the C++ end-to-end latency test (committed, ~100 KB):
    magic 'EMG1', uint32 count, then per contraction uint32 length + int16 samples."""
    import struct
    z = np.load(Path(__file__).with_name("data") / "contractions.npz")
    keys = sorted(z.files)
    pick = np.random.default_rng(seed).choice(keys, size=n, replace=False)
    dest = Path(__file__).resolve().parents[1] / "sim" / "data"
    dest.mkdir(exist_ok=True)
    with open(dest / "real_contractions.bin", "wb") as f:
        f.write(b"EMG1" + struct.pack("<I", n))
        for k in pick:
            x = z[k][: int(seconds * 1000)].astype("<i2")
            f.write(struct.pack("<I", len(x)) + x.tobytes())
    print("subset:", ", ".join(pick))


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "--subset":
    export_subset()
