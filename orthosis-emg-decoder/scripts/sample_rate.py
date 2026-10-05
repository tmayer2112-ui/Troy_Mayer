"""What the firmware's 1 kHz ADC rate costs the decoder. Writes results/sample_rate.json.

Everything else in results/ runs at the dataset's 2048 Hz, but firmware/emg_orthosis
samples the v2 board at 1 kHz (Config::fs_hz). The board's only real anti-alias filter
is the 482 Hz Sallen-Key low-pass (the 4.8 kHz output RC is far above either Nyquist),
so content between 500 Hz and 1 kHz folds back into the band. Three v2 variants,
identical otherwise:

  2048 Hz                      what every other number in results/ assumes
  1000 Hz                      the firmware as written: content above 500 Hz aliases
  1000 Hz, FIR anti-alias      a sharp FIR low-pass at 500 Hz (transition ~420-580 Hz) before
                               the ADC, i.e. what a much steeper analog filter would buy

Windows stay 195 ms long with a 49 ms step (195 and 49 samples at 1 kHz).
"""
import json
import sys
from multiprocessing import Pool
from pathlib import Path

from emgdec import protocols as P
from emgdec.data import FS, load_all
from emgdec.pipeline import build_table, run_chain

OUT = Path("results/sample_rate.json")
VARIANTS = {
    "2048 Hz": {},
    "1000 Hz": {"adc_fs": 1000},
    "1000 Hz, FIR anti-alias": {"adc_fs": 1000, "anti_alias": True},
}

RECS = None


def _init():
    global RECS
    RECS = load_all()


def job(name):
    fs = VARIANTS[name].get("adc_fs", FS)
    sig, info = run_chain(RECS, "v2", **VARIANTS[name])
    T = build_table(sig, fs=fs)
    del sig
    out = {"fs": fs, "info": info, "cross_day": P.cross_day(T)["per_day"], "within_day": P.within_day(T)["per_day"]}
    print(f"{name:26s} cross-day {out['cross_day']['mean']:.3f}  within-day {out['within_day']['mean']:.3f}",
          flush=True)
    return name, out


def main(workers):
    with Pool(min(workers, len(VARIANTS)), initializer=_init) as pool:
        res = dict(pool.map(job, list(VARIANTS), chunksize=1))
    OUT.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 2)
