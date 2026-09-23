"""Write the Python<->C++ parity fixture used by cpp/tests/test_parity.cpp.

Takes 3 s of the headline dataset (walking, contacts switching), runs the Python
reference filter on it and stores its posterior every 10th sample.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402

N_SAMPLES = 3000
EVERY = 10
OUT = pl.ROOT / "cpp" / "tests" / "data"


def main():
    log = pl.load_dataset(pl.ROOT / "data" / "headline_seed0.npz")
    contact = pl.estimator_contacts(log)
    f, k0 = pl.init_filter(log, pl.ImuSpec(), pl.EskfParams.from_imu_spec(pl.ImuSpec()))
    OUT.mkdir(parents=True, exist_ok=True)
    pl.write_cpp_config(OUT / "parity_config.txt", f, f.prm, log["dt"], extra={"every": EVERY})
    pl.export_csv(log, contact, OUT / "parity_sensors.csv", k0=k0, k1=k0 + N_SAMPLES)
    rows = []
    for j in range(N_SAMPLES):
        k = k0 + j
        p, v, q, ba, bg, Pd = f.step(log["acc"][k], log["gyro"][k], log["qenc"][k], contact[k], log["dt"])
        if j % EVERY == 0:
            rows.append(np.r_[j, p, v, q, ba, bg, Pd, len(f.x.order)])
    hdr = "k,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz," + ",".join(f"P{i}" for i in range(15)) + ",nfeet"
    np.savetxt(OUT / "parity_python.csv", np.array(rows), delimiter=",", header=hdr, comments="", fmt="%.17g")
    print(f"wrote fixture: {N_SAMPLES} samples, {len(rows)} reference rows, "
          f"augment={f.stats['augment']} marginalize={f.stats['marginalize']}")


if __name__ == "__main__":
    main()
