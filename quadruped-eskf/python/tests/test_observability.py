"""The analytic unobservable directions (3 translations + yaw) are exactly the null space."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import pipeline as pl  # noqa: E402
from qeskf import observability as ob  # noqa: E402
from qeskf.imu_model import quantize_encoders, EncoderSpec  # noqa: E402
from qeskf.sim import simulate  # noqa: E402


def test_nullspace_is_translation_plus_yaw():
    log = simulate(5.0)
    log["qenc"] = quantize_encoders(log["qj"], EncoderSpec())
    c = pl.estimator_contacts(log)
    k0, legs = ob.stance_window(c, 3000, 100)
    O, N = ob.observability_matrix(log, k0, legs, 100)
    # (a) analytic directions are in the null space
    rel = np.linalg.norm(O @ N, axis=0) / (np.linalg.norm(O) * np.linalg.norm(N, axis=0))
    assert rel.max() < 1e-12
    # (b) and there are no others
    sc = np.linalg.norm(O, axis=0)
    s = np.linalg.svd(O / sc, compute_uv=False)
    s = s / s[0]
    assert np.sum(s < 1e-9) == 4
    assert s[-5] > 1e-8   # the 5th direction (tilt/accel-bias pair) is weak but observable
