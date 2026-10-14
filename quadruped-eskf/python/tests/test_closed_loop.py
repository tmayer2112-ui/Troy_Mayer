"""Closed-loop wiring: with feedback="truth" the physics must match the open-loop simulator exactly,
and the per-sample IMU model must reproduce the batch one."""
import numpy as np

from qeskf.closed_loop import run
from qeskf.imu_model import ImuSpec, imu_noise, measure_imu, synthesize_imu
from qeskf.sim import simulate


def test_truth_feedback_matches_open_loop_sim():
    r = run(duration=1.5, feedback="truth", seed=0)
    log = simulate(duration=3.5)
    n = len(r.log["t"])
    assert not r.fell
    np.testing.assert_array_equal(r.log["p"], log["p"][:n])
    np.testing.assert_array_equal(r.log["quat"], log["quat"][:n])


def test_estimate_feedback_walks():
    r = run(duration=3.0, feedback="estimate", seed=0)
    assert not r.fell
    k = r.log["k0"]
    assert np.all(np.isfinite(r.log["est_v"][k:]))


def test_per_sample_imu_matches_batch():
    spec = ImuSpec()
    rng = np.random.default_rng(3)
    a = rng.normal(0, 3, (200, 3))
    w = rng.normal(0, 1, (200, 3))
    acc_b, gyr_b, _, _ = synthesize_imu(a, w, spec, np.random.default_rng(7))
    ba, bg, wa, wg = imu_noise(200, spec, np.random.default_rng(7))
    for k in (0, 57, 199):
        acc, gyr = measure_imu(a[k], w[k], ba[k], bg[k], wa[k], wg[k], spec)
        np.testing.assert_array_equal(acc, acc_b[k])
        np.testing.assert_array_equal(gyr, gyr_b[k])
