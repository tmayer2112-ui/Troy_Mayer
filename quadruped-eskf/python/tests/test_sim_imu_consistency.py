"""Pins DEBUG_LOG #1, #5 and #6: sensor/ground-truth alignment in the simulator.

If someone re-enables eulerdamp, changes gravity, or records ground truth after
mj_step instead of before, these fail.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import kinematics as K  # noqa: E402
from qeskf.sim import load_model, simulate  # noqa: E402
from qeskf.so3 import quat_to_rot  # noqa: E402

G = np.array([0.0, 0.0, -9.80665])


def test_accelerometer_matches_ground_truth_velocity():
    log = simulate(4.0)
    dt = log["dt"]
    ks = np.arange(2500, 3990)
    R = np.array([quat_to_rot(q) for q in log["quat"][ks]])
    a_fd = (log["v"][ks + 1] - log["v"][ks]) / dt
    a_imu = np.einsum("nij,nj->ni", R, log["acc_true"][ks]) + G
    assert np.sqrt(np.mean((a_fd - a_imu) ** 2)) < 1e-9


def test_free_joint_linear_velocity_is_world_frame():
    log = simulate(3.0)
    v_fd = (log["p"][1:] - log["p"][:-1]) / log["dt"]
    assert np.abs(v_fd[2200:2900] - log["v"][2201:2901]).max() < 1e-9


def test_forward_kinematics_matches_mujoco():
    import mujoco
    m = load_model()
    d = mujoco.MjData(m)
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = np.array([rng.uniform(-0.5, 0.5), rng.uniform(0.2, 1.5), rng.uniform(-2.5, -1.0)] * 4)
        d.qpos[:] = 0
        d.qpos[3] = 1
        d.qpos[7:] = q
        mujoco.mj_kinematics(m, d)
        for i, n in enumerate(K.LEGS):
            np.testing.assert_allclose(K.foot_position(i, q[3 * i:3 * i + 3]), d.site(n).xpos, atol=1e-12)
