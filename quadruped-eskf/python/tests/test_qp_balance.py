"""QP force-balance controller: standing force distribution, torque sign, friction pyramid."""
import mujoco
import numpy as np

from qeskf import kinematics as K
from qeskf.qp_balance import G_W, QPBalanceController, QPGains
from qeskf.sim import _mean_stance_foot_z, load_model
from qeskf.so3 import quat_to_rot, rpy_from_rot


def _standing():
    m = load_model(torque=True)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d, QPBalanceController(m)


def test_standing_forces_sum_to_weight_and_are_shared():
    m, d, c = _standing()
    c.plan(0.0, np.zeros(3), (np.zeros(3), np.zeros(3), np.zeros(3), c.h_des), d.qpos[7:19].copy())
    assert c.stance.all()
    np.testing.assert_allclose(c.f.sum(0), -c.mass * G_W, atol=1e-3)
    np.testing.assert_allclose(c.mass, mujoco.mj_getTotalmass(m), rtol=1e-9)
    fz = c.f[:, 2]
    assert np.all(np.abs(fz / fz.mean() - 1.0) < 0.05), fz      # ~m g / 4 each (CoM is 2 mm off centre)
    assert np.abs(c.f[:, :2]).max() < 1e-3


def _stand(m, d, c, seconds, flip=False):
    fs = [m.site(n).id for n in K.LEGS]
    for k in range(int(seconds / m.opt.timestep)):
        if k % 2 == 0:
            R = quat_to_rot(d.qpos[3:7])
            fb = (R.T @ d.qvel[:3], rpy_from_rot(R), d.qvel[3:6].copy(), d.qpos[2] - _mean_stance_foot_z(d, fs))
            c.plan(0.0, np.zeros(3), fb, d.qpos[7:19].copy())
            if flip:
                c.tau_ff *= -1
        d.ctrl[:] = c.torque(d.qpos[7:19], d.qvel[6:18])
        mujoco.mj_step(m, d)
    return d.qpos[2] - _mean_stance_foot_z(d, fs)


def test_torque_sign_holds_height():
    m, d, c = _standing()
    h = _stand(m, d, c, 2.0)
    assert abs(h - c.h_des) < 0.01, h
    m, d, c = _standing()
    assert _stand(m, d, c, 2.0, flip=True) < c.h_des - 0.05   # tau = J^T f (wrong sign) collapses


def test_forces_respect_friction_pyramid():
    m, d, c = _standing()
    rng = np.random.default_rng(0)
    q0 = d.qpos[7:19].copy()
    mu = QPGains().mu
    saturated = 0
    for i in range(200):
        t = 2.0 + rng.uniform(0, 1)                      # walking: two or four legs in stance
        big = i % 4 == 0                                 # every 4th: a demand friction can't meet
        cmd = rng.uniform(-1, 1, 3) * (10.0 if big else 0.5)
        fb = (rng.uniform(-0.5, 0.5, 3), np.r_[rng.uniform(-0.2, 0.2, 2), 0.0], rng.uniform(-1, 1, 3),
              c.h_des + rng.uniform(-0.03, 0.03))
        c.plan(t, cmd, fb, q0 + rng.normal(0, 0.05, 12))
        f = c.f[c.stance]
        assert np.all(f[:, 2] >= -1e-9)
        assert np.all(np.abs(f[:, 0]) + np.abs(f[:, 1]) <= mu * f[:, 2] + 1e-6)
        assert np.all(c.f[~c.stance] == 0)
        if big:
            saturated += np.any(np.abs(f[:, 0]) + np.abs(f[:, 1]) > mu * f[:, 2] - 1e-6)
    assert saturated > 0      # the constraint actually binds, so the check above isn't vacuous
