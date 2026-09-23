"""Run the Go1 in MuJoCo at 1 kHz and record ground truth + ideal sensor signals.

Timing convention (this matters, see DEBUG_LOG): mj_step evaluates sensors on
the state at the *start* of the step, then integrates. So each logged row k
pairs the state x_k (recorded before mj_step) with the accelerometer/gyro that
MuJoCo computed from x_k during that step. Recording qpos *after* mj_step
instead shifts ground truth one sample ahead of the IMU.
"""
from pathlib import Path

import mujoco
import numpy as np

from . import kinematics as K
from .gait import GaitParams, TrotController, command_profile
from .so3 import quat_to_rot, rpy_from_rot

MODEL_DIR = Path(__file__).resolve().parents[2] / "models" / "go1"


def _mean_stance_foot_z(d, foot_sites):
    # Trunk height is measured relative to the lowest two feet (flat ground), foot-sphere centres.
    z = np.sort(d.site_xpos[foot_sites, 2])
    return 0.5 * (z[0] + z[1])


def load_model(floor_friction=None):
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "scene.xml"))
    if floor_friction is not None:
        # Foot geoms have priority=1, so their friction wins; scale those.
        for n in K.LEGS:
            m.geom(n).friction[0] = floor_friction
    return m


def simulate(duration=60.0, profile=command_profile, gait=GaitParams(), floor_friction=None,
             contact_threshold=5.0, push=None):
    """Returns a dict of numpy arrays sampled at 1 kHz (one row per physics step).

    push: optional (t_start, t_end, force_world[3]) applied to the trunk, for robustness tests.
    """
    m = load_model(floor_friction)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    ctrl = TrotController(gait)
    dt = m.opt.timestep
    n = int(round(duration / dt))
    trunk = m.body("trunk").id
    foot_geoms = [m.geom(nm).id for nm in K.LEGS]
    foot_sites = [m.site(nm).id for nm in K.LEGS]
    floor = m.geom("floor").id
    s_acc = m.sensor("accel").adr[0]
    s_gyr = m.sensor("gyro").adr[0]

    log = {k: np.zeros((n,) + s) for k, s in dict(
        t=(), p=(3,), quat=(4,), v=(3,), omega_b=(3,), acc_true=(3,), gyro_true=(3,),
        qj=(12,), dqj=(12,), fn=(4,), contact=(4,), foot_w=(4, 3), cmd=(3,), sched=(4,),
    ).items()}
    fc = np.zeros(6)
    q_cmd = d.ctrl.copy()
    ctrl_decim = 2  # controller at 500 Hz, physics + IMU at 1 kHz
    for k in range(n):
        t = k * dt
        # --- ground truth at the start of the step ---
        R = quat_to_rot(d.qpos[3:7])
        v_w = d.qvel[0:3].copy()      # free joint: linear velocity in WORLD frame
        w_b = d.qvel[3:6].copy()      # free joint: angular velocity in BODY frame
        cmd = profile(t)
        if k % ctrl_decim == 0:
            v_b = R.T @ v_w
            rpy = rpy_from_rot(R)
            q_cmd = ctrl.joint_targets(t, cmd, v_b, rpy, w_b, d.qpos[2] - _mean_stance_foot_z(d, foot_sites))
        d.ctrl[:] = q_cmd
        d.xfrc_applied[trunk] = 0
        if push is not None and push[0] <= t < push[1]:
            d.xfrc_applied[trunk, :3] = push[2]

        log["t"][k] = t
        log["p"][k] = d.qpos[0:3]
        log["quat"][k] = d.qpos[3:7]
        log["v"][k] = v_w
        log["omega_b"][k] = w_b
        log["qj"][k] = d.qpos[7:19]
        log["dqj"][k] = d.qvel[6:18]
        log["foot_w"][k] = d.site_xpos[foot_sites]
        log["cmd"][k] = cmd
        log["sched"][k] = ctrl.scheduled_stance(t)

        mujoco.mj_step(m, d)

        # Sensors and contacts were evaluated on the pre-step state -> same row.
        log["acc_true"][k] = d.sensordata[s_acc:s_acc + 3]
        log["gyro_true"][k] = d.sensordata[s_gyr:s_gyr + 3]
        fn = np.zeros(4)
        for c in range(d.ncon):
            con = d.contact[c]
            pair = {con.geom1, con.geom2}
            if floor not in pair:
                continue
            for i, g in enumerate(foot_geoms):
                if g in pair:
                    mujoco.mj_contactForce(m, d, c, fc)
                    fn[i] += fc[0]
        log["fn"][k] = fn
        log["contact"][k] = fn > contact_threshold

        if d.qpos[2] < 0.12:
            raise RuntimeError(f"robot fell at t={t:.3f}s (trunk z={d.qpos[2]:.3f})")
    log["dt"] = dt
    return log
