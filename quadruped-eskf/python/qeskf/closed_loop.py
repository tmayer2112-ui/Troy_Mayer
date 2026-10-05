"""Closed loop: the trot controller driven by the estimator instead of by ground truth.

`simulate()` in sim.py feeds the controller ground truth and the estimator runs afterwards on the
log. Here the estimator runs inside the 1 kHz physics loop and, with feedback="estimate", its
output is what the controller sees. Everything the controller and estimator read is causal:

  per physics step k (t_k = k * dt)
    1. encoders q_k (quantized), contact flags from the force sensed during step k-1 (1 ms old)
       AND the planner's stance window, ended `liftoff_advance_ms` early (as in the open-loop runs)
    2. estimator update at t_k (touchdown/liftoff bookkeeping + kinematic update)
    3. controller (500 Hz) reads either ground truth or the estimate, sets joint targets
    4. mj_step -> IMU sample k (from the state at t_k, plus BMI088 noise), contact force at t_k
    5. estimator predict t_k -> t_{k+1} with IMU sample k

What the controller consumes: body-frame velocity, roll, pitch, body rates and trunk height.
All of these are observable from IMU + legs. It never uses position or yaw, which drift.

Before T_START the robot stands still while the estimator calibrates (as in the open-loop runs).
In estimate mode there is no attitude estimate yet, so the controller holds a fixed stance:
zero velocity, level attitude, zero rates, and height from leg kinematics. (A first version fed it
roll/pitch from the low-passed accelerometer; the settling transient corrupted that and the robot
fell. See DEBUG_LOG #13.) No ground truth reaches the controller at any time.
"""
from dataclasses import dataclass

import mujoco
import numpy as np

from . import kinematics as K
from .config import ContactPolicy
from .eskf import Eskf, EskfParams, initial_covariance, static_alignment
from .gait import GaitParams, TrotController, command_profile
from .imu_model import EncoderSpec, ImuSpec, imu_noise, measure_imu, quantize_encoders
from .pipeline import CAL_WINDOW, CONTACT_FORCE_N, T_START
from .sim import _mean_stance_foot_z, load_model
from .so3 import quat_to_rot, rot_to_quat, rpy_from_rot

FALL_HEIGHT = 0.12      # m, trunk height below which the run counts as a fall (same as sim.py)
FALL_TILT = np.radians(60.0)


def straight_profile(vx=0.4, ramp=3.0, stand_time=2.0):
    """Stand, ramp to vx, walk straight. Used for the push tests."""
    def profile(t):
        a = np.clip((t - stand_time) / ramp, 0.0, 1.0)
        return np.array([vx * a, 0.0, 0.0])
    profile.__name__ = f"straight_{vx}"
    return profile


def turn_first(base=command_profile, t_turn=4.0, stand_time=2.0, vx=0.2, wz=0.5):
    """Stand, turn left then right for t_turn s while walking slowly, then run `base` shifted by t_turn.

    Turning makes tilt separable from accelerometer bias (README, Observability), so the estimate's
    roll/pitch converge before the robot starts relying on them for a long straight walk.
    """
    def profile(t):
        if t < stand_time:
            return np.zeros(3)
        if t < stand_time + t_turn:
            s = 1.0 if t < stand_time + t_turn / 2 else -1.0
            a = min((t - stand_time) / 0.5, 1.0)
            return np.array([vx * a, 0.0, s * wz * a])
        return base(t - t_turn)
    profile.__name__ = f"turn_first_{base.__name__}"
    return profile


@dataclass
class LoopResult:
    feedback: str
    seed: int
    fell: bool
    t_fall: float
    log: dict   # 1 kHz arrays, truncated at the fall if there was one


def _feet_height(R, qenc):
    """Trunk height above the two lowest feet, from leg kinematics and an attitude estimate."""
    z = np.sort([(R @ K.foot_position(i, qenc[3 * i:3 * i + 3]))[2] for i in range(4)])
    return -0.5 * (z[0] + z[1])


def run(duration=60.0, feedback="estimate", seed=0, profile=command_profile, gait=GaitParams(),
        params: EskfParams = None, policy: ContactPolicy = ContactPolicy(liftoff_advance_ms=5.0),
        imu=ImuSpec(), enc=EncoderSpec(), push=None, contact_delay_ms=0.0):
    """One closed-loop run. feedback: "truth" (controller reads the simulator) or "estimate".

    The estimator runs in both modes, so "truth" mode also gives the estimator's accuracy on the
    same physics, for comparison. push: (t_start, t_end, force_world[3]) on the trunk.
    contact_delay_ms: extra latency on the force-based contact signal (on top of the 1 ms above).
    """
    assert feedback in ("truth", "estimate")
    params = params or EskfParams.from_imu_spec(imu)
    m = load_model()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    ctrl = TrotController(gait)
    dt = m.opt.timestep
    n = int(round((T_START + duration) / dt))
    k0 = int(round(T_START / dt))
    cal = slice(int(CAL_WINDOW[0] / dt), int(CAL_WINDOW[1] / dt))
    trunk = m.body("trunk").id
    foot_geoms = [m.geom(nm).id for nm in K.LEGS]
    foot_sites = [m.site(nm).id for nm in K.LEGS]
    floor = m.geom("floor").id
    s_acc = m.sensor("accel").adr[0]
    s_gyr = m.sensor("gyro").adr[0]

    ba_n, bg_n, wa_n, wg_n = imu_noise(n, imu, np.random.default_rng(seed))
    adv = int(round(policy.liftoff_advance_ms / 1000.0 / dt))
    delay = int(round(contact_delay_ms / 1000.0 / dt))

    log = {k: np.full((n,) + s, np.nan) for k, s in dict(
        t=(), p=(3,), quat=(4,), v=(3,), omega_b=(3,), cmd=(3,), height=(),
        est_p=(3,), est_v=(3,), est_q=(4,), fb_v_b=(3,), fb_rpy=(3,), fb_height=(),
        acc=(3,), gyro=(3,), contact_est=(4,),
    ).items()}
    force_hist = np.zeros((n, 4), dtype=bool)
    f = None
    q_cmd = d.ctrl.copy()
    fb = None
    fell, t_fall, k_end = False, np.nan, n
    for k in range(n):
        t = k * dt
        R_true = quat_to_rot(d.qpos[3:7])
        v_w = d.qvel[0:3].copy()
        w_b = d.qvel[3:6].copy()
        qenc = quantize_encoders(d.qpos[7:19], enc)
        cmd = profile(t)

        # --- estimator: initialize at k0 from the standing window, then update at t_k ---
        if k == k0:
            yaw0 = rpy_from_rot(R_true)[2]   # nav frame = true pose at k0 (position/yaw unobservable)
            R0, ba0, bg0 = static_alignment(log["acc"][cal], log["gyro"][cal], yaw0)
            f = Eskf(params, p=d.qpos[0:3].copy(), v=np.zeros(3), q=rot_to_quat(R0), ba=ba0, bg=bg0,
                     P0=initial_covariance(imu, n_cal=cal.stop - cal.start))
        if f is not None:
            j = k - 1 - delay
            force = force_hist[j] if j >= 0 else force_hist[0]
            sched = ctrl.scheduled_stance(t) & ctrl.scheduled_stance(t + adv * dt)
            contact = force & sched if policy.use_schedule else force
            new = f.handle_contacts(contact, qenc)
            f.update(qenc, skip=new)
            log["contact_est"][k] = contact
            log["est_p"][k], log["est_v"][k], log["est_q"][k] = f.x.p, f.x.v, f.x.q

        # --- controller at 500 Hz ---
        if k % 2 == 0:
            if feedback == "truth":
                fb = (R_true.T @ v_w, rpy_from_rot(R_true), w_b,
                      d.qpos[2] - _mean_stance_foot_z(d, foot_sites))
            elif f is None:   # standing calibration: no estimate yet -> hold a fixed stance
                fb = (np.zeros(3), np.zeros(3), np.zeros(3), _feet_height(np.eye(3), qenc))
            else:
                Rh = quat_to_rot(f.x.q)
                w = log["gyro"][k - 1] - f.x.bg   # latest gyro sample, bias-corrected
                fb = (Rh.T @ f.x.v, rpy_from_rot(Rh), w, _feet_height(Rh, qenc))
            q_cmd = ctrl.joint_targets(t, cmd, *fb)
        d.ctrl[:] = q_cmd
        d.xfrc_applied[trunk] = 0
        if push is not None and push[0] <= t < push[1]:
            d.xfrc_applied[trunk, :3] = push[2]

        log["t"][k] = t
        log["p"][k], log["quat"][k], log["v"][k], log["omega_b"][k] = d.qpos[0:3], d.qpos[3:7], v_w, w_b
        log["cmd"][k] = cmd
        log["height"][k] = d.qpos[2] - _mean_stance_foot_z(d, foot_sites)
        log["fb_v_b"][k], log["fb_rpy"][k], log["fb_height"][k] = fb[0], fb[1], fb[3]

        mujoco.mj_step(m, d)

        acc, gyr = measure_imu(d.sensordata[s_acc:s_acc + 3], d.sensordata[s_gyr:s_gyr + 3],
                               ba_n[k], bg_n[k], wa_n[k], wg_n[k], imu)
        log["acc"][k], log["gyro"][k] = acc, gyr
        fn = np.zeros(4)
        fc = np.zeros(6)
        for c in range(d.ncon):
            con = d.contact[c]
            pair = {con.geom1, con.geom2}
            if floor not in pair:
                continue
            for i, g in enumerate(foot_geoms):
                if g in pair:
                    mujoco.mj_contactForce(m, d, c, fc)
                    fn[i] += fc[0]
        force_hist[k] = fn > CONTACT_FORCE_N
        if f is not None:
            f.predict(acc, gyr, dt)

        rpy = rpy_from_rot(quat_to_rot(d.qpos[3:7]))
        if d.qpos[2] < FALL_HEIGHT or abs(rpy[0]) > FALL_TILT or abs(rpy[1]) > FALL_TILT:
            fell, t_fall, k_end = True, t, k + 1
            break
    log = {k: v[:k_end] for k, v in log.items()}
    log["dt"] = dt
    log["k0"] = k0
    return LoopResult(feedback, seed, fell, t_fall, log)


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def metrics(res: LoopResult, t_from=T_START + 3.0, straight=(T_START + 3.0, T_START + 10.0)):
    """Tracking and estimation metrics over [t_from, end] (after the 3 s speed ramp).

    straight: a window of straight walking before the first turn, where tilt is only weakly observable.
    """
    L = res.log
    sel = L["t"] >= t_from
    out = dict(feedback=res.feedback, seed=res.seed, fell=res.fell,
               t_fall=None if not res.fell else float(res.t_fall))
    if sel.sum() < 10:
        return out
    R = np.array([quat_to_rot(q) for q in L["quat"][sel]])
    v_b = np.einsum("nji,nj->ni", R, L["v"][sel])
    cmd = L["cmd"][sel]
    rpy = np.array([rpy_from_rot(Ri) for Ri in R])
    Re = np.array([quat_to_rot(q) for q in L["est_q"][sel]])
    rpy_e = np.array([rpy_from_rot(Ri) for Ri in Re])
    out.update(
        # tracking: what the robot actually did vs the command (ground truth, both modes)
        track_vxy_rmse=_rms(np.linalg.norm(v_b[:, :2] - cmd[:, :2], axis=1)),
        track_wz_rmse=_rms(L["omega_b"][sel][:, 2] - cmd[:, 2]),
        roll_rms_deg=float(np.degrees(_rms(rpy[:, 0]))),
        pitch_rms_deg=float(np.degrees(_rms(rpy[:, 1]))),
        height_rms_err_mm=1000 * _rms(L["height"][sel] - GaitParams().height),
        # estimator accuracy on this run
        est_vel_rmse=float(np.sqrt(np.mean(np.sum((L["est_v"][sel] - L["v"][sel]) ** 2, 1)))),
        est_roll_rmse_deg=float(np.degrees(_rms(rpy_e[:, 0] - rpy[:, 0]))),
        est_pitch_rmse_deg=float(np.degrees(_rms(rpy_e[:, 1] - rpy[:, 1]))),
        # what the controller was fed vs truth (zero in truth mode)
        fb_v_b_rmse=_rms(np.linalg.norm(L["fb_v_b"][sel] - v_b, axis=1)),
        fb_height_rmse_mm=1000 * _rms(L["fb_height"][sel] - L["height"][sel]),
    )
    ws = (L["t"] >= straight[0]) & (L["t"] < straight[1])
    if ws.sum():
        Rw = np.array([quat_to_rot(q) for q in L["quat"][ws]])
        Rew = np.array([quat_to_rot(q) for q in L["est_q"][ws]])
        tw = np.degrees([rpy_from_rot(x)[:2] for x in Rw])
        ew = np.degrees([rpy_from_rot(x)[:2] for x in Rew]) - tw
        # mean (not RMS): a lean is a steady offset; the gait's own sway averages out
        out.update(straight_lean_deg=float(np.hypot(*tw.mean(0))),
                   straight_est_tilt_err_deg=float(np.hypot(*ew.mean(0))))
    return out
