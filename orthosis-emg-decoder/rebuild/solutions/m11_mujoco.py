"""Module 11 (optional): your speed loop on a MuJoCo elbow. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.

A hinge (the elbow) carries a 2 kg forearm with its centre of mass 17.8 cm out, so gravity is 3.5 N m at
most, the firmware's limb_mgr. The motor isn't a MuJoCo motor: each step, your motor_torque() turns
the PI loop's duty into joint torque with the DC-motor equations, and MuJoCo applies it. The numbers are
the firmware's plant (firmware/sim/plant.h). Questions from your project:
  A. Does the loop you wrote in Module 6 behave the same on a different simulator?
  B. What does each MuJoCo sensor actually report? (The joint sensors are in radians.)
"""
import numpy as np

import m6_speed_loop

# firmware/sim/plant.h
VBUS, R_OHM, KE = 12.0, 1.30, 0.0186           # V, ohm, V s/rad (= Kt in N m/A)
N = 5.2 * 1.0 * 2 * np.pi * 0.0235 / 0.004     # motor rad per joint rad: 191.95
ETA = 0.87 * 0.90                              # gearbox x ball screw
J_MOTOR = 3.0e-6 + 0.5e-6                      # rotor + gearbox, kg m^2 at the motor
MASS, COM = 2.0, 3.5 / (2.0 * 9.80665)         # 2 kg at 0.178 m: m g r = 3.5 N m
LIMB_I = 0.085                                 # kg m^2 about the elbow
FRICTION_JOINT = 0.25 * KE * N                 # the no-load current's torque, at the joint: 0.89 N m
ELBOW_AT_ZERO = 45.0                           # deg: the elbow angle when the joint reads 0

XML = f"""
<mujoco model="rebuild: the orthosis elbow">
  <option timestep="0.0002" gravity="0 0 -9.80665"/>
  <worldbody>
    <body name="forearm">
      <joint name="elbow" type="hinge" axis="0 1 0" armature="{J_MOTOR * N ** 2:.6f}"
             frictionloss="{FRICTION_JOINT:.6f}"/>
      <inertial pos="{-COM * np.sin(np.radians(ELBOW_AT_ZERO)):.6f} 0 {-COM * np.cos(np.radians(ELBOW_AT_ZERO)):.6f}"
                mass="{MASS}" diaginertia="{LIMB_I - MASS * COM ** 2:.6f} {LIMB_I - MASS * COM ** 2:.6f} {LIMB_I - MASS * COM ** 2:.6f}"/>
      <geom type="capsule" fromto="0 0 0 {-2 * COM * np.sin(np.radians(ELBOW_AT_ZERO)):.4f} 0 {-2 * COM * np.cos(np.radians(ELBOW_AT_ZERO)):.4f}"
            size="0.03" mass="0" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="elbow" gear="1"/>
  </actuator>
  <sensor>
    <jointpos name="elbow_pos" joint="elbow"/>
    <jointvel name="elbow_vel" joint="elbow"/>
  </sensor>
</mujoco>
"""


def motor_torque(duty, omega_joint):
    """Joint torque (N m) from the driver's duty, with the joint turning at omega_joint (rad/s).
      motor speed  w_m = N omega_joint
      current      i   = (duty VBUS - KE w_m) / R_OHM      (the back-EMF KE w_m opposes the supply)
      joint torque     = ETA N KE i                         (Kt = KE; the transmission multiplies by N
                                                             and loses 1 - ETA of it)
    (Worksheet 11.1)"""
    # TODO (2 lines)
    # --- solution ---
    i = (duty * VBUS - KE * N * omega_joint) / R_OHM
    return ETA * N * KE * i
    # --- end ---


def joint_speed_dps(d):
    """The elbow's speed in deg/s, read from the jointvel sensor named "elbow_vel".
    What unit does MuJoCo report it in?   (Worksheet 11.2)"""
    # TODO (1 line): d.sensor("elbow_vel").data[0] is the reading
    # --- solution ---
    return float(np.degrees(d.sensor("elbow_vel").data[0]))
    # --- end ---


# ------------------------------------------------------------------ provided from here down
def simulate(closed_loop, command, theta0_deg, seconds, ctrl_dt=1e-3):
    """Your Module 6 VelocityPI at 1 kHz on the MuJoCo elbow (5 physics steps per control tick).
    Returns arrays t, ref, speed (deg/s), angle (deg) and duty, one per control tick."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    d.qpos[0] = np.radians(theta0_deg)
    mujoco.mj_forward(m, d)
    ctrl = m6_speed_loop.VelocityPI(closed_loop=closed_loop)
    ramp = m6_speed_loop.Ramp(150.0)
    sub = int(round(ctrl_dt / m.opt.timestep))
    n = int(round(seconds / ctrl_dt))
    log = {k: np.zeros(n) for k in ("t", "ref", "speed", "angle", "duty")}
    for k in range(n):
        speed = joint_speed_dps(d)
        ref, ramping = ramp.step(command, ctrl_dt)
        duty = ctrl.step(ref, speed, ctrl_dt, ramping)
        for key, v in (("t", k * ctrl_dt), ("ref", ref), ("speed", speed),
                       ("angle", float(np.degrees(d.sensor("elbow_pos").data[0]))), ("duty", duty)):
            log[key][k] = v
        for _ in range(sub):
            d.ctrl[0] = motor_torque(duty, d.qvel[0])
            mujoco.mj_step(m, d)
    return log


def gravity_torque_at(theta_deg):
    """MuJoCo's gravity torque on the elbow at a joint angle, for the check: it should be
    -3.5 sin(theta + 45 deg) N m, the firmware plant's formula."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    d.qpos[0] = np.radians(theta_deg)
    mujoco.mj_forward(m, d)
    return -float(d.qfrc_bias[0])


def demo():
    print("A) Your Module 6 loop on a MuJoCo elbow (same numbers as the firmware's plant)")
    for closed in (False, True):
        lg = simulate(closed, 15.0, 5.0, 1.5)
        err = float(np.mean(15.0 - lg["speed"][-500:]))
        print(f"   lift at 15 deg/s, {'feed-forward + PI' if closed else 'feed-forward only':18s}: "
              f"error {err:6.2f} deg/s ({100 * err / 15:5.1f} %)")
    for closed in (False, True):
        lg = simulate(closed, 0.0, 40.0, 5.5)
        print(f"   hold at 40 deg, 5 s,  {'feed-forward + PI' if closed else 'feed-forward only':18s}: "
              f"drift {lg['angle'][-1] - lg['angle'][500]:+8.3f} deg")
    print("   The firmware's own simulation: 98.3 % and 0.8 % lifting, 0.000 deg drift holding")
    print("   (make test in firmware/; firmware/results/metrics.json). The hold differs, and why: MuJoCo's")
    print("   frictionloss is a soft constraint, not stiction, so the arm keeps creeping while the integrator")
    print("   catches up (it settles after about 5 s). The firmware's plant has stiction, which stops it dead.")
    print("\nB) What the sensors report: jointpos and jointvel are in RADIANS (rad, rad/s).")
    print(f"   gravity torque at joint 0 deg: {gravity_torque_at(0.0):+.3f} N m "
          f"(plant.h: -3.5 sin 45 deg = {-3.5 * np.sin(np.radians(45)):+.3f})")


if __name__ == "__main__":
    demo()
