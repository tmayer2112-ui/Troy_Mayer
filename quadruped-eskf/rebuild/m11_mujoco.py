"""Module 11 (optional): simulate one foot and an IMU in MuJoCo.

  Needs MuJoCo (see README, "Module 11"). Fill in the two TODOs (about 6 lines), then:
  python check.py 11   then:  python m11_mujoco.py

A body with a quarter of the Go1's mass sits on one Go1 foot sphere (same size and the same soft contact
settings, solimp="0.015 1 0.023", as models/go1/go1.xml) and drops 2.7 cm onto the floor. An IMU site
records what the accelerometer reads. Two questions from your project:
  A. What does an accelerometer measure? (Why predict "adds gravity back", Module 4.)
  B. When is a foot "planted"? Force says contact long before the foot stops moving (DEBUG_LOG #3).
"""
import numpy as np

G = 9.80665
GRAVITY = np.array([0.0, 0.0, -G])

XML = """
<mujoco model="rebuild: one foot and an IMU">
  <option timestep="0.001" gravity="0 0 -9.80665"><flag eulerdamp="disable"/></option>
  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane"/>
    <body name="trunk" pos="0 0 0.05">
      <joint name="drop" type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="3.185" diaginertia="0.01 0.01 0.01"/>
      <site name="imu"/>
      <geom name="foot" type="sphere" size="0.023" SOLIMP priority="1" condim="6" friction="0.8 0.02 0.01"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""
GO1_FOOT = 'solimp="0.015 1 0.023"'   # Menagerie's Go1 foot: a soft contact, a stand-in for the rubber
DEFAULT = ""                           # MuJoCo's default contact, much stiffer


def specific_force(R, a_world):
    """What an ideal accelerometer reads: the true acceleration minus gravity, in the sensor's frame."""
    # TODO (1 line), worksheet 11.1: rotate (a_world - GRAVITY) into the sensor frame with R^T
    raise NotImplementedError


def analyze_drop(t, z, force, threshold=5.0, settle_mm=0.5):
    """When the force first passes `threshold` N (t_touch), how far the foot sinks below that height
    afterwards (max_sink_mm), and from when it stays within settle_mm of its final height (t_settled)."""
    # TODO (4 lines), worksheet 11.2:
    #   i_touch = the first index where force > threshold          (np.argmax on a bool array finds it)
    #   max_sink_mm = 1000 * (height at i_touch - the lowest height from i_touch on)
    #   i_settled = one past the LAST index where |z - z[-1]| >= settle_mm / 1000
    #   return t[i_touch], max_sink_mm, t[i_settled]
    raise NotImplementedError


def simulate(contact=GO1_FOOT, duration=0.5):
    """(Provided.) Drop the body and log time, height, vertical speed, contact force and the IMU.

    The true acceleration a_world comes from the change in velocity over each step. That is exact here
    because eulerdamp is off (DEBUG_LOG #5). mj_step computes the sensors for the state at the START of
    the step, then moves it, so a_world[k] = (v after step k - v before step k) / dt (DEBUG_LOG #1)."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(XML.replace("SOLIMP", contact))
    d = mujoco.MjData(m)
    foot, floor = m.geom("foot").id, m.geom("floor").id
    n = int(round(duration / m.opt.timestep))
    log = {k: np.zeros(n) for k in ("t", "z", "vz", "force")}
    log["accel"], log["a_world"], log["R"] = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3, 3))
    fc = np.zeros(6)
    for k in range(n):
        v_before = d.qvel[0]
        mujoco.mj_step(m, d)
        log["a_world"][k, 2] = (d.qvel[0] - v_before) / m.opt.timestep
        log["t"][k], log["z"][k], log["vz"][k] = d.time, d.qpos[0] + 0.05, d.qvel[0]
        for c in range(d.ncon):
            if {d.contact[c].geom1, d.contact[c].geom2} == {foot, floor}:
                mujoco.mj_contactForce(m, d, c, fc)
                log["force"][k] += fc[0]
        log["accel"][k] = d.sensor("accel").data
        log["R"][k] = d.site("imu").xmat.reshape(3, 3)
    return log


def demo():
    log = simulate()
    print("A) What the accelerometer measures (body z axis)")
    print(f"   falling, t = 0.02 s:  reads {log['accel'][19, 2]:+.3f} m/s^2   (free fall feels like nothing)")
    print(f"   resting, t = 0.50 s:  reads {log['accel'][-1, 2]:+.3f} m/s^2   (the floor pushing up: +g)")
    mine = np.array([specific_force(R, a) for R, a in zip(log["R"], log["a_world"])])
    print(f"   your specific_force vs MuJoCo's accelerometer, worst difference over the drop: "
          f"{np.abs(mine - log['accel']).max():.1e} m/s^2")
    print("\nB) When is the foot planted? (a quarter of the Go1's weight on one foot sphere)")
    print("   contact                   force > 5 N at   foot centre then   sinks further   settled (0.5 mm) at")
    for name, contact in (("Go1 foot (soft, solimp)", GO1_FOOT), ("MuJoCo default (stiff)", DEFAULT)):
        lg = simulate(contact)
        t_touch, sink, t_settled = analyze_drop(lg["t"], lg["z"], lg["force"])
        z_touch = 1000 * lg["z"][int(round(t_touch / 0.001)) - 1]
        print(f"   {name:24s} {1000 * t_touch:9.0f} ms {z_touch:14.1f} mm {sink:12.1f} mm {1000 * t_settled:15.0f} ms")
    print("   Your project's Go1 foot crossed 5 N at 21.9 mm too, then kept going (DEBUG_LOG #3).")
    print("   'In contact' by force is not 'stopped'. The filter assumes a planted foot doesn't move, which is")
    print("   why the real project waits after touchdown and ends contact early (Module 7, DEBUG_LOG #3-#4).")


if __name__ == "__main__":
    demo()
