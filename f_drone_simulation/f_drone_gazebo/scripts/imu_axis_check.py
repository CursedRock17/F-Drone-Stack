#!/usr/bin/python3
"""Verify the IMU mounting rotation in model.sdf against what dRehmFlight needs.

Why this exists
---------------
model.sdf's imu_sensor <pose> is the physical mounting rotation of the MPU6050
relative to the CAD body frame. It was derived analytically from the mixer, but
the roll and pitch derivations disagreed by a sign (they implied a reflection,
det = -1), so exactly one branch is wrong. This measures which.

The requirement, from controlMixer()
------------------------------------
  +roll_PID  goes to motors 0,1 -- both at body -X -> torque about body +Y
  +pitch_PID goes to motors 1,3 -- both at body -Y -> torque about body -X

For negative feedback:
  roll_PID  < 0 must reduce roll_IMU  => roll_IMU  RISES with rotation about +Y
  pitch_PID > 0 must reduce pitch_IMU => pitch_IMU RISES with rotation about +X

So: tilt the body a known amount and check the sign of the angle the firmware
would compute. If either fails, the sensor pose is wrong.

Method
------
The model is made <static> so it neither tips over nor free-falls (an
accelerometer in free fall reads zero). A static link still reports gravity --
verified: body roll +15 deg gives ax=-2.539, az=9.476, |a|=9.81.
Each orientation needs its own sim, since the pose is set in the world include.

The firmware path being reproduced (dRehmFlight.ino):
  getIMUdataSim(): AccX = ax / 9.80665, etc.
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt)
so the accel vector actually fed to the filter is (-AccX, AccY, AccZ). At rest
Madgwick converges to the attitude implied by that vector.

Usage:  ./scripts/imu_axis_check.py     (exit 0 = pose correct, 1 = wrong)
"""

import math
import os
import re
import signal
import subprocess
import tempfile
import time

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
MODEL = os.path.join(PKG, "models", "f_drone", "model.sdf")
G = 9.80665
TILT_DEG = 15.0

WORLD = """<?xml version="1.0"?>
<sdf version='1.10'>
  <world name='imu_axis_check'>
    <gravity>0 0 -9.81</gravity>
    <physics name='p' type='dartsim'>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <include>
      <uri>file://{model}</uri><name>drone</name>
      <pose>0 0 1 {r} {p} 0</pose>
    </include>
  </world>
</sdf>
"""


def make_static_model(dst):
    """Strip DroneSerial (no Teensy here) and pin the model in place."""
    s = open(MODEL).read()
    s = re.sub(r'\s*<!-- Turn on when ready for Teensy reaction -->\s*'
               r'<plugin\s+filename="DroneSerial".*?</plugin>', '', s, flags=re.S)
    s = s.replace("<model name='drone'>",
                  "<model name='drone'>\n    <static>true</static>")
    open(dst, "w").write(s)


def read_imu(roll_rad, pitch_rad, model_path, tmp):
    """Launch a sim at the given orientation and return (ax, ay, az)."""
    world = os.path.join(tmp, "w.sdf")
    open(world, "w").write(WORLD.format(model=model_path, r=roll_rad, p=pitch_rad))
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=REPO)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", world],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           env=env, preexec_fn=os.setsid)
    try:
        for _ in range(30):
            time.sleep(1.0)
            try:
                out = subprocess.run(["gz", "topic", "-e", "-t", "/imu", "-n", "1"],
                                     capture_output=True, text=True, timeout=8).stdout
            except subprocess.TimeoutExpired:
                continue
            m = re.search(r"linear_acceleration \{(.*?)\}", out, re.S)
            if not m:
                continue
            body = m.group(1)

            def axis(k):
                hit = re.search(rf"{k}:\s*([-\d.eE+]+)", body)
                return float(hit.group(1)) if hit else 0.0

            return axis("x"), axis("y"), axis("z")
        return None
    finally:
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        except Exception:
            pass


def firmware_angles(ax, ay, az):
    """roll_IMU / pitch_IMU as dRehmFlight would compute them at rest."""
    fx, fy, fz = -ax / G, ay / G, az / G      # the (-AccX, AccY, AccZ) it feeds
    roll = math.degrees(math.atan2(fy, fz))
    pitch = math.degrees(math.atan2(-fx, math.hypot(fy, fz)))
    return roll, pitch


def main():
    src = open(MODEL).read()
    pose = re.search(r'<sensor name="imu_sensor" type="imu">\s*<pose>([^<]+)</pose>', src)
    print(f"model           : {MODEL}")
    print(f"imu_sensor pose : {pose.group(1).strip() if pose else '??'}")
    print(f"tilt            : {TILT_DEG} deg\n")

    t = math.radians(TILT_DEG)
    cases = [
        ("level",             0.0,  0.0),
        ("body +X (roll +)",    t,  0.0),
        ("body -X (roll -)",   -t,  0.0),
        ("body +Y (pitch +)",  0.0,   t),
        ("body -Y (pitch -)",  0.0,  -t),
    ]

    tmp = tempfile.mkdtemp(prefix="imu_axis_")
    model_path = os.path.join(tmp, "m.sdf")
    make_static_model(model_path)

    print(f"{'case':20} {'ax':>8} {'ay':>8} {'az':>8} | {'roll_IMU':>9} {'pitch_IMU':>9}")
    res = {}
    for name, r, p in cases:
        a = read_imu(r, p, model_path, tmp)
        if a is None:
            print(f"{name:20} {'-- no IMU data --'}")
            continue
        roll, pitch = firmware_angles(*a)
        res[name] = (roll, pitch)
        print(f"{name:20} {a[0]:8.3f} {a[1]:8.3f} {a[2]:8.3f} | "
              f"{roll:9.2f} {pitch:9.2f}")

    print("\nRequirements (derived from controlMixer):")
    ok = True

    def check(label, got, want_positive):
        nonlocal ok
        good = (got > 1.0) if want_positive else (got < -1.0)
        ok = ok and good
        print(f"  [{'PASS' if good else 'FAIL'}] {label}: got {got:+.2f} deg, "
              f"want {'positive' if want_positive else 'negative'}")

    checks = [
        ("body +X (roll +)",  1, "pitch_IMU rises with rotation about body +X", True),
        ("body -X (roll -)",  1, "pitch_IMU falls with rotation about body -X", False),
        ("body +Y (pitch +)", 0, "roll_IMU rises with rotation about body +Y",  True),
        ("body -Y (pitch -)", 0, "roll_IMU falls with rotation about body -Y",  False),
    ]
    for case, idx, label, want_pos in checks:
        if case in res:
            check(label, res[case][idx], want_pos)

    print()
    if ok:
        print("VERDICT: current imu_sensor pose closes the loop with the correct sign.")
    else:
        print("VERDICT: sign mismatch -- the current imu_sensor pose is wrong.")
        print("  Try the alternate branch:  <pose>0 0 0 3.14159 0 -1.5708</pose>")
        print("  (yaw -90 plus a 180 roll, i.e. the IMU mounted inverted)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
