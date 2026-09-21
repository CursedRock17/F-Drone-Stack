#!/usr/bin/python3
"""Open-loop rotor thrust sweep for the f_drone Gazebo model.

Measures the real thrust curve of gz-sim's MulticopterMotorModel as configured
in model.sdf, so `motorConstant` can be calibrated against a known hover point
instead of guessed.

Method
------
The world runs at ZERO GRAVITY. An accelerometer measures specific force, so
with gravity off the IMU's linear_acceleration.z is exactly T_total / m --
a direct thrust reading needing no differentiation of position (the odometry
topic is unreliable) and no drift correction. Vertical drift does not corrupt
the reading because MulticopterMotorModel's rotor drag acts perpendicular to
the rotor axis.

For each commanded rotor velocity w we record a_z, then:
    T_total = m * a_z          k = T_total / (4 * w^2)

A correct model gives a k that is constant across the sweep. Curvature in k
means thrust is not following k*w^2 -- e.g. a joint velocity limit clamping
the rotor, or rotorVelocitySlowdownSim being applied where it should not be.

Usage
-----
    ./scripts/thrust_sweep.py                     # default sweep
    ./scripts/thrust_sweep.py --omega 1000 2000   # explicit points
    ./scripts/thrust_sweep.py --keep-world        # leave generated files
"""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                       # f_drone_simulation
REPO = os.path.dirname(PKG)                       # F-Drone-Stack
MODEL = os.path.join(PKG, "models", "f_drone", "model.sdf")
WORLD_NAME = "thrust_sweep"
CMD_TOPIC = "/drone/gazebo/command/motor_speed"

WORLD_TEMPLATE = """<?xml version="1.0"?>
<sdf version='1.10'>
  <world name='{world}'>
    <!-- Zero gravity: the IMU then reads specific force == thrust/mass. -->
    <gravity>0 0 0</gravity>
    <physics name='2000hz' type='dartsim'>
      <max_step_size>0.0005</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <!-- Imu is a WORLD-level system in gz-sim; declaring it only on the model
         (as model.sdf does) is not enough to make /imu publish. -->
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <include>
      <uri>file://{model}</uri>
      <name>drone</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def strip_serial(sdf_text):
    """Remove the DroneSerial plugin: it opens /dev/ttyACM0 and, with no Teensy
    attached, logs on every physics step."""
    out = re.sub(
        r'\s*<!-- Turn on when ready for Teensy reaction -->\s*'
        r'<plugin\s+filename="DroneSerial".*?</plugin>',
        '\n    <!-- DroneSerial stripped by thrust_sweep.py -->',
        sdf_text, flags=re.S)
    return out


def model_mass(sdf_text):
    """Total model mass = base_link + the four rotor links."""
    masses = [float(m) for m in re.findall(r"<mass>\s*([\d.eE+-]+)\s*</mass>", sdf_text)]
    if not masses:
        raise RuntimeError("no <mass> found in model.sdf")
    return sum(masses), masses


def sdf_param(sdf_text, tag):
    vals = set(re.findall(rf"<{tag}>\s*([\d.eE+-]+)\s*</{tag}>", sdf_text))
    return vals


def wait_for_topic(topic, timeout=40):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                 text=True, timeout=8).stdout
            if topic in out.split():
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


def publish(omega):
    subprocess.run(
        ["gz", "topic", "-t", CMD_TOPIC, "-m", "gz.msgs.Actuators",
         "-p", "velocity:[{0},{0},{0},{0}]".format(omega)],
        capture_output=True, text=True, timeout=15)


class ImuReader:
    """One long-lived `gz topic -e` process, read incrementally.

    Repeated one-shot `-n 1` calls proved unreliable; a single streaming
    subscriber avoids the per-call connection churn.
    """

    def __init__(self, topic="/imu"):
        self.proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1, preexec_fn=os.setsid)

    def drain_az(self, duration):
        """Collect linear_acceleration.z samples for `duration` seconds."""
        import select
        samples, buf, in_la = [], "", False
        deadline = time.time() + duration
        while time.time() < deadline:
            r, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if not r:
                continue
            line = self.proc.stdout.readline()
            if not line:
                break
            if "linear_acceleration {" in line:
                in_la, buf = True, ""
                continue
            if in_la:
                if "}" in line:
                    m = re.search(r"z:\s*([-\d.eE+]+)", buf)
                    if m:
                        samples.append(float(m.group(1)))
                    in_la = False
                else:
                    buf += line
        return samples

    def close(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--omega", type=float, nargs="+",
                    default=[500, 1000, 1500, 2000, 2880, 4000, 5760])
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to let the rotor reach commanded speed")
    ap.add_argument("--window", type=float, default=1.5,
                    help="seconds of IMU samples to average")
    ap.add_argument("--keep-world", action="store_true")
    args = ap.parse_args()

    raw = open(MODEL).read()
    total_mass, masses = model_mass(raw)
    print(f"model.sdf  : {MODEL}")
    print(f"total mass : {total_mass:.6f} kg  (base {masses[0]:.6f} + "
          f"{len(masses)-1} rotors)")
    for tag in ("motorConstant", "maxRotVelocity", "rotorVelocitySlowdownSim"):
        print(f"{tag:26s}: {sorted(sdf_param(raw, tag))}")
    print()

    tmp = tempfile.mkdtemp(prefix="thrust_sweep_")
    bench_model = os.path.join(tmp, "model_bench.sdf")
    open(bench_model, "w").write(strip_serial(raw))
    world = os.path.join(tmp, f"{WORLD_NAME}.sdf")
    open(world, "w").write(WORLD_TEMPLATE.format(world=WORLD_NAME, model=bench_model))

    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=REPO)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", world],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           env=env, preexec_fn=os.setsid)
    reader = None
    try:
        if not wait_for_topic("/imu"):
            print("ERROR: /imu never appeared -- sim failed to start.", file=sys.stderr)
            return 1
        reader = ImuReader()
        time.sleep(1.0)

        base = reader.drain_az(1.0)
        if base:
            print(f"zero-command a_z = {sum(base)/len(base):+.4f} m/s^2 "
                  f"(should be ~0 in zero-g)\n")

        print(f"{'omega':>8} {'a_z':>10} {'thrust':>10} {'k':>12} {'N':>5}")
        print(f"{'rad/s':>8} {'m/s^2':>10} {'N tot':>10} {'N.s^2/rad^2':>12} {'':>5}")
        rows = []
        for w in args.omega:
            publish(w)
            time.sleep(args.settle)
            s = reader.drain_az(args.window)
            if not s:
                print(f"{w:8.0f} {'no samples':>10}")
                continue
            az = sum(s) / len(s)
            thrust = total_mass * az
            k = thrust / (4 * w * w) if w else float("nan")
            rows.append((w, az, thrust, k))
            print(f"{w:8.0f} {az:10.4f} {thrust:10.4f} {k:12.4e} {len(s):5d}")
        publish(0)

        if rows:
            print()
            ks = [r[3] for r in rows if r[0] > 0 and r[3] == r[3]]
            if ks:
                print(f"k: min {min(ks):.4e}  max {max(ks):.4e}  "
                      f"spread {100*(max(ks)-min(ks))/max(ks):.1f}%")
                print("  (a constant k means thrust follows k*w^2 as intended;"
                      " large spread means something is clamping the rotor)")
            weight = total_mass * 9.81
            best = ks[-1] if ks else None
            if best:
                import math
                print(f"\nweight = {weight:.4f} N  ->  hover omega = "
                      f"{math.sqrt(weight/(4*best)):.1f} rad/s at k={best:.4e}")
        return 0
    finally:
        if reader:
            reader.close()
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        except Exception:
            pass
        if args.keep_world:
            print(f"\nkept: {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
