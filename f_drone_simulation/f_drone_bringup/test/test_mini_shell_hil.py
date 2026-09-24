"""Exercise the real Gazebo sensors and DroneSerial using a virtual Teensy."""

import math
import os
import pty
import select
import signal
import struct
import subprocess
import time

import pytest
from gz.msgs10.actuators_pb2 import Actuators
from gz.transport13 import Node


def test_sensor_serial_motor_loop(tmp_path, monkeypatch):
    monkeypatch.setenv('GZ_PARTITION', f'mini_shell_hil_{os.getpid()}')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    master, slave = pty.openpty()
    node = Node()
    commands = []
    node.subscribe(Actuators, '/mini_shell/gazebo/command/motor_speed',
                   lambda msg: commands.append(list(msg.velocity)))
    log = (tmp_path / 'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'f_drone_bringup', 'mini_shell_hil.launch.py',
        'gazebo_gui:=false', f'serial_port:={os.ttyname(slave)}',
    ], stdout=log, stderr=subprocess.STDOUT, cwd=tmp_path, start_new_session=True)
    buffer = bytearray()
    readings = []
    # Firmware's seven float telemetry fields followed by four OneShot125 values.
    telemetry = struct.pack('<7f4H', 1, 0, 0, 0, 0, 0, 0, 120, 125, 130, 135)
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if select.select([master], [], [], 0.05)[0]:
                buffer.extend(os.read(master, 4096))
            while len(buffer) >= 31:
                if buffer[:2] != b'\xaa\x55' or buffer[30] != 0xfe:
                    del buffer[0]
                    continue
                readings.append(struct.unpack('<7f', buffer[2:30]))
                del buffer[:31]
                os.write(master, telemetry)
            if (any(all(math.isfinite(v) for v in row)
                    and 9 < row[2] < 10.5 and 0.01 < row[6] < 0.1
                    for row in readings) and commands):
                break
        assert readings, f'No serial sensor frames; see {log.name}'
        assert any(9 < row[2] < 10.5 and 0.01 < row[6] < 0.1 for row in readings), (
            f'Expected gravity and downward ground range; last readings: {readings[-3:]}; {log.name}')
        assert commands, f'No mini_shell actuator messages; see {log.name}'
        assert commands[-1] == pytest.approx([0, 0, 230.4, 460.8], abs=0.01)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        os.close(master)
        os.close(slave)
