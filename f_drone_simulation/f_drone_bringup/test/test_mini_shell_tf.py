"""Exercise the installed launch: missing state bridges must fail this test."""

import os
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


@pytest.mark.parametrize('launch_file, namespace, body_frame, check_climb', [
    ('mini_shell_launch.launch.py', 'mini_shell', 'base_link', True),
    ('simple_drone.launch.py', 'drone', 'root', False),
])
def test_model_and_tf_follow_simulation(tmp_path, monkeypatch, launch_file,
                                       namespace, body_frame, check_climb):
    # Isolate discovery and logs from an operator's running simulation.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    monkeypatch.setenv('GZ_PARTITION', f'mini_shell_tf_{os.getpid()}')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]

    log = (tmp_path / 'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'f_drone_bringup', launch_file,
        'gazebo_gui:=false', 'foxglove_gui:=false', f'foxglove_port:={port}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('mini_shell_tf_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received = {}

    # Keep subscriptions alive and retain the latched URDF for late viewers.
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(
        msg_type, topic, lambda msg, key=key: received.update({key: msg}), qos,
    ) for msg_type, topic, key, qos in [
        (String, '/robot_description', 'urdf', latched),
        (JointState, '/joint_states', 'joints', 10),
        (Clock, '/clock', 'clock', 10),
        (Odometry, f'/{namespace}/odometry', 'odom', 10),
    ]]
    publisher = node.create_publisher(Twist, f'/{namespace}/teleop/twist', 10)

    def wait_for(predicate, seconds=25):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        pytest.fail(f'Missing simulation state; received {sorted(received)}; log: {log.name}')

    try:
        wait_for(lambda: {'urdf', 'joints', 'clock', 'odom'} <= received.keys())
        root = ET.fromstring(received['urdf'].data)
        links = [link.get('name') for link in root.findall('link')]
        assert set(received['joints'].name) == {
            j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'
        }
        for mesh in root.findall('.//mesh'):
            package, relative = mesh.get('filename').removeprefix('package://').split('/', 1)
            assert (Path(get_package_share_directory(package)) / relative).is_file()

        wait_for(lambda: all(buffer.can_transform('world', link, rclpy.time.Time())
                            for link in links))
        start_z = buffer.lookup_transform('world', body_frame, rclpy.time.Time()).transform.translation.z
        if check_climb:
            command = Twist()
            command.linear.z = 0.3
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
            publisher.publish(Twist())
            wait_for(lambda: received['odom'].pose.pose.position.z > start_z + 0.3)

        # The body TF must agree with simulated odometry, including spawn yaw.
        odom = received['odom']
        stamp = rclpy.time.Time.from_msg(odom.header.stamp)
        wait_for(lambda: buffer.can_transform('world', body_frame, stamp))
        body = buffer.lookup_transform('world', body_frame, stamp).transform
        if check_climb:
            assert body.translation.z > start_z + 0.3
        for axis in ('x', 'y', 'z'):
            assert getattr(body.translation, axis) == pytest.approx(
                getattr(odom.pose.pose.position, axis), abs=1e-5)
        for axis in ('x', 'y', 'z', 'w'):
            assert getattr(body.rotation, axis) == pytest.approx(
                getattr(odom.pose.pose.orientation, axis), abs=1e-5)

        # All propellers stay attached at their CAD joint origins during flight.
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            parent = joint.find('parent').get('link')
            tf = buffer.lookup_transform(parent, child, rclpy.time.Time())
            expected = [float(v) for v in joint.find('origin').get('xyz').split()]
            actual = tf.transform.translation
            assert [actual.x, actual.y, actual.z] == pytest.approx(expected, abs=1e-6)
            assert buffer.can_transform('world', child, rclpy.time.Time())
        wait_for(lambda: received['clock'].clock.sec > 0)
        if check_climb:
            assert any(abs(v) > 1 for v in received['joints'].velocity)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
