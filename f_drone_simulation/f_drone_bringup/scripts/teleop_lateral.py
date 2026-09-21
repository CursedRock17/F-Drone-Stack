#!/usr/bin/python3
# Pinned to the system interpreter on purpose: /usr/bin/env python3 resolves
# to miniforge's 3.13, which cannot load Jazzy's 3.12 _rclpy_pybind11.
"""Remap the Gazebo Teleop pad's yaw axis onto lateral velocity.

The gz-gui Teleop plugin only ever publishes linear.x, linear.z and angular.z,
so A/D arrive as a yaw rate. MulticopterVelocityControl takes its linear
command in the body frame, so writing that value to linear.y makes the drone
bank and translate sideways instead of spinning in place.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopLateral(Node):
    def __init__(self):
        super().__init__('teleop_lateral')
        # Namespaced so one node type serves any drone: crazyflie, mini_shell.
        self.declare_parameter('input_topic', '/crazyflie/teleop/twist')
        self.declare_parameter('output_topic', '/crazyflie/gazebo/command/twist')
        # Rotation from the drone's visual forward to base_link's +x, in
        # radians. The controller commands in the body frame, so a model whose
        # CAD frame is not aligned with its nose needs that offset applied
        # here. Crazyflie is aligned, hence the 0.0 default.
        self.declare_parameter('command_yaw_offset', 0.0)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        offset = self.get_parameter('command_yaw_offset').value
        self.cos_off = math.cos(offset)
        self.sin_off = math.sin(offset)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self.on_twist, 10)
        self.get_logger().info(
            f'{in_topic} -> {out_topic} (yaw offset {offset} rad)')

    def on_twist(self, msg):
        # W/S arrive as linear.x, A/D as angular.z (+1 = left).
        forward = msg.linear.x
        left = msg.angular.z
        out = Twist()
        out.linear.x = forward * self.cos_off - left * self.sin_off
        out.linear.y = forward * self.sin_off + left * self.cos_off
        out.linear.z = msg.linear.z
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TeleopLateral()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
