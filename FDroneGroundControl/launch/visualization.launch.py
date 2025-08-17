from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f_drone_ground_control',
            executable='visualization_process',
            name='tf_broadcaster'
        ),
    ])
