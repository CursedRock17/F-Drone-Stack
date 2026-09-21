import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


# Teleop-only bringup: a blank world holding a single Crazyflie 2.X.
def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    f_drone_gazebo = get_package_share_directory("f_drone_gazebo")
    pkg_share_dir = get_package_share_directory("f_drone_bringup")

    # Blank world (ground plane + sun) with the crazyflie already included.
    world = os.path.join(f_drone_gazebo, "worlds", "crazyflie.sdf")

    # Same GUI layout as teleop_gui.config, but its Teleop pad is pointed at
    # a staging topic that teleop_lateral rewrites before the controller sees
    # it: W/S forward, Q/E vertical, A/D left/right.
    gui_config = os.path.join(pkg_share_dir, "config", "crazy_gui.config")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": world + " -r --gui-config " + gui_config
        }.items(),
    )

    # The model's MulticopterVelocityControl listens under <robotNamespace>
    # crazyflie, regardless of the entity name given in the world file.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_share_dir, 'config', 'crazy_bridge.yaml'),
        }],
        output='screen'
    )

    # The Teleop pad has no lateral axis, so its yaw output is rewritten as
    # body-frame linear.y on the way to the controller.
    teleop_lateral = Node(
        package='f_drone_bringup',
        executable='teleop_lateral.py',
        name='teleop_lateral',
        output='screen'
    )

    # The velocity controller ignores twist commands until it is enabled, and
    # it only latches the flag once the world is up.
    enable_controller = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(cmd=[
            'gz', 'topic', '-t', '/crazyflie/enable',
            '-m', 'gz.msgs.Boolean', '-p', 'data: true'
        ], output='screen')]
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        teleop_lateral,
        enable_controller,
    ])
