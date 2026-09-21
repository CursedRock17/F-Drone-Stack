import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Mini-shell teleop with live URDF and TF visualization through Foxglove.
def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    f_drone_gazebo = get_package_share_directory("f_drone_gazebo")
    pkg_share_dir = get_package_share_directory("f_drone_bringup")
    description_dir = get_package_share_directory("f_drone_description")
    with open(os.path.join(description_dir, "models", "mini_shell", "model.urdf")) as source:
        robot_description = source.read()

    # GUI switches also allow the same launch to run in integration tests.
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    foxglove_gui = LaunchConfiguration('foxglove_gui')
    foxglove_port = LaunchConfiguration('foxglove_port')

    # Blank world (ground plane + sun) with the quadrotor already included.
    world = os.path.join(f_drone_gazebo, "worlds", "mini_shell.sdf")

    # Same GUI layout as teleop_gui.config, but its Teleop pad is pointed at
    # a staging topic that teleop_lateral rewrites before the controller sees
    # it: W/S forward, Q/E vertical, A/D left/right.
    gui_config = os.path.join(pkg_share_dir, "config", "mini_shell_gui.config")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [world, " -r --gui-config ", gui_config,
                        PythonExpression(["'' if '", gazebo_gui, "' == 'true' else ' -s'"])]
        }.items(),
    )

    # The model's MulticopterVelocityControl listens under <robotNamespace>
    # mini_shell, regardless of the entity name given in the world file.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_share_dir, 'config', 'mini_shell_bridge.yaml'),
        }],
        output='screen'
    )

    # The URDF describes geometry; measured joint states place its rotor links.
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True, 'publish_frequency': 50.0}],
        output='screen',
    )
    foxglove_bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        parameters=[{'port': ParameterValue(foxglove_port, value_type=int),
                     'address': '127.0.0.1', 'use_sim_time': True}],
        output='screen',
    )

    # Open the desktop client on the live bridge; import the saved layout once.
    foxglove = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['foxglove-studio', ['foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:',
                                 foxglove_port, '/']],
        condition=IfCondition(foxglove_gui), output='screen',
    )])

    # The Teleop pad has no lateral axis, so its yaw output is rewritten as
    # body-frame linear.y on the way to the controller.
    teleop_lateral = Node(
        package='f_drone_bringup',
        executable='teleop_lateral.py',
        name='teleop_lateral',
        parameters=[{
            'input_topic': '/mini_shell/teleop/twist',
            'output_topic': '/mini_shell/gazebo/command/twist',
            # base_link's +x points out the drone's right (MCU end is the
            # nose), so the pad's axes are rotated +90 deg on the way to the
            # controller. Keep this in step with the world's spawn yaw: the
            # two must sum to zero for W to drive the nose forward.
            'command_yaw_offset': 1.5708,
        }],
        output='screen'
    )

    # The velocity controller ignores twist commands until it is enabled, and
    # it only latches the flag once the world is up.
    enable_controller = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(cmd=[
            'gz', 'topic', '-t', '/mini_shell/enable',
            '-m', 'gz.msgs.Boolean', '-p', 'data: true'
        ], output='screen')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('foxglove_gui', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        gz_sim,
        ros_gz_bridge,
        state_publisher,
        foxglove_bridge,
        foxglove,
        teleop_lateral,
        enable_controller,
    ])
