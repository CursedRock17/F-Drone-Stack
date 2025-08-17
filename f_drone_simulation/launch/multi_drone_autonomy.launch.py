import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


# General Call to Create the Launch File
def generate_launch_description():
    # Add in all separate packages
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    foxglove_dir = get_package_share_directory("foxglove_bridge")

    # Need to grab the gz_sim package which will launch gazebo for us, into our world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PathJoinSubstitution([
            "worlds", "drone_world.sdf"
        ])}.items(),
    )

    # Convert our drone URDF to SDF then read into variable
    drone_sdf = os.path.join("models", "drone", "f_drone.sdf")
    with open(drone_sdf, 'r') as infp:
        drone_desc = infp.read()

    # Spawn both drones into the world - split evenly about the origin
    entity_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_spawn_model.launch.py")),
        launch_arguments={
            "world": PathJoinSubstitution(["empty"]),
            "file": PathJoinSubstitution(["models", "drone", "f_drone.sdf"]),
            "entity_name": "f_drone_one",
            "x": '-0.5',
            "y": '0.0',
            "z": '0.0',
        }.items()
    )

    entity_two = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_spawn_model.launch.py")),
        launch_arguments={
            "world": PathJoinSubstitution(["empty"]),
            "file": PathJoinSubstitution(["models", "drone", "f_drone.sdf"]),
            "entity_name": "f_drone_two",
            "x": '0.5',
            "y": '0.0',
            "z": '0.0',
        }.items()
    )

    # Drone Control Goes Here (Some Sort of Controller Node)
    controller = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': drone_desc},
        ]
    )

    # Visualize Data with Foxglove
    # Connect our Foxglove Bridge to hear all of our ROS 2 topics
    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(foxglove_dir, "launch", "foxglove_bridge_launch.xml"))
    )

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        ExecuteProcess(cmd=[['foxglove-studio']]),
        gz_sim,
        entity_one,
        entity_two,
        controller,
        foxglove,
    ])
