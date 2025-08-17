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
    foxglove_bridge = get_package_share_directory("foxglove_bridge")
    pkg_share_dir = get_package_share_directory('f_drone_simulation')
    f_drone_sim_worlds = os.path.join("worlds")

    # The gz_sim package which will launch gazebo for us, into our world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PathJoinSubstitution([
            "worlds", "drone_world.sdf"
        ])}.items(),
    )

    # Convert our drone URDF to SDF then read into variable
    # ExecuteProcess(cmd=[['gz sdf -p model.urdf > ../Hardware/model.sdf']])
    drone_sdf = os.path.join("models", "f_drone", "model.urdf")
    with open(drone_sdf, 'r') as infp:
        drone_desc = infp.read()

    # Spawn our Entity(drone) into the world - at the origin
    gz_entity_import = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_spawn_model.launch.py")),
        launch_arguments={
            "world": PathJoinSubstitution(["empty"]),
            "file": PathJoinSubstitution(["models", "f_drone", "model.sdf"]),
            "entity_name": "drone",
            "x": '0.0',
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
            os.path.join(foxglove_bridge, "launch", "foxglove_bridge_launch.xml"))
    )

    # Create easier translation from ROS to gazebo
    gz_drone_topic = '/drone'

    # Need to remap all of our TF topics for Gazebo
    gz_odom_topic = gz_drone_topic + '/odom'
    gz_joint_state_topic = '/world' + gz_drone_topic + '/joint_state'
    gz_link_pose_topic = gz_drone_topic + '/pose'

    # ROS2 -> Gazebo bridge to allow constant communication - converts from
    # ROS msg types to Gazebo msg types
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # All the Tf topic translated using ([) which is a ROS bridge
            gz_joint_state_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            gz_link_pose_topic + '@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            gz_link_pose_topic +
                '_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',

            # Odometry with (@) is a bidirectional bridge
            gz_odom_topic + '@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # Get access to the speed of the motors and send cmds too
            gz_drone_topic + '/motor_cmd_vel' +
                '@actuator_msgs/msg/Actuators@gz.msgs.Actuators'
        ],
        name="gazebo_bridge",
        remappings=[
            (gz_joint_state_topic, 'joint_states'),
            (gz_link_pose_topic, '/tf'),
            (gz_link_pose_topic + '_static', '/tf_static'),
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    # Get access to our visualization Node as our "Ground Control"
    # *Note* a parameter to define the name of drone for the TF broadcaster
    ground_control = Node(
        package='f_drone_ground_control',
        executable='visualization_process',
        name='ground_control',
        output='both',
        parameters=[{
            # Name our drone so we can connect to a certain one
            'drone_name': 'drone'
        }]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(f_drone_sim_worlds, 'rviz', 'f_drone.rviz')],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Set gazebo environment variables
    models_path = pkg_share_dir + "/gazebo/models"
    worlds_path = pkg_share_dir + "/gazebo/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    os.environ['BIG_PATH'] = pkg_share_dir

    # Add in all our separate commands into one general launch command
    return LaunchDescription([
        # ExecuteProcess(cmd=[['foxglove-studio']]),
        # foxglove,
        gz_sim,
        gz_entity_import,
        controller,
        ros_gz_bridge,
        ground_control,
        rviz
    ])
