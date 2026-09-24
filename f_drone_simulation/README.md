# f_drone_simulation
This folder is based on the [ros_gz_template_project](https://github.com/gazebosim/ros_gz_project_template) which is the correct way to define ROS 2 / Gazebo projects. 
While we are technically ROS agnostic, we should still follow Gazebo guidelines as they apply well.
In accordance to the project [guidelines](https://gazebosim.org/docs/latest/ros_gz_project_template_guide/) this project pairs ROS and Gazebo variants, so for `jazzy` we're using `harmonic`.

## Getting Started
This part of the project will expand in the future. The easiest way to run project is to start by launching the various examples
Enter the project, source the workspace, build, and launch as such:

```shell
export ROS_VERSION=jazzy
export LAUNCH_NAME=whatever_launch

source /opt/ros/${ROS_VERSION}/setup.bash
colcon build --base-paths f_drone_simulation && source install/setup.bash
ros2 launch f_drone_bringup ${LAUNCH_NAME}.launch.py
```

You can replace `whatever_launch` with whatever name of the launch file you want to run from the following.

| LAUNCH_NAME | Description | launch arguments |
|-------------|-------------|------------------|
| [crazy_launch](f_drone_bringup/launch/crazy_launch.launch.py) | This is meant to be a dry-test run with a standard Crazyflie drone which has been proven to work with the teleop GUI | None |
| [mini_shell_launch](f_drone_bringup/launch/mini_shell_launch.launch.py) | OnShape-derived drone with Gazebo teleop, live URDF/TF, and Foxglove | `gazebo_gui`, `foxglove_gui`, `foxglove_port` |
| [mini_shell_hil](f_drone_bringup/launch/mini_shell_hil.launch.py) | Mini-shell IMU/range sensors and motor models connected to the Teensy through DroneSerial | `serial_port`, `gazebo_gui` |
| [simple_drone](f_drone_bringup/launch/simple_drone.launch.py) | Newer drone export with live URDF, fixed/moving TF, and Foxglove | `gazebo_gui`, `foxglove_gui`, `foxglove_port` |

Run these commands from the repository root; see the
[Gazebo and Foxglove example](docs/basic_gazebo_foxglove_example.md)
for the saved layout, first-time setup, and verification steps.
For the newer `models/drone` export, use the
[drone Foxglove guide](docs/drone_foxglove.md).
For Teensy integration, use the [mini-shell HIL guide](docs/mini_shell_hil.md),
including the firmware checks required before running powered actuators.
