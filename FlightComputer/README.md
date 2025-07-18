# Flight Computer

## Setting up the Software

### PreRequisites
  - ROS 2 Humble [Installed](https://docs.ros.org/en/humble/Installation.html)
  - Python's Pip has been [Installed](https://pypi.org/project/pip/)
  - The virtual environment Pipenv has been [Installed](https://pipenv.pypa.io/en/latest/)

### Installing micro-ROS
We Need to install micro-ROS to allow our Teensy to communicate with its
surroundings whether that be ground control or other Teensies

*Note* I'm doing this in Ubuntu 22.04, where ROS 2 presents us as a Tier 1 OS
if you choose another platform the process should remain the same, but you
may find it to be a bit more difficult.
```
source /opt/ros/humble/setup.bash

# Create A Working Directory from which we have a stable micro-ROS setup
mkdir microros_ws && cd microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Ensure Our dependencies are in line with the version
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Use the colcon buildchain tool to set up our resources
colcon build && source install/local_setup.bash
```
This install provides us with a setup agent for micro-ROS which will walk
us through some install steps
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
