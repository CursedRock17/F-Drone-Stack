# Flight Computer

## Setting up the Software

### PreRequisites
  - ROS 2 Humble [Installed](https://docs.ros.org/en/humble/Installation.html)
  - Python's Pip has been [Installed](https://pypi.org/project/pip/)
  - The virtual environment Pipenv has been [Installed](https://pipenv.pypa.io/en/latest/)
  - MAVROS [Installed](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md#installation)

### Installing micro-ROS
We Need to install micro-ROS to allow our Teensy to communicate with its
surroundings whether that be ground control or other Teensies

*Note* I'm doing this in Ubuntu 22.04, where ROS 2 presents us as a Tier 1 OS
if you choose another platform the process should remain the same, but you
may find it to be a bit more difficult.
```
source /opt/ros/humble/setup.bash

# Create A Working Directory from which we have a stable micro-ROS setup
mkdir -p microros_ws/src && cd microros_ws/src
git clone https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup -b humble

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
# Then
source install/local_setup.bash
# Then
ros2 run micro_ros_setup build_agent.sh
# Then
source install/local_setup.bash
```
At this point in time plug in your choice of Teensy into the board and ensure
it has the ability to flash, *Note* ensure the USB Micro-B Cable can do data transfer
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Issues/Resolutions

<details>
<summary> Teensy Hardware Issues </summary>

You may stumble into ROS definitions not being recognized under the guise of 
"no precompiled library support". First make sure you're using Teensy 1.57.3, which
you can alter in the board manager by searching "teensy". We then need to upload 
a new hardware patch, [found in](https://github.com/micro-ROS/micro_ros_arduino/blob/kilted/extras/patching_boards/platform_teensy.txt) the microROS github.

Navigate to the original install location of the Teensy firmware :
`cd ~/.arduino15/packages/teensy/hardware/avr/1.57.3`, then
replace the `platform.txt` file with the contents of the micro_ros patch
`curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/refs/heads/kilted/extras/patching_boards/platform_teensy.txt > platform.txt` should do the trick.


</details>
