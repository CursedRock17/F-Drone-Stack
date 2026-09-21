# Drone Control
-------------------------

### Running the World
1) Open an instance of the terminal (in the hardware directory)
2) Source your local ROS environment : `source /opt/ros/humble/setup.bash`
3) Need to also build and source our local packages:
    1) `cd ../FDroneGroundControl`, now build the package: `colcon build`
    2) Navigate back to our local directory `cd ../Simulation` and source that
    previous install `source ../FDroneGroundControl/install/setup.bash`

4) Build the simulation package `colcon build && source install/setup.bash`
5) Run the launch file `ros2 launch f_drone_simulation drone_control.launch.py`
6) Enjoy!

Showing the change of the drone (base_link) to the world frame: 
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id base_link --x 1.0 --y 1.0 --z 1.0

That's like if it were flying, call `ros2 run tf2_tools view_frames` to 
see all active frames in comparison to on another, the only joints are the motors
which we really don't care about right now.
