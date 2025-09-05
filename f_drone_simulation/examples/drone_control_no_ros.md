# Drone Control (No ROS) - Hardware in the Loop
-------------------------------------------------
This tutorial will explain the necessary steps of Hardware in the Loop (HIL). HIL is 
a method for higher fiedlity testing a robot, making the onboard computer think that
it's completing actions in the real world, when really we're running in simulation.
Ensure, you have access to a Teensy 4.0/4.1 connected to the ELRS receiver. *Note*,
once we're able to run without the receiver, we won't even need that. You don't need
any other components on the drone as we can find simulated sensor information.

### Running the World
1) Open an instance of the terminal (in the hardware directory).
2) Source your local Gazebo/ROS environment : `source /opt/ros/humble/setup.bash`.
3) Build the simulation package `colcon build && source install/setup.bash`.
4) Run the command `gz sim worlds/drone_world.sdf` to open our world.

### Controlling the Drone
1) With the sim up and running, open your Arduino IDE, connect a USB to the Teensy,
upload the code onto the Teensy, then optionally close the IDE.
2) Open another Gazebo-sourced terminal and run the command:
```
gz topic --echo --topic /drone/gazebo/command/motor_speed
```
We can now view the desired velocity (in RPM) of each of our 4 motors.
3) Connect a handset, then feed it stick information.
4) Click play in sim and our drone should be flying around.
