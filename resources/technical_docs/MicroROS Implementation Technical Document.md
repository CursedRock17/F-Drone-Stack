#### Overall Objective: Utilize MicroROS to allow our drone to communicate with external interfaces.

##### Communication Layer: 
MicroROS utilizes micro-XRCE-DDS as the middleware layer, which has a set memory profile. It starts with an XRCE Agent which acts as the server, while each connected device (the Teensy 4.0 in our case) acts as the client. That being said, it utilizes a publisher/subscription interface to communicate over WiFi (IoT) or Serial Port or even a customizable interface. It can handle messages up to 512 bytes in size.

##### External Communication: 
Our first form of communication should take place between 1 drone and 1 ground station, we can create regular `rclcpp` or `rclpy` nodes on that ground station and communicate via Serial port. We can ensure that the communication of messages (any type) flows between the serial port and the Teensy 4.0, via [MicroROS Agent](https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/) and Arduino support. We can then transition to WiFi based protocol in which we use an ESP32 Wifi chip or RPi Nano chip with builtin wifi and send that to the ground station. We could then possibly use MavLink to send message over radio, which allows multiple drones to communicate with each other.

##### Topics to Publish :
 - `/current_state` : Type (`nav_msgs/odometry)
	 - `geometry_msgs/Twist` : Twist: Current Linear & Angular Velocities
	 - `geometry_msgs/Pose` : Pose: Current Orientation & Position
##### Topics to Subscribe: 
- `/desired_state`  : Type (`trajectory_msgs/MultiDOFJointTrajectoryPoint`)
	- `geometry_msgs/Transform` :  Wanted Position 
		- `Vector3` : Translation: Desired Position
		- `Quaternion` Rotation: Desired Rotation
	- `geometry_msgs/Twist` : Velocities : Desired Linear & Angular Velocities
	- `geometry_msgs/Twist` : Accelerations : Desired Linear & Angular Accelerations
- `/desired_motor_cmds` : Type (`mav_msgs/Actuators`) 
	- `float64[]` Angular Velocities (in RPM)
	
[list2mermaid]

- dRehmFlight_Loop
 - Odometry_Node
	    - /currrent_state
- External_Device
 - Trajectory_Node
		  - /desired_state
 - Motors_Node
		  - /desired_motor_cmds

##### References:
https://micro.ros.org/docs/concepts/middleware/Micro_XRCE-DDS/
https://micro.ros.org/docs/concepts/middleware/memo_prof/
https://mavlink.io/en/about/overview.html