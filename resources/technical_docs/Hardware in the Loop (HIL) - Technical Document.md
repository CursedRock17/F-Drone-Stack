This document discusses the plan for implementing hardware in the loop.

We need to run the Arduino code running on the Teensy, within Gazebo, best way to do this is via plugins, which can connect to the drone sdf model by interfacing data over Serial Port between Teensy and Gazebo. Refering back to the large overview of our system:
![[Sensor Components.svg | 700x800]]
We can replace the two blocks `IMU` and `Matek-3901` with two sensors in Gazebo with their corresponding plugins (respectively): 
```
<sensor type="imu"/>
<sensor type="gpu_lidar"/>
```
The intent is to have to mimic as little as possible, we should be able to run of the regular Teensy code, just inserting the values from the stream into their physical counterparts. All we have to do is let the Teensy know the values are simulated as to not prioritize the real sensors, the Teensy should be on during this process.

### Things to Address:
- How do we get the Teensy to run in Gazebo, while in real life? -> Use the Serial Port
- How can we replicate dRehmFlight code in Gazebo Plugin -> No need to do so, in fact we shouldn't, bad behavior.
- How can we replicate PWM -> Motor Speeds?

### Required Plugins:
- IMU
- Teensy 4.0 
- Rangefinder
- Motors

### Components and Loop
- #### Gazebo Ignition (physics & sensors)
    - Simulates drone dynamics.
    - Provides simulated **IMU** and **rangefinder** readings.
    - A **custom plugin** collects these simulated values and writes them over the serial port (e.g. `/dev/ttyACM0`).    
- #### Teensy 4.1 (real hardware, Arduino sketch)
    - Reads the sensor packets from serial instead of physical IMU/rangefinder.
    - Runs your PID / altitude controller logic.
    - Produces PWM outputs for motors (normally to ESCs).
    - Instead of toggling pins, it serializes those PWM values and sends them back over serial.
- #### Gazebo plugin (actuator side)
    - Reads motor PWM values from the Teensy.
    - Converts them to thrust & torque.
    - Applies those forces to the rotor joints in the physics engine.


[list2mermaid]

- Gazebo_Sensors
	- Teensy
		- Gazebo_Sim
			- Gazebo_Sensors
