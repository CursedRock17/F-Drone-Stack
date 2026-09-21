This document discusses the plan for implementing hardware in the loop.

We need to run the Arduino code running on the Teensy, within Gazebo, best way to do this is via plugins, which can connect to the drone sdf model by interfacing data over Serial Port between Teensy and Gazebo. Refering back to the large overview of our system:
![[Sensor Components.svg | 700x800]]
We can replace the two blocks `IMU` and `Matek-3901` with two sensors in Gazebo with their corresponding plugins (respectively): 
```
<sensor type="imu"/>
<sensor type="gpu_lidar"/>
```
The intent is to have to mimic as little as possible, we should be able to run of the regular 
Teensy code, just inserting the values from the stream into their physical counterparts. 
All we have to do is let the Teensy know the values are simulated as to not prioritize the real sensors, 
the Teensy should be on during this process. In fact, nothing other than tuning parameters should be
changed on the side of the Teensy as to transition from simulation to real as smoothly as possible.

### Things to Address:
- How do we get the Teensy to run in Gazebo, while in real life? -> Use the Serial Port
- How can we replicate dRehmFlight code in Gazebo Plugin -> No need to do so, in fact we shouldn't, bad behavior.
- How can we replicate PWM to Motor Speeds? Utilize a simulated ESC profile which uses collected data
from the motor/prop configuration within some equations.
- How are we able to run a realistic simulation at the correct 2KHz Loop Rate that dRehmFlight requires?

### Required Plugins:
- IMU         (IMU sensor)
- Teensy 4.0  (SerialData, custom plugin)
- Rangefinder (GPULidar sensor)
- Motors      (MulticopterMotorModel)

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


### Correctly setting up the simulated motors 

#### Getting Real Results
There are many thing that need high fidelity as we look to transition from simulation to real life,
one of which is our motors. The Plugin that Gazebo natively offers, `MulticopterMotorModel` is great,
but there are some constants which need filling out. If you should find uncertainty w/provided
datasheets for your motor/prop configuration, you can test up a weighted test stand on top of a scale.
Attach one motor and one prop to the test stand in normal configuration, but you want the air to blow
up such that you get the accurate value of thrust in grams. 

You can use this Matlab script to help visualize your acquire data:
```
%% HGLRC 1202.5 11000 KV Motor + Gemfan 3018-2 2.5" props - 7.6v
% Variables Relating to Your Setup
max_voltage = 8.4;
voltage_used = 7.6;
given_kv = 11000;

% Acquired Data
pwm_percent = [10 20 30 40 50 60 70 80 90 100];
force_output = [3.6 15.8 22.0 29.2 33.1 46.8 48.5 51.5 53.3 55.2];

% Plot the Point, the connect with a line
figure(3);
plot(pwm_percent, force_output, '-og', 'MarkerSize', 15, 'LineJoin', 'round');
grid on;

xlabel('PWM Percentage');
ylabel('Thrust Force (g)');
xlim([0, 110]);
title('PWM Percentage versus Prop Thrust Generated');
```
