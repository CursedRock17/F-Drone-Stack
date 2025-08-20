#### Overall Objective: Attach an optical flow sensor to our drone and fuse the sensor data in dRehmFlight.

#### Hardware Connection: 
We need to mount the Matek 3901-L0X Optical Flow Sensor to the front of the drone, with not too many loose wires, connected to the Teensy 4.0.
	Need 2 M3 screw hole embedded in a printed mount, which has some overhang to clear the drone frame and a slot so that the 4 wire pads are accessible. 4 Wires will be soldered to each of the pads and put into a connector, the male connector will mate with a female connector, which is soldered directly onto the PCB. There are already 4 pads prepared for the addition of 1 UART device. Pinouts as followed:
		- Teensy Pin 16 (TX)
		- Teensy Pin 17 (RX)
		- GND
		- 5V
	We can then click in the 2 connectors and tie it to the frame with Nylon Wax string. We should prefer some sort of stabilized house/socket to the board.

#### Software Implementation:
Utilizes 1 UART port, leaves us with 3 remaining, at 115200 Baud Rate. Uses [UART MSP V2](https://github.com/iNavFlight/inav/wiki/MSP-V2) for the communication protocol. We can either write a small header file library for it, or use a stable implementation like [Reefwing MSP](https://github.com/Reefwing-Software/Reefwing-MSP) which has the logic prewritten. I will start with the premade Arduino library, then move to a header file library should we need a slight performance boost or more control. That being said we get access to 65536 bits of payload data which is plenty and there have been no complaints of lossy data. Data passed as Hexadecimal, we can use the Arducopter [reference](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_MSP/msp.cpp) to decipher the payload buffer and extract the data.

To utilize the data given from the sensor, we will be implementing sensor fusion with the IMU. It's important to provide an optional argument in actually using the optical flow sensor in case we would like to remove it in the future, or test with other sensors. We already have attitude estimation (Roll, Pitch, Yaw) from the Madgwick Filter, but we can get pose estimation (X, Y, Z) from the Optical Flow sensor package using an Extended Kalman Filter, using the IMU as a supplement.

[list2pumlWBS]

- EKF
	- PWM3901 OpFlow Sensor
	- ST VL53L0X Rangefinder
	- MPU6050 (IMU)
		- Accelerometer
		- < Gyroscope


#### Benefits
- Allows for position hold and height adjustment on non flat surface
- Provides Velocity Estimation, thus allowing for Trajectory based waypoints
- Limited to 40mA of current
- Allows us to work in GPS denied environments and works intraprocessly
#### Drawbacks
- Limited use to indoor environments, the Matek 3901-L0X has a range of 2cm -> 200cm
- Uses 5V to power the device, may need to do some flight testing with and without
- Cannot provide navigation ability *within itself*, Cannot see surroundings

##### Useful Resources:
https://www.mateksys.com/?portfolio=3901-l0x#tab-id-3
https://github.com/iNavFlight/inav/wiki/MSP-V2
https://forum.arduino.cc/t/problem-in-converting-the-mspv2-to-get-flow-x-flow-y-and-distance-from-module-optical-flow-lidar-sensor/1281208/12
https://github.com/jagennath-hari/Project-3-UKF/tree/7dc175fe0f9440d320c4b40e9311bcc8955398c4?tab=readme-ov-file
https://www.mdpi.com/1424-8220/24/7/2183#FD3-sensors-24-02183
file:///home/cursedrock17/Downloads/Resumo%20Alargado.pdf