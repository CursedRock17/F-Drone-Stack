
Sensors required to get Pose & Twist Data.

[list2pumlWBS]

- Position
	- PWM3901 OpFlow Sensor
	- ST VL53L0X Rangefinder
	- MPU6050 (IMU)
![[Sensor Components.svg | 700x800]]
## Altitude Hold
---------------------------------------------------------------------
### Closed Loop Control

[list2mermaid]

- Handset
	- Target_Altitude_Increase
		- Target_Altitude
			- PID
				- Throttle_Change
					- Target_Altitude_Increase
- Rangefinder
	- Current_Altitude
		- PID
	- Range_Quality
		- PID



The handset gimbal should correlate with movement, i.e. if we move up on the stick the drone should move upwards. *Think of it like* RPY control, the stick is in the middle (0 roll), when we wish to move to the right, we drag the stick to the right, so the roll increases and the drone is angled. When we reach the target location, we let go of the stick, it naturally moves back to the left (to the zero points), so it rolls to the left, just enough to where it levels out, based off of zero desired input, and an unlevel plane.

We can say that midpoint gimbal position = 0 movement, where our throttle = hover throttle, so we need to find out hover throttle first (4 * Tp = mg).

This whole process should act as a PID loop that's cyclical in nature, at each loop iteration. We should get the desired altitude increase which consists of the wanted increase from the handset (i.e 25% up the stick is 2.5mm increase) + the recognized current altitude. This is then compared to the current altitude from the rangefinder. So DV - PV = some error, we can then increase the desired throttle based on this change, as a percentage, so maybe 50% + hover throttle. We should be able to taper off these values, so the closer we are to the target altitude, the less throttle being supplied. *Ensure* there's some form of scaliing

**Empirical** vs **Approximate**? Is it just worth say it's good enough for this case.

#### Caveats:
  - What happens when we leave max/min range?
  - What things affect the motor PWM
  - What do we do when we run into sharp obstacles
  - How do we track hover throttle
  - Create a full blown diagram with the handset, all of the sensors which feeds into a Kalman filter
  - Showcase variation in motor speed with kV
