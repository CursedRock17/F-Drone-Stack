# Mini-shell hardware in the loop

This variant uses the existing `DroneSerial` protocol to send Gazebo IMU and
range measurements to the Teensy and apply its four motor outputs in Gazebo.
The Teensy runs the estimator and controller; the Gazebo teleop velocity
controller is absent from this model.

## Run

Build and source from the repository root, then launch with the Teensy's port:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths f_drone_simulation --executor sequential
source install/setup.bash
ros2 launch f_drone_bringup mini_shell_hil.launch.py serial_port:=/dev/ttyACM0
```

Use `gazebo_gui:=false` for the server with headless sensor rendering.
The launch creates a temporary world with the selected serial port and removes
it on shutdown; it does not edit the installed model.
No ROS or Foxglove bridge is needed for this Gazebo Transport ↔ serial loop.
Radio commands and arming remain firmware responsibilities; the Gazebo keyboard
teleop controller does not control this variant.

**Before connecting powered actuators:** the checked-in firmware's
`simulatedEnvironment = true` selects simulated inputs, but `loop()` still calls
`commandMotors()` and `setup()` calls `armMotors()`; keep the ESCs disconnected
for this bench session until SIM mode explicitly isolates physical motor outputs.
This model change does not modify or flash firmware.

## Model and sensors

[`model_hil.sdf`](../f_drone_description/models/mini_shell/model_hil.sdf)
retains the geometry, collisions, mass, inertia and center of mass from the CAD
`model.sdf`, without the artificial center-of-mass correction used by
`model_teleop.sdf` for Gazebo's velocity controller.
The four motor models retain the teleop variant's parameters: 5760 rad/s maximum,
`motorConstant = 6.0e-08`, and visual rotor slowdown of 20.
These carried-over parameters are a starting point, not new hardware calibration.

| Component | Configuration |
| --- | --- |
| IMU | `base_link`, CAD IMU position, 1000 Hz, `/mini_shell/imu` |
| IMU orientation | Yaw −π/2, inherited from the old HIL model |
| Rangefinder | `base_link`, `(0, 0.055, -0.0045)` m, downward single ray, 100 Hz, 0.01–2 m, `/mini_shell/range` |
| Sensor systems | World-level `Imu` and `Sensors` with Ogre2 |
| Serial plugin | `DroneSerial`, both link names `base_link` |
| Motor commands | `/mini_shell/gazebo/command/motor_speed` |
| State outputs | `/mini_shell/odometry`, `/mini_shell/tf`, `/mini_shell/joint_states` |

The body nose points along CAD +Y; rotor indices match the firmware mixer:

| Telemetry index | Joint | Position | Rotation |
| --- | --- | --- | --- |
| 0 | `motor_prop_1` | Front left | CCW |
| 1 | `motor_prop_2` | Rear left | CW |
| 2 | `motor_prop_3` | Front right | CW |
| 3 | `motor_prop_4` | Rear right | CCW |

The rangefinder mounting position is inherited from the old simulation and needs
confirmation against the actual sensor mount; it measures distance from the
sensor, not world altitude of `base_link`.
The IMU mounting convention also needs a controlled roll/pitch/yaw sign check
against the Teensy estimator and mixer before attempting closed-loop flight.
Sensors currently have no added noise or bias, and no optical-flow source is
implemented.

## Existing wire protocol

The PDF's CRC/sequence/timestamp protocol is a proposal; this integration retains
the protocol implemented by `dRehmFlight.ino` and `DroneSerial`:

- PC → Teensy: `AA 55`, seven float32 values `(ax, ay, az, gx, gy, gz, range)`, `FE`: 31 bytes.
- Teensy → PC: seven float32 values `(timestamp, roll_PID, pitch_PID, yaw_PID, posZ, desZ, z_PID)` followed by four uint16 motor values: 36 bytes, without framing.
- Acceleration is in m/s², angular velocity in rad/s, and range in meters; firmware converts the IMU values for its estimator.
- Motor values use OneShot125: 125–250 maps linearly to 0–5760 rad/s, with clamping; disarmed 120 maps to zero.
- Host and Teensy use little-endian binary layouts; there is no CRC or sequence number.

`DroneSerial` now accepts `imu_topic`, `rangefinder_topic`, and `actuator_topic`
SDF parameters; omitting them preserves the old `/imu`, `/radar`, and
`/drone/gazebo/command/motor_speed` defaults.

## Verification and remaining work

The automated test launches the actual HIL world using a pseudo-terminal as a
virtual Teensy, receives framed sensor data, checks stationary gravity and ground
range, sends firmware-format telemetry, and checks all four published motor
speeds, including disarmed clamping.
It requires the Gazebo Harmonic Python bindings (`gz.transport13`, `gz.msgs10`)
and a working Ogre2 headless rendering environment.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q f_drone_simulation/f_drone_bringup/test/test_mini_shell_hil.py
```

This passes without a physical Teensy and does **not** establish stable flight,
altitude hold, measured latency, or real-time performance.
Before those tests, resolve these existing implementation issues:

- The firmware sensor parser writes the footer at index 28 of a 28-byte buffer; remove that out-of-bounds write and validate complete packets before using them.
- SIM mode must suppress physical motor outputs in both startup and the control loop.
- The serial bridge uses blocking reads, assumes telemetry arrives in one complete read, and lacks a command timeout; fragmented or stopped telemetry is not handled robustly.
- World reset closes the bridge's serial port without reopening it; fully stop and relaunch this HIL launch for now, with firmware state reset separately.
- Sensor callbacks and serial packing currently share data without synchronization; invalid range samples retain the previous range.
- Firmware mass, hover feed-forward, gains and range offset need checking for mini-shell, whose CAD model totals about 0.208 kg; the checked-in firmware still declares about 1.079 kg.

Successful sensor fusion on the previous drone supports reusing the bridge, but
these checks are still necessary for control of this airframe.
