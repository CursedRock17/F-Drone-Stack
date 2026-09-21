# Simulation Debug Notes — Why the Drone Flips in Gazebo

Date: 2026-07-02/03. Investigation of the "drone flips all around" problem.
Verdict up front: **it is not a CAD problem, not a physics problem, and not a
Gazebo problem. The model is flyable. The bug is a 90° frame-convention
mismatch between the IMU axes and the motor layout in the HIL control path.**

## Evidence (headless Gazebo tests, gz sim 9.5.0)

Test worlds and logs live in the session scratchpad
(`scratchpad/simtest/`): the real `model.sdf` with the DroneSerial/battery/
lidar plugins stripped and nothing else changed physically.

1. **f_drone + MulticopterVelocityControl** (the commented-out plugin block in
   `model.sdf`, enabled as-is): commanded 0.5 m/s climb → rose to 6.45 m and
   held, roll/pitch/yaw flat at 0.00°, zero angular rates for 20 s.
   Identical behavior to the known-good crazyflie run as a positive control.
   ⇒ inertia tensor, motor constants, spin directions, prop placement are all
   consistent and flyable.

2. **f_drone open loop**: published the firmware's "+roll" actuator pattern
   (actuators 0,1 faster than 2,3 — what dRehmFlight thinks are the two LEFT
   motors) → the body rotated purely about **+y** (mean ω_y ≈ 1.63 rad/s,
   ω_x ≈ ω_z ≈ 0) and flipped inverted within ~2 s.
   dRehmFlight damps roll with GyroX — which never moved. That is the flip.

## Root cause, in full

- The OnShape export placed the drone's **forward axis along SDF +y**
  (rangefinder at y=+0.055 is the nose, ESC/battery cables at −y are the
  back). The inertia corroborates it: ixx (2.60e-4) ≈ 3× iyy (0.83e-4)
  because the heavy components are strung along y.
- The dRehmFlight mixer numbering (`controlMixer()`, dRehmFlight.ino:819-838:
  1=Front-Left, 2=Back-Left, 3=Front-Right, 4=Back-Right) maps **perfectly**
  onto prop_1..prop_4 positions *if* front=+y and left=−x:
  | firmware motor | corner | SDF prop | position (x, y) | spin |
  |---|---|---|---|---|
  | 1 (pwm[0], actuator 0) | Front-Left  | prop_1 | (−0.051, +0.050) | cw |
  | 2 (pwm[1], actuator 1) | Back-Left   | prop_2 | (−0.051, −0.050) | ccw |
  | 3 (pwm[2], actuator 2) | Front-Right | prop_3 | (+0.051, +0.050) | ccw |
  | 4 (pwm[3], actuator 3) | Back-Right  | prop_4 | (+0.051, −0.050) | cw |
  (Diagonal pairs match, and the yaw mixing signs are consistent with these
  spin directions — the motor table itself is correct.)
- **But** the Gazebo IMU sensor reports accel/gyro in the *link frame*, where
  x is the drone's *right side*, not its front. dRehmFlight assumes the IMU
  x-axis is forward (that's the physical MPU6050 mounting convention its
  Madgwick sign flips are built for). DroneSerial.cpp forwards the IMU
  axes raw. Net effect: the firmware's roll axis is physically the pitch
  axis and vice versa. Roll error produces pitch motion → PID loops are fully
  cross-coupled → guaranteed tumble. Confirmed empirically by test 2 above.

## The fix (pick ONE, not several)

**Recommended: rotate the IMU sensor frame in `model.sdf` by +90° yaw** so the
virtual IMU is "mounted" the way dRehmFlight expects (x = forward):

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0 0 0 1.5708</pose>
  ...
```

Sensor x then points at link +y (front), sensor y at link −x (left), z up —
the MPU6050 x-forward/y-left/z-up convention. With that, all three loops
close with the correct sign (checked analytically: +roll_PID → left motors
faster → +GyroX in sensor frame → damped; pitch and yaw likewise).

Alternatives (equivalent, more invasive): re-export the CAD with x-forward so
the whole link frame rotates, or remap the actuator order in
DroneSerial::pwmToVelocity. Doing more than one of these re-breaks it.

**Bench check for the real drone:** the same constraint applies to hardware —
the physical MPU6050 chip x-axis must point at the front (between motors 1
and 3, away from the battery cables). If HIL was flipping but the real drone
flew, the physical chip is mounted correctly and only the sim IMU frame was
wrong.

## Secondary findings (not the flip, but worth fixing)

1. **DroneSerial crashes Gazebo when no Teensy is plugged in.**
   `initializeSerialPort` (DroneSerial.cpp:243-300): if `open()` fails it
   only prints, then calls `flock(-1, ...)` which fails → throws
   `std::runtime_error`. Should return early after a failed open.
2. **Duplicate OdometryPublisher** plugin instances in `model.sdf` (lines
   ~457 and ~587) — both publish to `/model/drone/odometry`.
3. The IMU `<orientation_reference_frame>` block mixes `NWU` localization
   with `grav_dir_x` (model.sdf:123-127). Currently harmless because the
   firmware only consumes accel/gyro (never the orientation quaternion), but
   it's confusing — and the comment in `getIMUdataSim` claiming Gazebo data
   is "NWD" is wrong: the Gazebo IMU is z-up, +9.81 on z at rest, same as a
   flat-mounted MPU6050. Units conversion there (m/s²→g, rad/s→deg/s) is
   correct.
4. **Real pitch/roll inertia asymmetry**: ixx ≈ 3× iyy is *real* (mass along
   y). After the frame fix, expect the roll axis (about body-front) to be 3×
   more agile than pitch — per-axis PID gains will likely need to differ.
   The commented-out `*1.1926` boost hack in `pwmToVelocity` (DroneSerial
   .cpp:346-347) was probably compensating for a symptom of the frame bug —
   delete it.
5. Thrust budget sanity (all fine): hover ≈ 341 rad/s (sim units) ≈ PWM 213;
   kv mapping in pwmToVelocity caps at ≈ 483 rad/s < maxRotVelocity 600 →
   thrust/weight ≈ 2.0. The `kv = 11000/20` correctly bakes in
   `rotorVelocitySlowdownSim=20`.
6. DroneAutoSim.cpp forwards IMU axes raw and maps pwm[i]→actuator i just
   like DroneSerial, so it needs no change once the IMU sensor pose is fixed
   (and would have needed the same fix otherwise).

## Build environment (added 2026-07-08)

`colcon build` with Jazzy sourced failed on `find_package(gz-common)` because
this machine has THREE Gazebo stacks: system Harmonic debs (gz-sim8), system
Ionic debs (gz-sim9 — what the bare `gz sim` CLI runs), and ROS Jazzy's
vendored Harmonic under `/opt/ros/jazzy/opt/` (what `ros_gz_sim` launches).
Unversioned gz-* CMake names only exist in the vendor packages' `extra_cmake`
dirs, which are visible only after `find_package(gz_*_vendor)`.

Fixed in `f_drone_simulation/CMakeLists.txt` by finding the `gz_*_vendor`
packages before the `gz-*` ones. Verified: build succeeds and `ldd` shows all
three plugins linking the vendored libs — the same gz-sim8 that ros_gz_sim
runs, which is what matters for plugin load-time ABI compatibility.

Follow-ups worth doing:
- `f_drone_simulation/README.md:11-13` says ROS Humble + Gazebo Ionic; the
  branch is `humble`; the CMakeLists targets Harmonic via Jazzy vendors.
  Reconcile the docs to the real setup: **ROS Jazzy + vendored Harmonic**.
- Declare the `gz_*_vendor` packages in `package.xml` so rosdep/CI installs
  them.
- Plugins built against gz-sim8 will NOT load in the bare `gz sim` CLI
  (Ionic/gz-sim9). Launch through ros_gz_sim, or run
  `/opt/ros/jazzy/opt/gz_sim_vendor/bin/gz sim` explicitly.

## Ogre2 segfault in quadcopter_teleop.sdf (added 2026-07-09)

`gz sim worlds/quadcopter_teleop.sdf` crashed in
`SensorsPrivate::RenderThread` (segfault or Ogre `createDatablock` abort,
varies by timing). Cause: the **Sensors system was loaded twice** — once at
world level (quadcopter_teleop.sdf) and once inside the included
`backup_model.sdf`. Two instances race to create the same ogre2 scene/
materials. Fixed by removing the world-level copy; the drone models in this
repo are self-contained and carry their own Sensors system (the HIL and
autonomous worlds depend on that). Verified: server runs cleanly after.

Rule of thumb for this repo: a world must end up with **exactly one** Sensors
system after all includes. `crazyflie.sdf` and `real.sdf` keep theirs at
world level because their models don't carry one; any world that includes
`model.sdf`/`backup_model.sdf`/`blank.sdf` must NOT declare it.

## backup_model.sdf vs model.sdf (added 2026-07-09)

**`model.sdf` is the real drone.** `backup_model.sdf` is a stale draft carrying
x500-scale parameters: motorConstant 8.54858e-06, momentConstant 0.016, and —
fatally — base_link inertia ixx=iyy=3.95e-3 / izz=7.5e-3, a ~14 cm radius of
gyration on a 10 cm airframe. Physically impossible; the velocity controller
saturates the rotors trying to meet inertia-scaled moment demands and the
drone just skids and yaw-spins on the ground. Verified empirically: identical
controller + world flies model.sdf perfectly and cannot lift backup_model.

Changes made for Teensy-free flying:
- `worlds/quadcopter_teleop.sdf` now includes `model.sdf` (not backup_model).
- `model.sdf`: DroneSerial commented out (uncomment for HIL — note
  `drone_world_hil.sdf` needs it), MulticopterVelocityControl active.
- `backup_model.sdf` also got a (correctly configured) velocity controller
  block, but the model cannot fly until its inertia is fixed — treat the file
  as deprecated.

Verified headless on `worlds/quadcopter_teleop.sdf`: enable → climb 0.5 m/s
→ zero-twist hold: reached 5.52 m, 15 s hold with zero drift, level attitude,
spawn yaw held. Recipe:
```
gz topic -t /drone/enable  -m gz.msgs.Boolean -p 'data: true'
gz topic -t /drone/cmd_vel -m gz.msgs.Twist   -p 'linear: {z: 0.5}'
```
`/drone/gazebo/command/motor_speed` (gz.msgs.Actuators) also works, but only
with the controller left disabled — both write the same motor commands.

Minor: the world's physics block is named "2000hz" but max_step_size is
0.005 s (200 Hz). Flight is stable anyway; rename or set 0.0005 for honesty.

### Manual stress test (user, 2026-07-09)

Using the Teleop GUI plugin on `/drone/cmd_vel`, commanded 10 m/s forward
(well outside the small-angle regime `velocityGain`/`attitudeGain` were
tuned for) to deliberately force a crash and rule out the physics quietly
clamping/no-op'ing instead of actually being pushed to its limit. Result:
drone lost control and crashed — confirmed by the user to look physically
correct (attitude tumble, not a teleport/clip-through-ground/frozen-state
artifact). This is a second, independent confirmation (alongside the
position-hold test above) that the airframe, inertia, and collision
geometry in `model.sdf` are all sound — the failure mode is the controller
saturating at an unreasonable command, not a bug.

Follow-up if the flight envelope matters later: find the highest commanded
velocity that still holds a stable trajectory before tipping into
uncontrolled tumble, rather than just the point of total failure.

## backup_model.sdf: real per-part collision ported into model.sdf, backup left broken (2026-08-04)

Revisited the "model.sdf vs backup_model.sdf" call above after being shown
side-by-side renders. Correction to that entry: **the "perfectly balanced
variant" read on model.sdf was wrong.** Diffed both files link-by-link:
same STL meshes, same mass (0.2035 kg), sub-millimeter pose differences
consistent with two export passes of the *same* CAD assembly, not a
redesign. There is no balanced-vs-true-asymmetric distinction. The x500-scale
inertia call from before stands — that part was real.

What backup_model.sdf actually has that model.sdf didn't: real per-part
collision meshes (18, one for the frame deck + one per motor/leg/battery/PCB/
ESC/MCU/IMU) instead of model.sdf's single crude bounding box. That's a
genuine fidelity advantage worth keeping.

Fixed backup_model.sdf's inertia (copied model.sdf's tensor, same geometry)
and motor constants (x500 template → model.sdf's tuned values, same as
before) and it *still* produced exactly zero joint response to any actuator
command — confirmed via `/world/.../model/drone/joint_state`, angular
velocity ~1e-20 rad/s regardless of commanded speed (0 to 450 rad/s tested).
Isolated one variable at a time, all ruled out, no change in any case:
- real STL collision vs. simplified box collision
- `timeConstantUp`/`timeConstantDown` (present in backup, absent in model.sdf)
- the motor plugin XML itself (swapped in model.sdf's exact working block,
  retargeted to backup's joint/link names)
- `pose relative_to=` chaining vs. model.sdf's absolute-pose convention

Topic wiring confirmed correct (`gz topic -i` shows all 4 motor plugins
subscribed) and the file passes `gz sdf -k`. Root cause not found — something
in backup_model.sdf's structure silently prevents the physics engine from
applying joint velocity commands, independent of every difference tried.
**Do not spend more time on backup_model.sdf directly without a new lead;**
this was a thorough elimination pass, not a shallow one.

Pragmatic resolution: ported backup_model.sdf's real per-part collision
meshes into model.sdf (the file that actually works), rather than continuing
to debug backup's dead joints. Script: session scratchpad
`simtest/add_real_collisions.py`. Change: `model.sdf`'s single
`0.1029x0.09925x0.10` bounding box collision shrunk to a `0.105x0.105x0.003`
deck-plate box (`base_link_deck_collision`), plus 13 new `<collision>`
elements added — one per mounted-part visual (legs, motors, battery, MCU,
IMU, custom PCB, ESC), each using the same mesh and pose as its matching
visual. The main frame mesh (base_link.stl) is intentionally *not* used
directly as collision, matching backup_model.sdf's own choice to box the
deck rather than collide against that mesh raw.

Verified headless on the same climb/hold recipe as above: reached 5.53 m,
held with zero drift, dead-level attitude — statistically identical to the
pre-change box-collision result (5.52 m). `backup_model.sdf` is now
redundant; `model.sdf` has both the correct physics and the real collision
fidelity. Treat `backup_model.sdf` as dead/do-not-use going forward — it has
an unresolved bug that prevents any motor command from producing joint
motion, on top of already being superseded by this port.

## How to re-verify after the fix

1. Headless regression (no Teensy): the velocity-control world in the
   scratchpad, or equivalently uncomment MulticopterVelocityControl —
   should still climb/hover level (the frame fix doesn't touch physics).
2. HIL smoke test: with the Teensy attached, command pure roll from the
   firmware and confirm the sim rotates about the axis the firmware calls
   roll (GyroX response, not GyroY); then pitch; then yaw. Then attempt
   hover.
