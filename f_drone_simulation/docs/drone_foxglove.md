# Visualize the newer drone in Foxglove

This example uses `models/drone/urdf/drone.urdf` and `drone.sdf`, rather
than the mini-shell export. Run one example at a time because both use
`/robot_description`, `/joint_states`, `/tf`, and the default Foxglove port.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths f_drone_simulation
source install/setup.bash
ros2 launch f_drone_bringup simple_drone.launch.py
```

The launch opens Gazebo and the installed Foxglove desktop app and starts
both bridges and `robot_state_publisher`. In your signed-in Foxglove session,
connect to `ws://localhost:8765` and import
[simple_drone_foxglove.json](../f_drone_bringup/config/simple_drone_foxglove.json)
from the layout menu. Use `foxglove_gui:=false` if Foxglove is already open.

## How the model stays assembled

The layout reads the live `/robot_description` topic and positions the
links using TF. The URDF references meshes installed in the
`f_drone_description` package; a model folder named `drone` is not itself
a ROS package. Foxglove retrieves these meshes through its bridge.

Gazebo publishes `world → root` on `/drone/tf`, which the ROS–Gazebo bridge
forwards to `/tf`. The URDF's fixed joints, including `root → base_link`,
are published by `robot_state_publisher` on `/tf_static`; its four movable
propeller joints use the actual Gazebo `/drone/joint_states` data bridged
to `/joint_states`. This gives all 23 URDF links a path to `world`.

The ROS `/clock` topic comes from `/world/simple_drone_world/clock`.
The bridge and state publisher use simulation time, so this world name
must match the SDF world exactly.

## Check the result

With the workspace sourced in another terminal:

```bash
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo world prop__1_
```

Confirm that Foxglove displays the battery, electronics, legs, motors, and
propellers assembled together and follows Gazebo's pose. The saved layout
follows `base_link`; choose `world` as the display frame when inspecting
motion relative to the ground. Stop continuous TF inspection with Ctrl+C.

The regression in `f_drone_bringup` checks mesh resolution, simulation time,
all link transforms, joint-origin placement, and agreement between body TF
and odometry for this model. It also retains the mini-shell climb check;
passing the newer drone's visualization check does not certify its flight
controller or physical parameters.
