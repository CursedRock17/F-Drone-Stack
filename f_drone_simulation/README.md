# Simulation
------------------------------
All work in simulation will be features here. Whether we go with IssacSim or Gazebo.
We can also put data visualization stuff here: RViz, Foxglove, *Plot Juggler*.
We can also store all of model information here (sdf, urdf, world) to prevent going out.

## Machine Learning
The MCAP and BAG files may be held here, IDK, I might make another section for all that

## Required Installs
- Install [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) Humble
- Install [Gazebo](https://gazebosim.org/docs/latest/install_ubuntu/) Ionic
- Install [GZ-ROS Bridge](https://gazebosim.org/docs/latest/ros_installation/) Humble
- Install [Foxglove](https://foxglove.dev/download)
        
## Remapping mesh names for proper rendering 
 - Gazebo (SDF): 
    <uri>assets/front_wing.stl</uri>
 - RViz (URDF): 
    filename="package://f_drone_simulation/models/f_drone/assets/front_wing.stl"

## CAD -> URDF Key Points
There are a few tidbits you should know, should you have to regenerate the files from 
CAD.
1) Ensure joints are properly named, any part that should be able to move should be a joint.
Name the joint: dof_part_name, so that it registers a joint in the URDF.
| Onshape   | URDF |
|----------:|--------------|
| Revolute  | Revolute  |
| Slider    | Prismatic |
| Fastened  | Fixed     |
2) Make sure your top level link (aka first part) is your base_link, probably named
`base_link` as well.
3) Make a fixed connnection to the Z-axis of the origin, then offset it the height of
your robot, so that it spawns in above the floor.
4) Ensure all of your parts have mass and they add up to the true mass of your robot,
this is important in increasing fidelity in the simulation environment.
5) Gazebo works in SDF files, call `gz sdf -p your_robot.urdf > your_robot.sdf`, 
replacing `your_robot` with the path and name to your actual robot. This model 
file is where all the plugins for your model will live, so ensure you have a backup
to push in your plugins, so you don't lose those settings.

## List of Working Tutorials
 - [Drone Control without ROS](./examples/drone_control_no_ros.md)
