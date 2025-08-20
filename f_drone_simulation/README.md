# Simulation
------------------------------
All work in simulation will be features here. Whether we go with IssacSim or Gazebo.
We can also put data visualization stuff here: RViz, Foxglove, *Plot Juggler*.
We can also store all of model information here (sdf, urdf, world) to prevent going out.

## Machine Learning
The MCAP and BAG files may be held here, IDK, I might make another section for all that

## Required Installs
Install [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) Humble
Install [Gazebo](https://gazebosim.org/docs/latest/install_ubuntu/) Ionic
Install [GZ-ROS Bridge](https://gazebosim.org/docs/latest/ros_installation/) Humble
Install [Foxglove](https://foxglove.dev/download)
        
## Remapping mesh names for proper rendering 
 - Gazebo (SDF): 
    <uri>assets/front_wing.stl</uri>
 - RViz (URDF): 
    filename="package://f_drone_simulation/models/f_drone/assets/front_wing.stl"
