cmake_minimum_required(VERSION 3.8)
project(f_drone_simulation)

# External Packages
find_package(ament_cmake REQUIRED)
find_package(foxglove_bridge REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Internal Packages *compile these first*
find_package(f_drone_ground_control REQUIRED)

# Install model files to easily find in Gazebo
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Add hooks to easily find objects
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
