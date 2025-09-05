cmake_minimum_required(VERSION 3.8)
project(f_drone_simulation)

# External Packages
find_package(ament_cmake REQUIRED)
find_package(foxglove_bridge REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Internal Packages *compile these first*
find_package(f_drone_ground_control REQUIRED)

## Gazebo Versioning ## 
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
# Harmonic
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
find_package(gz-sensors8 REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz-transport REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "Compiling against Gazebo Harmonic")

# Create a library so that our Gazbeo environment is able to be found
add_library(DroneSerial
  SHARED
  plugins/src/DroneSerial.cpp)
add_library(DroneWorld
  SHARED
  plugins/src/DroneWorld.cpp)

target_include_directories(DroneSerial PRIVATE plugins/include)
target_include_directories(DroneWorld PRIVATE plugins/include)

target_link_libraries(DroneSerial PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SIM_VER}::gz-sensors${GZ_SIM_VER})

target_link_libraries(DroneWorld PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Create an install directory for our system
install(TARGETS DroneSerial DESTINATION lib/${PROJECT_NAME})
install(TARGETS DroneWorld DESTINATION lib/${PROJECT_NAME})

# Install model files to easily find in Gazebo
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Install project configuration files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

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
