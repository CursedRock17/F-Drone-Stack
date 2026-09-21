cmake_minimum_required(VERSION 3.10)
project(f_drone_gazebo)

find_package(ament_cmake REQUIRED)
find_package(f_drone_description REQUIRED)

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
# These must be versioned: unversioned configs (gz-commonConfig.cmake etc.)
# are not installed. Harmonic pairs with 5 / 13 / 10.
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "Compiling against Gazebo Harmonic")

# One shared library per Gazebo system plugin.
add_library(DroneSerial  SHARED plugins/src/DroneSerial.cpp)
add_library(DroneWorld   SHARED plugins/src/DroneWorld.cpp)
add_library(DroneAutoSim SHARED plugins/src/DroneAutoSim.cpp)

foreach(tgt DroneSerial DroneWorld DroneAutoSim)
  target_include_directories(${tgt} PRIVATE plugins/include)
endforeach()

target_link_libraries(DroneSerial PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SIM_VER}::gz-sensors${GZ_SIM_VER})
target_link_libraries(DroneWorld PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
target_link_libraries(DroneAutoSim PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SIM_VER}::gz-sensors${GZ_SIM_VER})

install(
  TARGETS DroneSerial DroneWorld DroneAutoSim
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Calibration / bench tooling (thrust_sweep.py, imu_axis_check.py)
install(
  PROGRAMS scripts/thrust_sweep.py scripts/imu_axis_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
