cmake_minimum_required(VERSION 3.10)
project(f_drone_description)

find_package(ament_cmake REQUIRED)

# Models are exposed on GZ_SIM_RESOURCE_PATH by the hooks below, so worlds
# refer to them as model://f_drone/... rather than by filesystem path.
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
