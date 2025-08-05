cmake_minimum_required(VERSION 3.8)
project(fdrone_ground_control)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

# Include our header files 
include_directories(include)

# Create our "library" which houses all of our ndoes
add_library(drone_nodes SHARED
  src/StateEstimatorNode.cpp
  src/WaypointProvisionNode.cpp
)
# Handle the linking and "packing" of our library
ament_target_dependencies(drone_nodes 
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
  "mavros_msgs"
)

target_include_directories(drone_nodes 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(drone_nodes 
  rclcpp::rclcpp 
  tf2_ros::tf2_ros 
  ${geometry_msgs_TARGETS}
  ${mavros_msgs_TARGETS})

# Create a Running Process (Visualization)
add_executable(visualization_process src/VisualizationProcess.cpp)
ament_target_dependencies(visualization_process
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
)
# Add our nodes to our running procses
target_link_libraries(visualization_process 
   drone_nodes 
   rclcpp::rclcpp
)

# Install our node into our library
install(
  TARGETS drone_nodes 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin 
)

# Install Our Headers 
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install Our Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Executables 
install(TARGETS 
  visualization_process
  DESTINATION lib/${PROJECT_NAME}
)

# Tell downstream packages where to find our headers
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(visualization_process)
# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  rclcpp
)

ament_package()
