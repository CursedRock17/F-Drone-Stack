cmake_minimum_required(VERSION 3.10)
project(f_drone_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY examples/ DESTINATION share/${PROJECT_NAME}/examples
)
install(
  PROGRAMS scripts/teleop_lateral.py DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(mini_shell_tf test/test_mini_shell_tf.py TIMEOUT 90)
endif()

ament_package()
