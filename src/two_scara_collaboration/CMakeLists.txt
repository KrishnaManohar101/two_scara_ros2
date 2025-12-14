cmake_minimum_required(VERSION 3.8)
project(two_scara_collaboration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  two_scara_collaboration/scara_left_motion_planner.py
  two_scara_collaboration/scara_right_motion_planner.py
  two_scara_collaboration/block_spawner.py
  two_scara_collaboration/block_publisher.py
  two_scara_collaboration/gripper_action_server.py
  two_scara_collaboration/spawn_robots.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  launch
  urdf
  config
  meshes
  rviz
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_files()
# endif()

ament_package()
