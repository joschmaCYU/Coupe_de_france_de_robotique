cmake_minimum_required(VERSION 3.8)
project(robot_creation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(multi_pose_navigator src/multi_pose_navigator.cpp)
ament_target_dependencies(multi_pose_navigator rclcpp rclcpp_action nav2_msgs geometry_msgs)
install(TARGETS multi_pose_navigator DESTINATION lib/${PROJECT_NAME})

add_executable(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector rclcpp sensor_msgs)
install(TARGETS cylinder_detector DESTINATION lib/${PROJECT_NAME})

add_executable(robot_manager src/robot_manager.cpp)
ament_target_dependencies(robot_manager rclcpp std_msgs geometry_msgs)
install(TARGETS robot_manager DESTINATION lib/${PROJECT_NAME})

add_executable(invert_laser src/invert_laser.cpp)
ament_target_dependencies(invert_laser rclcpp sensor_msgs)
install(TARGETS invert_laser DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src/launch src/worlds src/description src/config scripts src/serial_communication
  DESTINATION share/${PROJECT_NAME}
)

add_executable(arduino_serial_node_read src/serial_communication/arduino_serial_node_read.cpp)
ament_target_dependencies(arduino_serial_node_read rclcpp std_msgs serial_driver geometry_msgs)
install(TARGETS arduino_serial_node_read DESTINATION lib/${PROJECT_NAME})

add_executable(arduino_serial_node_write src/serial_communication/arduino_serial_node_write.cpp)
ament_target_dependencies(arduino_serial_node_write rclcpp std_msgs serial_driver)
install(TARGETS arduino_serial_node_write DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
#chmod +x scripts/my_python_node.py
install(PROGRAMS
  scripts/example_nav_through_poses.py
  scripts/example_nav_to_pose.py
  scripts/lidar_distance_node.py
  scripts/initial_pose_publisher.py
  scripts/max_launch.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
