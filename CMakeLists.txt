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

add_executable(multi_pose_navigator src/multi_pose_navigator.cpp)

ament_target_dependencies(multi_pose_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

install(TARGETS
  multi_pose_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  src/launch src/worlds src/description src/config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
#chmod +x scripts/my_python_node.py
install(PROGRAMS
  scripts/example_nav_through_poses.py
  scripts/example_nav_to_pose.py
  scripts/lidar_distance_node.py
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
