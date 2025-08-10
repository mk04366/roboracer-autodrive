cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include the include/ directory
include_directories(include)

# Declare executable
add_executable(pid_controller_node
  src/pid_controller_node.cpp
)

add_executable(ftg_controller_node
  src/ftg_controller_node.cpp
)

add_executable(stanley_controller_node
  src/stanley_node.cpp
  src/stanley_controller.cpp
)
add_executable(waypoints_loader
  src/waypoints_loader.cpp
)

# Link dependencies
ament_target_dependencies(pid_controller_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(ftg_controller_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(stanley_controller_node
  rclcpp
  nav_msgs
  ackermann_msgs
  visualization_msgs
)
ament_target_dependencies(waypoints_loader
  rclcpp
  visualization_msgs
)

# Install target
install(TARGETS pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ftg_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS stanley_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS waypoints_loader
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
