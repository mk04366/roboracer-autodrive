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
find_package(Eigen3 REQUIRED)
find_package(autodrive_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include the include/ directory
include_directories(include)

# ----------- EXECUTABLES ------------
add_executable(pid_controller_node
  src/pid_controller_node.cpp
)

add_executable(pure_pursuit
  src/pure_pursuit.cpp
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

# ----------- DEPENDENCIES ------------
ament_target_dependencies(pid_controller_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(pure_pursuit
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs autodrive_msgs tf2 tf2_geometry_msgs
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

# ----------- EIGEN FIX ------------
target_include_directories(pure_pursuit PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(pure_pursuit Eigen3::Eigen)

target_include_directories(pid_controller_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(pid_controller_node Eigen3::Eigen)

target_include_directories(ftg_controller_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ftg_controller_node Eigen3::Eigen)

target_include_directories(stanley_controller_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(stanley_controller_node Eigen3::Eigen)

target_include_directories(waypoints_loader PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(waypoints_loader Eigen3::Eigen)

# ----------- INSTALL TARGETS ------------
install(TARGETS
  pid_controller_node
  pure_pursuit
  ftg_controller_node
  stanley_controller_node
  waypoints_loader
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
