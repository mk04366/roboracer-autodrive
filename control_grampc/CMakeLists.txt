cmake_minimum_required(VERSION 3.14)
project(control_grampc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Debug flags
add_compile_options(-g -O0 -fno-omit-frame-pointer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# === Link GRAMPC ===
set(GRAMPC_DIR "/home/ammar/ros2_ws/GRAMPC_v2.2")

set(GRAMPC_INCLUDE_DIR "${GRAMPC_DIR}/include")
include_directories(${GRAMPC_INCLUDE_DIR})
set(GRAMPC_LIB "${GRAMPC_DIR}/libs/libgrampc.a")

include_directories(
  include
  ${GRAMPC_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# === Libraries ===
add_library(mpcc_model STATIC
  grampc/mpcc_model.c
)

add_library(path_utils STATIC
  src/path_utils.cpp
)

target_include_directories(path_utils PUBLIC include)

# === Executables ===
# Original MPCC node (keep for compatibility)
add_executable(mpcc_grampc_node
  src/mpcc_grampc_node.cpp
)

target_link_libraries(mpcc_grampc_node
  ${GRAMPC_LIB}
  mpcc_model
  path_utils
)

ament_target_dependencies(mpcc_grampc_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  message_filters
)

ament_target_dependencies(path_utils Eigen3)

# === Install ===
install(TARGETS
  mpcc_grampc_node
  mpcc_model
  path_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
