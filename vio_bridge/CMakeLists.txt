cmake_minimum_required(VERSION 3.8)
project(vio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vio_bridge_node src/vio_bridge_node.cpp)
target_include_directories(vio_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vio_bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(
  vio_bridge_node
  "rclcpp"
  "geometry_msgs"
  "px4_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs"
)

# OpenVINS -> PX4 bridge (subscribes nav_msgs/Odometry from ov_msckf).
add_executable(openvins_bridge_node src/openvins_bridge_node.cpp)
target_compile_features(openvins_bridge_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  openvins_bridge_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "px4_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# OptiTrack -> PX4 bridge (subscribes geometry_msgs/PoseStamped from VRPN).
add_executable(optitrack_bridge_node src/optitrack_bridge_node.cpp)
target_compile_features(optitrack_bridge_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  optitrack_bridge_node
  "rclcpp"
  "geometry_msgs"
  "px4_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

install(TARGETS vio_bridge_node openvins_bridge_node optitrack_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
