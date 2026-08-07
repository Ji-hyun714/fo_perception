cmake_minimum_required(VERSION 3.8)
project(radar_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)

add_executable(RadarGeneral src/radar_general_node.cpp)
add_executable(RadarBagNode src/radar_bag.cpp)
add_executable(RadarParser src/radar_parser.cpp)
add_executable(RadarMarker src/radar_marker.cpp)
add_executable(RadarVel src/radar_parser_vel.cpp)

ament_target_dependencies(RadarGeneral rclcpp std_msgs fo_msgs)
ament_target_dependencies(RadarBagNode rclcpp std_msgs fo_msgs)
ament_target_dependencies(RadarParser rclcpp std_msgs fo_msgs)
ament_target_dependencies(RadarMarker rclcpp std_msgs fo_msgs visualization_msgs)
ament_target_dependencies(RadarVel rclcpp std_msgs fo_msgs visualization_msgs ublox_ubx_msgs)

install(TARGETS RadarGeneral RadarBagNode RadarParser RadarMarker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()