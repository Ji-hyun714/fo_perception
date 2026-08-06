cmake_minimum_required(VERSION 3.8)
project(udp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(radar_can REQUIRED)
find_package(fo_msgs REQUIRED)
# find_package(septentrio_gnss_driver REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(MsgSubscriber src/UDP_node_v4.1.cpp)

ament_target_dependencies(MsgSubscriber
  rclcpp
  sensor_msgs
  # radar_can
  fo_msgs
  ublox_ubx_msgs
  # septentrio_gnss_driver
)

target_include_directories(MsgSubscriber PRIVATE
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(MsgSubscriber
  yaml-cpp
  # pthread
)

install(TARGETS
  MsgSubscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()