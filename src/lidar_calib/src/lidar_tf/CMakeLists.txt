cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)
project(lidar_tf)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(iv_points_tf
  src/iv_points_tf.cpp
)

ament_target_dependencies(iv_points_tf
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(iv_points_tf
  ${PCL_LIBRARIES}
)

install(TARGETS iv_points_tf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
