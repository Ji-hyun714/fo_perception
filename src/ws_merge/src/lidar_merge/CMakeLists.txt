cmake_minimum_required(VERSION 3.8)
project(lidar_merge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(merge src/merge.cpp)
add_executable(tf_11 src/tf_11.cpp)
add_executable(tf_12 src/tf_12.cpp)
add_executable(tf_13 src/tf_13.cpp)

add_executable(ground_merge src/ground_merge.cpp)
add_executable(tf_ground_11 src/tf_ground_11.cpp)
add_executable(tf_ground_12 src/tf_ground_12.cpp)
add_executable(tf_ground_13 src/tf_ground_13.cpp)

add_executable(tfed src/tfed_ground_merge.cpp)


ament_target_dependencies(merge rclcpp sensor_msgs)
ament_target_dependencies(tf_11 rclcpp sensor_msgs tf2)
ament_target_dependencies(tf_12 rclcpp sensor_msgs tf2)
ament_target_dependencies(tf_13 rclcpp sensor_msgs tf2)

ament_target_dependencies(ground_merge rclcpp sensor_msgs)
ament_target_dependencies(tf_ground_11 rclcpp sensor_msgs tf2)
ament_target_dependencies(tf_ground_12 rclcpp sensor_msgs tf2)
ament_target_dependencies(tf_ground_13 rclcpp sensor_msgs tf2)

ament_target_dependencies(tfed rclcpp sensor_msgs tf2)


install(TARGETS
  merge
  tf_11
  tf_12
  tf_13
  ground_merge
  tf_ground_11
  tf_ground_12
  tf_ground_13
  tfed
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()