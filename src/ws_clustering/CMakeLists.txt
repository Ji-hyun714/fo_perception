cmake_minimum_required(VERSION 3.8)
project(clustering)

cmake_policy(SET CMP0074 NEW)  # PCL_ROOT 관련 경고 제거

# C++17 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 경고 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------
# 의존성 찾기
# -------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(vision_msgs REQUIRED)
find_package(fo_msgs REQUIRED)

# PCL include & lib
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------------------
# 실행 파일
# -------------------
add_executable(clustering_node src/clustering.cpp)
add_executable(tracking_node src/tracking.cpp)
add_executable(tracking_node_test src/tracking_test.cpp)
add_executable(tracking_velo src/tracking_velo.cpp)
add_executable(sub_interval src/sub_interval.cpp)

ament_target_dependencies(clustering_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  vision_msgs
  fo_msgs
)
ament_target_dependencies(tracking_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  vision_msgs
  fo_msgs
)
ament_target_dependencies(tracking_node_test
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  vision_msgs
  fo_msgs
)
ament_target_dependencies(tracking_velo
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  vision_msgs
  fo_msgs
)
ament_target_dependencies(sub_interval
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  vision_msgs
  fo_msgs
)

target_link_libraries(clustering_node
  ${PCL_LIBRARIES}
)
target_link_libraries(tracking_node
  ${PCL_LIBRARIES}
)
target_link_libraries(tracking_node_test
  ${PCL_LIBRARIES}
)
target_link_libraries(tracking_velo
  ${PCL_LIBRARIES}
)
target_link_libraries(sub_interval
  ${PCL_LIBRARIES}
)

# -------------------
# 설치
# -------------------
install(TARGETS
  clustering_node
  tracking_node
  tracking_node_test
  tracking_velo
  sub_interval
  DESTINATION lib/${PROJECT_NAME})

ament_package()
