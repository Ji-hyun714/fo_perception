cmake_minimum_required(VERSION 3.8)
project(fo_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_can REQUIRED)
find_package(fo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


add_executable(SF2Publisher src/SF2Publisher.cpp)
ament_target_dependencies(SF2Publisher
  rclcpp
  vision_msgs
  visualization_msgs
  sensor_msgs
  radar_can
  fo_msgs
)

ament_export_dependencies(rosidl_default_runtime)

target_include_directories(SF2Publisher PRIVATE
  include
)

target_link_libraries(SF2Publisher
  pthread
)

install(TARGETS
  SF2Publisher
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
