cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies ###
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fo_msgs REQUIRED)

### export ###
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  image_transport
  yaml-cpp
  fo_msgs
)

### install ###
install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  script/image_rect_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME image_rect_publisher
)

install(PROGRAMS
  script/video_image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME video_image_publisher
)

install(PROGRAMS
  script/bag_image_player.py    ##bag 프레임 단위 재생 소스
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_image_player
)
  
install(PROGRAMS
  script/ld_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ld_recorder_node
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
