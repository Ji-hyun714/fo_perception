cmake_minimum_required(VERSION 3.8)
project(camera_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# ==================================================================
# Target: gst_ipcam_node
# ==================================================================
add_executable(gst_ipcam_node src/gst_ipcam_node.cpp src/GstAppsink.cpp)

# 타겟별로 필요한 모든 include 경로를 명시적으로 지정
target_include_directories(gst_ipcam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

# ament (ROS 2) 패키지 의존성 설정
ament_target_dependencies(gst_ipcam_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  yaml_cpp_vendor
  fo_msgs
)

# non-ament (시스템) 라이브러리 링크
target_link_libraries(gst_ipcam_node
  yaml-cpp
  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)
message(STATUS "YAML-CPP Libraries found by CMake: ${yaml-cpp_LIBRARIES}")

# ==================================================================
# Target: rectify_node
# ==================================================================
add_executable(rectify_node src/rectify_node.cpp)

# 타겟별로 필요한 모든 include 경로를 명시적으로 지정
target_include_directories(rectify_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# ament (ROS 2) 패키지 의존성 설정
# ament_target_dependencies가 cv_bridge의 include와 link를 모두 처리해줍니다.
ament_target_dependencies(rectify_node
  rclcpp
  sensor_msgs
  cv_bridge
  yaml_cpp_vendor
)

# non-ament (시스템) 라이브러리 링크
target_link_libraries(rectify_node
  ${OpenCV_LIBS}
)

# ==================================================================
# Python Scripts & Installation
# ==================================================================
install(PROGRAMS
  scripts/video_stream.py
  scripts/image_jpg_saver.py
  scripts/qos_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gst_ipcam_node
  rectify_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY yaml
  DESTINATION share/${PROJECT_NAME}
)

# ==================================================================
# Ament Linting & Package Definition
# ==================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()