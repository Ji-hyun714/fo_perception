cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(utils REQUIRED)
find_package(fo_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)       # bag 뷰어: bag 직접 읽기
find_package(rosbag2_storage REQUIRED)   # bag 뷰어: StorageFilter

# Shared includes
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

option(LANE_DETECTION_ENABLE_YOLOPV2_TRT "Build yolopv2_trt third-party modules" ON)

# ByteTrack library
add_library(bytetrack
  ByteTrack-cpp/src/BYTETracker.cpp
  ByteTrack-cpp/src/KalmanFilter.cpp
  ByteTrack-cpp/src/lapjv.cpp
  ByteTrack-cpp/src/Object.cpp
  ByteTrack-cpp/src/Rect.cpp
  ByteTrack-cpp/src/STrack.cpp
)

target_include_directories(bytetrack PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/ByteTrack-cpp/include
)

target_link_libraries(bytetrack PUBLIC Eigen3::Eigen)

# Core lane detection library
add_library(${PROJECT_NAME}_lib
  src/LaneDetection.cpp
  src/LDHelpers.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(LANE_DETECTION_ENABLE_YOLOPV2_TRT)
  set(YOLOPV2_TRT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/yolopv2_trt)
  set(YOLOPV2_COMMON_HELPER_DIR ${YOLOPV2_TRT_ROOT}/common_helper)
  set(YOLOPV2_INFERENCE_HELPER_DIR ${YOLOPV2_TRT_ROOT}/InferenceHelper/inference_helper)
  set(YOLOPV2_IMAGE_PROCESSOR_DIR ${YOLOPV2_TRT_ROOT}/pj_tensorrt_perception_yolopv2/image_processor)

  if(EXISTS ${YOLOPV2_IMAGE_PROCESSOR_DIR}/CMakeLists.txt)
    add_subdirectory(${YOLOPV2_COMMON_HELPER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/yolopv2_common_helper)
    set(INFERENCE_HELPER_ENABLE_TENSORRT ON CACHE BOOL "With TensorRT? [on/off]")
    add_subdirectory(${YOLOPV2_INFERENCE_HELPER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/yolopv2_inference_helper)
    add_subdirectory(${YOLOPV2_IMAGE_PROCESSOR_DIR} ${CMAKE_CURRENT_BINARY_DIR}/yolopv2_image_processor)

    target_include_directories(${PROJECT_NAME}_lib PUBLIC
      ${YOLOPV2_IMAGE_PROCESSOR_DIR}
    )
    target_link_libraries(${PROJECT_NAME}_lib ImageProcessor)
  else()
    message(WARNING "yolopv2_trt kit not found under ${YOLOPV2_TRT_ROOT}; skipping ImageProcessor build")
  endif()
endif()

# Library dependencies
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  utils
  fo_msgs
)

# Lane publisher node
add_executable(ld_node src/ld_node.cpp)

target_include_directories(ld_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS 의존성 연결
ament_target_dependencies(ld_node
  rclcpp
  sensor_msgs
  visualization_msgs
  cv_bridge
  OpenCV
  utils
  fo_msgs
)

target_link_libraries(ld_node
  ${PROJECT_NAME}_lib   # 위에서 만든 LaneDetection 라이브러리 연결
  yaml-cpp              # 시스템 라이브러리는 직접 연결해야 확실함
)

# YOLOPv2 lane node
if(TARGET ImageProcessor)
  add_executable(yolopv2_node src/yolopv2_node.cpp)
  add_executable(yolopv2_cam2_node
    src/yolopv2_cam2_node.cpp
    src/helper_lane.cpp
    src/helper_tracking.cpp
    src/helper_obj_builder.cpp
  )

  target_include_directories(yolopv2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(yolopv2_cam2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(yolopv2_node
    rclcpp
    sensor_msgs
    visualization_msgs
    cv_bridge
    OpenCV
    utils
    fo_msgs
  )
  ament_target_dependencies(yolopv2_cam2_node
    rclcpp
    sensor_msgs
    visualization_msgs
    cv_bridge
    OpenCV
    utils
    fo_msgs
    rosbag2_cpp
    rosbag2_storage
  )

  target_link_libraries(yolopv2_node
    ${PROJECT_NAME}_lib
    yaml-cpp
    ImageProcessor
  )
  target_link_libraries(yolopv2_cam2_node
    ${PROJECT_NAME}_lib
    yaml-cpp
    ImageProcessor
    bytetrack
  )
endif()

# Tracking node
set(BYTETRACK_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/ByteTrack-cpp/include/ByteTrack/BYTETracker.h)

if(EXISTS ${BYTETRACK_HEADER})
  add_executable(tracking_node src/obj_tracking_node.cpp)

  target_include_directories(tracking_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(tracking_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    utils
    fo_msgs
  )

  target_link_libraries(tracking_node
    bytetrack
    yaml-cpp
  )
else()
  message(WARNING "ByteTrack headers not found under ${CMAKE_CURRENT_SOURCE_DIR}/ByteTrack-cpp; skipping tracking_node build")
endif()

# Install
install(TARGETS 
  ${PROJECT_NAME}_lib
  ld_node
  DESTINATION lib/${PROJECT_NAME}
)

if(TARGET yolopv2_node)
  install(TARGETS yolopv2_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET yolopv2_cam2_node)
  install(TARGETS yolopv2_cam2_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET tracking_node)
  install(TARGETS tracking_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
