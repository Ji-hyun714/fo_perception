cmake_minimum_required(VERSION 3.8)
project(fo_cam2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(yolo_msgs REQUIRED)
find_package(fo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_stream REQUIRED)

# common settings
set(FO_CAM2_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>

  # ${CMAKE_CURRENT_SOURCE_DIR}/include/fo_cam2
  ${CMAKE_CURRENT_SOURCE_DIR}/include/autocal/wafl3
  
  ${OpenCV_INCLUDE_DIRS}
)

# 모든 타겟이 사용할 공통 종속성 변수 생성
set(FO_CAM2_DEPENDENCIES
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "yolo_msgs"
  "fo_msgs"
  "message_filters"
  # "yaml-cpp"
  "yaml_cpp_vendor"
  "camera_stream"
  # "Eigen3::Eigen"
)

# 실행파일
add_executable(cam2_builder src/cam2_builder_node.cpp)
add_executable(cam2_builder_v2 src/cam2_builder_node_v2.cpp)
add_executable(ld_interface src/cam2_ld_interface_node.cpp src/lane_detection.cpp)
add_executable(tdtlod_interface src/cam2_obj_interface_node.cpp src/camera_geometry.cpp)
    

# 모든 실행 파일에 공통 설정 적용
foreach(TARGET cam2_builder cam2_builder_v2 ld_interface tdtlod_interface)
  # C++ 표준 설정
  target_compile_features(${TARGET} PUBLIC c_std_99 cxx_std_17)
  
  # 공통 인클루드 디렉터리 설정
  target_include_directories(${TARGET} PUBLIC ${FO_CAM2_INCLUDE_DIRS})

  # ament 종속성 설정
  ament_target_dependencies(${TARGET} ${FO_CAM2_DEPENDENCIES})

  # OpenCV 라이브러리 링크
  # target_link_libraries(${TARGET} PUBLIC ${OpenCV_LIBRARIES})
  # target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})
  target_link_libraries(${TARGET}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
  )
endforeach()

install(TARGETS
  cam2_builder
  cam2_builder_v2
  ld_interface
  tdtlod_interface
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
