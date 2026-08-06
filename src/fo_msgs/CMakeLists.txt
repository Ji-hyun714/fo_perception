cmake_minimum_required(VERSION 3.8)
project(fo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Cam2Data.msg"
  "msg/Cam2LaneA.msg"
  "msg/Cam2LaneB.msg"
  "msg/Cam2LaneAdditionalData1.msg"
  "msg/Cam2LD.msg"
  "msg/Cam2OD.msg"
  "msg/Cam2ObstacleDataA.msg"
  "msg/Cam2ObstacleDataB.msg"
  "msg/Cam2StaticObjectData.msg"
  "msg/Cam2TDTLOD.msg"
  "msg/Cam2TD.msg"
  "msg/Cam2TL.msg"
  "msg/Cam2DataForSF2.msg"
  "msg/Obj2SF.msg"

  "msg/StatusCamera.msg"
  "msg/Object2D.msg"
  "msg/TrackResults.msg"
  
  "msg/Lidar2DataForSf2.msg"
  "msg/LidarobjListsForSF2.msg"

  "msg/RadarTrArray.msg"
  "msg/RadarTr.msg"
  "msg/CanFrame.msg"

  "msg/FusionForSF2.msg"
  "msg/FusionobjListForSF2.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cam2Data.msg"
  "msg/Cam2LaneA.msg"
  "msg/Cam2LaneB.msg"
  "msg/Cam2LaneAdditionalData1.msg"
  "msg/Cam2LD.msg"
  "msg/Cam2ObstacleDataA.msg"
  "msg/Cam2ObstacleDataB.msg"
  "msg/Cam2OD.msg"
  "msg/Cam2StaticObjectData.msg"
  "msg/Cam2TD.msg"
  "msg/Cam2TL.msg"
  "msg/Cam2TDTLOD.msg"
  "msg/Cam2DataForSF2.msg"
  "msg/Obj2SF.msg"
  "msg/StatusCamera.msg"
  "msg/Object2D.msg"
  "msg/TrackResults.msg"

  "msg/Lidar2DataForSf2.msg"
  "msg/LidarobjListsForSF2.msg"

  "msg/RadarTr.msg"
  "msg/RadarTrArray.msg"
  "msg/CanFrame.msg"

  "msg/FusionForSF2.msg"
  "msg/FusionobjListForSF2.msg"

  DEPENDENCIES std_msgs
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
