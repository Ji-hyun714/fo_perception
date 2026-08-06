#!/bin/bash

# 시스템 종료 시 모든 백그라운드 프로세스를 안전하게 종료
trap "kill 0" SIGINT SIGTERM EXIT

# 기존 ROS2 로그 삭제
rm -rf ~/.ros/log/*

source /home/wise/fo_perception/fo_msgs/install/setup.bash

# ws_merge
source /home/wise/fo_perception/ws_merge/install/setup.bash

ros2 launch lidar_merge tf_merge.launch.py &
sleep 2


# LiDAR clustering
source /home/wise/fo_perception/ws_lidar/install/setup.bash

ros2 launch patchworkpp ground.launch.py &
sleep 1

ros2 launch lidar_obstacle_detector detect.launch.py log_level:=warn &
sleep 1


# 스크립트가 종료되지 않도록 대기
wait
