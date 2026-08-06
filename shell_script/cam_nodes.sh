#!/bin/bash

# 시스템 종료 시 모든 백그라운드 프로세스를 안전하게 종료
trap "kill 0" SIGINT SIGTERM EXIT

# 기존 ROS2 로그 삭제
rm -rf ~/.ros/log/*

source /home/wise/fo_perception/fo_msgs/install/setup.bash

# ws_cam2
source /home/wise/fo_perception/ws_cam2/install/setup.bash

ros2 run camera_stream gst_ipcam_node &
sleep 1

ros2 run camera_stream rectify_node &
sleep 1

ros2 launch fo_cam2 cam2_bringup.launch.py & 
sleep 1

# ros2 launch yolo_bringup yolov11.launch.py &
ros2 launch yolo_bringup yolov11.launch.py input_image_topic:=/image_rect model:=/home/wise/fo_perception/ws_cam2/fo_g70_250903.pt &
sleep 1


# 스크립트가 종료되지 않도록 대기
wait