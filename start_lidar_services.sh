#!/bin/bash

# 시스템 종료 시 모든 백그라운드 프로세스를 안전하게 종료
trap "kill 0" SIGINT SIGTERM EXIT

# 기존 ROS2 로그 삭제
rm -rf ~/.ros/log/*

source /home/wise/fo_perception/install/setup.bash

# =================================================================

# LiDAR velodyne driver
ros2 launch velodyne velodyne-11_201-nodes-VLP16-launch.py &
sleep 2

ros2 launch velodyne velodyne-12_201-nodes-VLP16-launch.py &
sleep 2

ros2 launch velodyne velodyne-13_201-nodes-VLP16-launch.py &
sleep 2


# LiDAR 객체 인지 트래킹
docker exec -i autoware_oang bash -lc 'bash /home/oang/autoware_oang/tracking/run_tracking.sh' &
sleep 1


# # LiDAR 지면 제거
# ros2 launch velo16_ground multi_ground_removal.launch.py &
# sleep 1


# # LiDAR TF
# ros2 launch lidar_merge tf_ground_merge.launch.py &
# sleep 1


# # LiDAR merge
# # ros2 launch lidar_merge tf_ground_merge.launch.py &
# python /home/wise/fo_perception/src/Documents/lidar_frame_change/lidar_merge_xyz.py &
# sleep 1


# LiDAR fail_monitoring
python /home/wise/fo_perception/src/Documents/fail_monitoring2.py &
sleep 1


# # LiDAR clustering
# # ros2 run clustering tracking_velo &
# ros2 run clustering tracking_node_test &
# sleep 1

# =================================================================

# ws_radar
# ros2 launch radar_can radar_nodes.launch.py &  ## general data
ros2 run radar_can RadarGeneral &  ## general data
sleep 1

ros2 run radar_can RadarParser &  ## track data
sleep 1

# =================================================================

# ws_cam2
# ros2 launch camera_stream cam_driver_g70.launch.py &
# ros2 launch camera_stream 106_cam.launch.py &
# ros2 launch camera_stream 220_g70_cam_251203.launch.py & # 251203 수정 @KNUT
# sleep 1

# # ros2 launch fo_cam2 cam2_bringup.launch.py &  # /cam2_data, ld, td 이런거 생성
# ros2 run fo_cam2 ld_interface --ros-args -p debug_mode:=false &
# sleep 1

# ros2 run fo_cam2 tdtlod_interface --ros-args -p debug_mode:=false &
# sleep 1

# # # ros2 launch yolo_bringup yolov11.launch.py input_image_topic:=/image_rect model:=/home/wise/fo_perception/ws_cam2/fo_g70_250903.pt &  # /yolo/tracking 생성(->td 노드로)
# ros2 launch yolo_bringup yolov11.launch.py &
# sleep 1

# # cam2_data 토픽 노드
# ros2 run fo_cam2 cam2_builder &
# sleep 1

## 260408
ros2 launch camera_stream g70_frontcam.launch.py &
sleep 1

ros2 launch lane_detection cam2_yolopv2.launch.py &
sleep 1

# =================================================================

# ws_gnss
ros2 launch ublox_dgnss ublox_mb+r_rover.launch.py &  ## driver
sleep 1

# RTK (MRD -> base에 socat 단방향 주입)
MRD=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0
MB=/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0
socat -u $MRD,b115200,raw $MB,b115200,raw &

## base pvt, rtk flag
python /home/wise/fo_perception/src/ws_gnss/fo_spi_info/basePvt_rtkFlag.py &
sleep 1

## calculate heading (heading 방어코드)
python /home/wise/fo_perception/src/ws_gnss/fo_spi_info/calculate_heading.py &
sleep 1

## gnss general
python /home/wise/fo_perception/src/ws_gnss/fo_spi_info/gnss_monitoring.py &
sleep 1

## gnss velocity
# python /home/wise/fo_perception/src/ws_gnss/fo_spi_info/calculate_velocity.py &
# sleep 1

# =================================================================

# 센서 퓨전
ros2 run fo_fusion SF2Publisher &
sleep 1

# =================================================================

# UDP 송신
ros2 run udp_package MsgSubscriber &
sleep 1

# =================================================================

# 스크립트가 종료되지 않도록 대기
wait
