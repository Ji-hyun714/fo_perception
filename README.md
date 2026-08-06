## velodyne driver

ros2 launch velodyne velodyne-11_201-nodes-VLP16-launch.py

ros2 launch velodyne velodyne-12_201-nodes-VLP16-launch.py

ros2 launch velodyne velodyne-13_201-nodes-VLP16-launch.py


## 지면 제거 + merge

ros2 launch linefit_ground_segmentation_ros fl_segmentation.launch.py

ros2 launch linefit_ground_segmentation_ros fr_segmentation.launch.py

ros2 launch linefit_ground_segmentation_ros r_segmentation.launch.py

ros2 launch lidar_merge tf_ground_merge.launch.py


## clustering

ros2 run clustering tracking_node_test


## lidar fail 모니터링

python3 /home/wise/fo_perception/src/Documents/fail_monitoring2.py



## 카메라 명령어

카메라 드라이버 ros2 launch camera_stream g70_frontcam.launch.py

객체+차선 ros2 launch lane_detection cam2_yolopv2.launch.py


## 토픽명 정리(260407)

### 카메라쪽 publish되는 토픽명
/camera/image_raw  : Raw 이미지
/camera/image_rect : 왜곡 보정 이미지
/camera/det_bboxes : (debug mode일 때만) 객체 bbox 그려진 이미지
/camera/general    : 카메라 상태 정보
/camera/sf_objs    : SF노드로 보내는 객체 데이터
/camera/cam2data   : UDP 노드로 보내는 Cam2_XX 필드 전체 데이터

### 라이다쪽 publish되는 토픽명
/lidar_11_201/points       : 라이다 Raw points (fl)
/lidar_12_201/points       : 라이다 Raw points (fr)
/lidar_13_201/points       : 라이다 Raw points (r)

/nonground/lidar_11        : 라이다 지면제거 points (fl)
/nonground/lidar_12        : 라이다 지면제거 points (fr)
/nonground/lidar_13        : 라이다 지면제거 points (r)

/TFnonground/lidar_11      : 라이다 지면제거 TFpoints (fl)
/TFnonground/lidar_12      : 라이다 지면제거 TFpoints (fr)
/TFnonground/lidar_13      : 라이다 지면제거 TFpoints (r)
/TFnonground/merged_points : 라이다 clustering에 쓰이는 points

/lidar_markers/bbox        : 라이다 bbox 마커
/lidar_markers/id          : 라이다 id 마커
/lidar/sf_objs             : 라이다 sf 마커 (센서퓨전용)

/lidar2_general            : 라이다 general (UInt8MultiArray 형식, 순서는 아래와 같음)
(fl fail_flags, fl alive_counters, fl failure_states,
 fr fail_flags, fr alive_counters, fr failure_states,
  r fail_flags,  r alive_counters,  r failure_states)




ros2 run udp_package MsgSubscriber
cb --packages-select udp_package && si