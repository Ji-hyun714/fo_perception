
기존 인지부 1차 보드에서 실행하던 cam2_XX 필드 채우던 로직 ROS2 노드화

fo_cam2_builder.cpp : rclcpp 노드, yolo_msgs/DetectionArray 받아서 cam2_data 채우고 UDP 노드로 전송

모듈
- 차선 인식 노드
- 차선 인식 결과 받아서 Cam2_LD 채우기
- DetectionArray 받아서 처리(거리 계산 등) 후 Cam2_TD, OD, TL 채우기

헤더 
- fo_tracking에서 사용했던거 정의 
- lan