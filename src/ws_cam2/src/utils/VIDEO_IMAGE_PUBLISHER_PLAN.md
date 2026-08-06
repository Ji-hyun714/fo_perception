# Video Image Publisher Plan

## 목표

`utils` 패키지에 비디오 파일을 열어서 프레임 단위로 `sensor_msgs/Image`를 publish 하는 ROS2 Python 노드를 추가한다.

기준 구현은 `script/image_rect_publisher.py`의 구조를 따른다.

## 요구 파라미터

- `video_path` (`str`)
- `topic_name` (`str`, default: `/camera/image_raw`)
- `framerate` (`float`)
- `width` (`int`)
- `height` (`int`)

## 구현 방식

1. `cv2.VideoCapture`로 `video_path`를 연다.
2. `rclpy` timer를 `1.0 / framerate` 주기로 생성한다.
3. timer callback마다 비디오에서 프레임 1장을 읽는다.
4. 읽은 프레임을 `width`, `height` 크기로 맞춘다.
5. `cv_bridge.CvBridge`로 OpenCV 이미지를 `sensor_msgs/Image`로 변환한다.
6. `topic_name`으로 publish 한다.
7. 비디오 끝(EoF)에 도달하면 timer를 멈추고 노드를 종료한다.

## 예외 처리

- `video_path`가 비어 있으면 실행 실패
- 파일이 없으면 실행 실패
- 비디오를 열지 못하면 실행 실패
- `framerate <= 0`이면 실행 실패
- `width <= 0` 또는 `height <= 0`이면 실행 실패

## 패키지 반영 항목

- 신규 스크립트: `script/video_image_publisher.py`
- 실행 파일 등록: `CMakeLists.txt`의 `install(PROGRAMS ...)`

## 실행 예시

```bash
ros2 run utils video_image_publisher \
  --ros-args \
  -p video_path:=/home/wise/sample.mp4 \
  -p topic_name:=/camera/image_raw \
  -p framerate:=30.0 \
  -p width:=1280 \
  -p height:=720
```

## 확인 항목

- `ros2 topic hz /camera/image_raw`로 publish 주기 확인
- `rqt_image_view` 또는 downstream 노드에서 영상 수신 확인
- 입력 비디오 종횡비와 출력 크기 차이로 인한 왜곡 가능성 확인
