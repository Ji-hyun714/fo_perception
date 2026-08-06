# G70 차선인식 bag 검증 README

ROS bag(`7_to_hoamji`)으로 차선인식 노드를 검증하고 rviz2에서 차선 마커를 확인하는 절차.

## 요약

- bag `7_to_hoamji`에는 **`/camera/image_rect` (sensor_msgs/Image) 토픽 하나만** 들어있음 (이미 rectify 완료된 이미지, 2237장, 약 134초).
- 차선인식 노드(`yolopv2_cam2_node`)의 기본 입력 토픽이 `/camera/image_rect`라서 **bag → 차선인식 노드로 바로 연결**됨.
- 따라서 카메라용 launch(`g70_frontcam.launch.py`)는 **사용하지 않음**. 그건 `gst_ipcam_node`(실시간 카메라) + `rectify_node`를 띄우는 실시간용이며, 카메라가 없으면 빈 프레임을 발행해 `rectify_node`가 "Empty image received"를 출력함.

## bag 위치

```
/home/wise/Desktop/7_to_hoamji
# (원본: /media/wise/SSD/bag/kds_0609/7_to_hoamji)
```

## 사전 준비 (빌드)

rviz용 차선 마커(`/camera/lane_marker`)는 **debug 빌드 + debug_mode 파라미터**에서만 발행됨.
`yolopv2_cam2_node`는 CMakeLists의 debug 조건 블록 안에서 빌드되므로, 마커가 필요하면 debug로 빌드되어 있어야 함.

```bash
cd /home/wise/fo_perception
source /opt/ros/humble/setup.bash   # 환경에 맞게
colcon build --packages-select lane_detection
source install/setup.bash
```

## 실행 절차

### 터미널 1 — bag 재생
```bash
source /home/wise/fo_perception/install/setup.bash
ros2 bag play /home/wise/Desktop/7_to_hoamji --loop
```
- `--loop`: 134초짜리 bag을 반복 재생 (rviz에서 계속 보기 편함).

### 터미널 2 — 차선인식 노드 실행
```bash
source /home/wise/fo_perception/install/setup.bash
ros2 launch lane_detection cam2_yolopv2.launch.py debug_mode:=true
```
- `debug_mode:=true` **필수** — 안 켜면 `/camera/lane_marker`(rviz용)와 `/camera/det_bboxes`(검출 이미지)가 발행되지 않음.
- `input_topic` 기본값이 `/camera/image_rect`라 bag과 그대로 일치 (변경 불필요).

### 터미널 3 — 발행 확인
```bash
source /home/wise/fo_perception/install/setup.bash
ros2 topic list | grep camera
ros2 topic hz /camera/lane_result    # 차선 결과 (fo_msgs/Cam2LD)
ros2 topic hz /camera/lane_marker    # rviz용 MarkerArray (debug_mode 필요)
ros2 topic hz /camera/det_bboxes     # 검출 시각화 이미지 (debug_mode 필요)
```

### 터미널 4 — rviz2
```bash
source /home/wise/fo_perception/install/setup.bash
rviz2
```
rviz2 설정:
1. **Fixed Frame** → `camera_link`
2. **Add → By topic → `/camera/lane_marker` → MarkerArray** (좌/우 차선 LINE_STRIP, 원점 SPHERE, 축 ARROW)
3. (원본 확인용) **Add → Image → `/camera/image_rect`** 또는 `/camera/det_bboxes`

## 프레임 단위 인터랙티브 뷰어 (bag 직접 읽기)

`bag_path`를 넘기면 노드가 `ros2 bag play` 없이 bag을 **직접 읽어** 프레임 단위로 재생/탐색하는 뷰어 모드로 동작한다. config + LaneResult 전체 값이 별도 `Lane Values` 창에 표시된다.

```bash
source /home/wise/fo_perception/install/setup.bash
ros2 launch lane_detection cam2_yolopv2.launch.py \
    debug_mode:=true \
    bag_path:=/home/wise/Desktop/7_to_hoamji \
    debug_window_scale:=1.5
```
- `bag_path` 지정 시 토픽 구독 대신 뷰어 모드. 이때는 `ros2 bag play`를 따로 띄우지 않는다.
- `bag_topic` 기본값 `/camera/image_rect` (bag 안 토픽명과 일치해야 함).
- 여전히 `/camera/lane_result`, `/camera/lane_marker` 등은 현재 보이는 프레임 기준으로 발행되므로 rviz2와 병행 확인 가능.

### 창 구성 (단일 창 `G70 Lane Viewer`)
- **상단**: 결과 오버레이 이미지 + 맨 위 **프레임 트랙바**(드래그로 이동)
- **하단**: 값 패널 (PLAYBACK / LaneResult / LaneConfig 전체)
- 패널 우상단에 **재생/정지 아이콘**(▶ 재생 / ⏸ 정지), `GOTO FRAME:` **입력 필드**, `last key code`(키 검증용) 표시

### 조작키 (이미지 창이 포커스인 상태에서)
| 키 | 동작 |
|---|---|
| `Space` | 재생 / 일시정지 |
| `←` / `→` | 이전 / 다음 프레임 |
| `.` / `,` | 배속 +0.1 / −0.1 (x0.1 ~ x4.0) |
| `0`~`9` 입력 후 `Enter` | 해당 프레임 번호로 점프 (0 ~ 전체-1) |
| `Backspace` | 점프 입력 숫자 지우기 |
| `q` 또는 `Esc` | 종료 |
| 트랙바 드래그 | 해당 프레임으로 이동 |

패널 상단 `GOTO: [ ___ ] / 전체` 박스에 입력 중인 프레임 번호가 표시되고, `Enter`로 이동한다. 그 위에 `frame: 현재 / 전체`, 재생상태(PLAY/PAUSE), 배속도 표시된다.

> 참고: 화살표 키는 이미지 창(`Lane Detection Process`)이 포커스인 상태에서 동작한다. 키가 안 먹으면 창을 한 번 클릭해 포커스를 준다.

## 토픽 정리

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/camera/image_rect` | sensor_msgs/Image | **bag이 발행** (입력) |
| `/camera/lane_result` | fo_msgs/Cam2LD | 차선인식 결과 (수치) |
| `/camera/lane_marker` | visualization_msgs/MarkerArray | **rviz 시각화용** (debug_mode 필요) |
| `/camera/det_bboxes` | sensor_msgs/Image | 검출 bbox 오버레이 이미지 (debug_mode 필요) |
| `/camera/cam2data` | fo_msgs/Cam2Data | Cam2 데이터 |
| `/camera/sf_objs` | fo_msgs/Cam2DataForSF2 | SF 연동 데이터 |

## 관련 파일

- 차선인식 노드: `src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp`
  - 입력 토픽 기본값 `/camera/image_rect` (89행)
  - 마커 퍼블리셔 `/camera/lane_marker` (169-170행), 마커 생성 `publishLaneMarkers()` (332-477행)
  - 마커 발행 조건: `debug_mode_ && enable_lane_marker_ && pub_lane_marker_` (334행)
- launch: `src/ws_cam2/src/lane_detection/launch/cam2_yolopv2.launch.py`
  - `debug_mode` 기본값 `false` (54-58행)
- 차선 마커 설정: `src/ws_cam2/src/lane_detection/yaml/kcity.yaml` (219-228행)
  - `enable_lane_marker: 1`, `lane_marker_frame_id: "camera_link"`

## 트러블슈팅

### "결과가 없다" — 원인은 노드 콘솔 로그로 판별한다
뷰어/노드를 실행한 **터미널의 로그**를 먼저 확인할 것. 원인별로 서로 다른 메시지가 찍힌다.

| 콘솔 메시지 | 원인 | 조치 |
|---|---|---|
| `FATAL ... Failed to initialize DetectionEngine` | TensorRT 엔진/모델 로드 실패 (경로·GPU) | 엔진 파일(work_dir), GPU/드라이버 확인. 이게 뜨면 어떤 프레임도 처리 못 함 |
| `FATAL Failed to open bag '...'` | `bag_path`가 틀림 | 경로 확인 (`/home/wise/Desktop/7_to_hoamji`) |
| `FATAL No messages on topic '...'` | `bag_topic`이 bag 안 토픽명과 불일치 | `ros2 bag info`로 토픽명 확인 → `bag_topic:=` 로 맞춤 (여기선 `/camera/image_rect`) |
| `WARN ... lane_mask is empty` | 추론은 됐으나 세그멘테이션이 차선을 못 잡음 | 입력 이미지/모델 mismatch. 아래 "검출 자체 확인" 참고 |
| `WARN Inference failed for one frame` | 추론 단계 실패 | 입력 해상도/포맷, 엔진 상태 확인 |
| 로그는 정상인데 화면이 아예 안 뜸 | X 디스플레이 없음(SSH 등) | 로컬 디스플레이에서 실행하거나 `ssh -X`. GUI(imshow) 창은 DISPLAY 필요 |

### 검출 자체 확인 (값이 계속 0 / NODET 일 때)
1. `Lane Values` 패널에서 `state_L/R`가 **NODET**이고 `pixel_count`가 0이면 → 세그멘테이션이 차선 픽셀을 못 잡은 것.
2. `coeffs_L/R`이 `[0,0,0,0]`이면 미검출. `coeffs_fit_L/R`에 값이 있는데 `coeffs_L/R`만 0이면 게이트(validateAndSmooth) 탈락.
3. 이미지 창(`Lane Detection Process`)에 오버레이가 전혀 없으면 debug 이미지가 비어 raw 프레임만 표시되는 상태 = 검출 실패.

### rviz 마커가 안 보임 (ros2 bag play 절차로 진행한 경우)
1. `debug_mode:=true`로 실행했는지 확인 (마커는 debug에서만 발행)
2. `ros2 topic hz /camera/lane_marker`로 발행 여부 확인
3. rviz **Fixed Frame**을 `camera_link`로 (TF가 없으면 status:Error)
4. `/camera/lane_result`는 debug 없이도 발행되므로 `ros2 topic echo /camera/lane_result`로 검출값부터 확인

### "Empty image received"
`g70_frontcam.launch.py`(카메라용)를 실행한 경우. bag 검증에서는 이 launch를 쓰지 말고 `cam2_yolopv2.launch.py`(+ `bag_path`)만 실행할 것.
