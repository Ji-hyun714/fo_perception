# 차선 ROI 사용자 조정 기능 — 개발 계획서

작성일: 2026-07-23
대상 패키지: `lane_detection` (`src/ws_cam2/src/lane_detection`)

---

## 1. 목표

사다리꼴 ROI 마스크(`applyRoiMask`)의 범위를 **사용자가 실행 중에 직접 조정**할 수 있게 한다.
조정 결과는 ROI 범위를 나타내는 **가상의 선(경계선)** 으로 실시간 시각화한다.

### 조정 방식 (요구사항)
1. **디버그 창 트랙바** — 슬라이더로 파라미터 조절
2. **변수(숫자) 입력** — 값을 직접 지정 (ROS 파라미터 / rqt_reconfigure)
3. **마우스 끌어놓기(drag)** — 창 위에서 직접 드래그하여 ROI 범위 조절

---

## 2. 현재 구조 분석

### 조정 대상 파라미터 (`LaneConfig`, LaneDetection.hpp:234~238)
| 변수 | 기본값 | 의미 |
|---|---|---|
| `roi_mask_enabled` | true | ROI 마스크 적용 여부 |
| `roi_mid_offset_px` | 0 | 중심 오프셋 (양수=오른쪽, 음수=왼쪽) |
| `roi_half_top_ratio` | 0.45 | 상단(원거리) 반폭 비율 |
| `roi_half_bot_ratio` | 0.35 | 하단(근거리) 반폭 비율 |

### 마스크 생성 로직 (`applyRoiMask`, LaneDetection.cpp:5285~)
- BEV 이진영상(`bev_size`, 기본 640×640) 좌표계에서 동작.
- 행 `y`마다 `t = y/h`, `half_ratio = top + (bot-top)*t`, `half_width = w*half_ratio`.
- `mid_img = w/2 + roi_mid_offset_px` 기준 좌/우 밴드만 남기고 중앙은 제거.
- 즉 **좌/우 내측 경계 2개(사다리꼴 빗변) + 중앙선**이 ROI 범위를 규정한다.

### 현재 한계
- 위 4개 값은 **생성자에서 YAML 1회 로드** 후 변경 불가 (런타임 setter/getter 없음).
- 노드(`yolopv2_cam2_node.cpp`)의 debug 창은 `getDebugImage()` **모자이크(320×240 셀 스케일)** 라
  마우스 좌표→BEV 좌표 역매핑이 부정확 → **드래그 조작 창으로 부적합**.

### 설계 결론
- 모자이크 위 드래그 대신 **전용 1:1 BEV 크기 컨트롤 창**을 신설해 좌표 매핑을 단순화한다.
- 세 입력 경로(트랙바/파라미터/마우스)는 모두 동일한 setter를 호출하고 서로 동기화한다.

---

## 3. 구현 계획

### S1. Detector 런타임 조정 API 추가 (LaneDetection.hpp / .cpp)
- getter: `GetBevSize`, `GetRoiMaskEnabled`, `GetRoiMidOffsetPx`, `GetRoiHalfTopRatio`, `GetRoiHalfBotRatio`
- setter: `SetRoiMaskEnabled`, `SetRoiMidOffsetPx`, `SetRoiHalfTopRatio(clamp 0~0.5)`, `SetRoiHalfBotRatio(clamp 0~0.5)`
- 시각화: `cv::Mat renderRoiMaskOverlay() const`
  - `applyRoiMask`와 **동일한 기하**로 좌/우 내측 경계선 + 중앙선을 그린 `bev_size` 캔버스 반환
  - 유지 영역은 회색 음영, 경계는 녹색(활성)/주황(비활성), 파라미터 텍스트 표시
  - ⚠️ `applyRoiMask`의 기하식을 복제하므로 **로직 변경 시 양쪽 동기화** 필요 (주석 명시)

### S2. 노드 컨트롤 창 + 트랙바 (yolopv2_cam2_node.cpp, debug_mode 시)
- 전용 창 `"ROI Mask Control"` 생성.
- 트랙바 4종: `enabled`(0/1), `mid_offset+w/2`(0~w), `half_top x100`(0~50), `half_bot x100`(0~50).
  - OpenCV 트랙바는 0~max 정수만 → 오프셋은 `+w/2` 시프트, 비율은 ×100 정수로 표현.
- 콜백에서 트랙바 값 → detector setter 반영.
- 매 프레임 `renderRoiMaskOverlay()` 결과를 `imshow` → 조정 결과 실시간 표시.

### S3. 마우스 드래그 (setMouseCallback)
- **좌버튼 드래그**: `half_ratio = |x - mid| / w`.  y < h/2 → `top_ratio`, else → `bot_ratio`.
- **우버튼 드래그**: `mid_offset = x - w/2`.
- 조정 후 트랙바 위치·ROS 파라미터를 동기화(재귀 방지 플래그 사용).

### S4. 변수(숫자) 입력 — ROS 파라미터 (런타임)
- 파라미터 선언: `roi_mask_enabled(bool)`, `roi_mid_offset_px(int)`, `roi_half_top_ratio(double)`, `roi_half_bot_ratio(double)`.
- `add_on_set_parameters_callback` 등록 → `ros2 param set` / rqt_reconfigure 로 즉시 반영.
- 초기값은 YAML 로드 후 detector 실제값으로 선언(일관성).

### S5. 동기화·재귀 방지
- 트랙바 ↔ 마우스 ↔ 파라미터 3자가 서로를 갱신하므로 `roi_updating_` 플래그로 콜백 재진입 차단.
- `setTrackbarPos` / `set_parameters` 프로그램 호출이 콜백을 재발화시키는 문제 처리.

---

## 4. 사용 시나리오 (완료 후)

```bash
# 노드 실행 (debug 창 + ROI 컨트롤 창)
ros2 run lane_detection yolopv2_cam2_node --ros-args -p debug_mode:=true

# (A) 트랙바: "ROI Mask Control" 창의 슬라이더 조절
# (B) 마우스: 컨트롤 창에서 좌드래그=반폭, 우드래그=중심
# (C) 숫자입력: 런타임 파라미터
ros2 param set /yolopv2_cam2_node roi_half_top_ratio 0.40
ros2 param set /yolopv2_cam2_node roi_mid_offset_px 20
```

---

## 5. 변경 파일 요약
| 파일 | 변경 |
|---|---|
| `include/lane_detection/LaneDetection.hpp` | ROI getter/setter, `renderRoiMaskOverlay` 선언, `<algorithm>` |
| `src/LaneDetection.cpp` | `renderRoiMaskOverlay` 구현 |
| `src/yolopv2_cam2_node.cpp` | 컨트롤 창·트랙바·마우스·파라미터 콜백, 매 프레임 오버레이 표시 |

## 6. 검증 계획
1. 빌드: `colcon build --packages-select lane_detection --cmake-args -DCMAKE_BUILD_TYPE=Release`
2. 트랙바/마우스/파라미터 각각 변경 → 컨트롤 창 경계선이 즉시 갱신되는지 확인.
3. 세 경로 상호 동기화(한쪽 변경 시 다른 쪽 값도 갱신) 확인.
4. `renderRoiMaskOverlay` 경계선과 실제 `applyRoiMask` 결과(bev_bin)의 절단 위치 일치 확인.
5. `debug_mode:=false` 에서 창 없이 파라미터 경로만 정상 동작하는지 확인.

## 7. 미결정/확인 필요
- 컨트롤 창 배경에 **실제 BEV 이진영상**을 겹칠지(현재 계획: 음영만). 필요 시 BEV getter 추가.
- 조정한 값의 **YAML 영구 저장** 기능 필요 여부(현재 계획: 런타임 한정).
