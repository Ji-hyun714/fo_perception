# 차선 인식 개발 기획서 (2026-07-09)

bag `7_to_hoamji`(`/camera/image_rect`, 2237 frames)로 `yolopv2_cam2_node` 실측 후, 문제점과 해결안을 정리한 개발 기획서.

## 1. 검증 방법

- 노드를 **headless(구독 모드, `debug_mode:=false`)**로 실행, `/camera/image_rect` 재생 → `/camera/lane_result`(`fo_msgs/Cam2LD`) 수집.
- 분석 스크립트로 프레임별 quality / position / heading / model_a / model_da / view_range / availability 집계.
- 재생 rate 2 기준 **1318 프레임** 처리분 통계 + rate 1.5 기준 원시값 샘플링.

## 2. 정량 결과

| 항목 | Left | Right |
|---|---|---|
| quality 분포 | GOOD 1208 / WEAK 106 / NODET 4 | GOOD 1252 / WEAK 66 |
| availability=1 | 99.7% | 100% |
| view_range 중앙값 | 9.9 m | 10.8 m |
| view_range 평균 | 8.7 m | 10.0 m |
| position 표준편차 | **898.4** | **223.5** |
| heading 표준편차(rad) | **123.5** | **30.4** |
| width | 255(고정) | 255(고정) |

- 좌우 동시 가용률 99.7%. **검출 성립률 자체는 매우 높음.**
- 그러나 position/heading의 표준편차가 물리적으로 불가능한 수준(수백~수천) → **소수의 degenerate 프레임이 극단값 출력.**

## 3. 문제점 (우선순위순)

### P1. Cam2LD 계수→필드 매핑 오류 (off-by-one) — 치명
`helper_lane.cpp buildCam2LD`에서 3차 계수 `coeffs=[a,b,c,d]`([x³,x²,x,1])를 아래처럼 매핑:

```cpp
lane_mark_position   = coeffs[2];  // = c (heading slope)   ← 실제 횡위치가 아님
lane_mark_heading    = coeffs[1];  // = b (curvature)       ← 실제 heading이 아님
lane_mark_model_a    = coeffs[0];  // = a (curvature rate)
lane_mark_model_da   = coeffs[3];  // = d (lateral offset)  ← 실제 "위치"가 여기 들어감
```

**실측 증거(정상 프레임 f45):** `pos=0.14, head=-0.02, a=0.0007, da=1.32` — 물리적 횡offset(≈1.3m)이 `model_da`에 들어가 있고, `position`에는 heading slope(0.14)가 들어감.
물리 정의상 x=0(차량)에서 y=d=횡위치, dy/dx=c=heading. 즉 **한 칸씩 밀림.**
→ 하위 소비단(SF/planning)이 위치·heading을 잘못 받음. **영향 최대.**

- 위치: `src/ws_cam2/src/lane_detection/src/helper_lane.cpp:37-53`

### P2. degenerate fit이 GOOD로 통과 — 심각
view_range가 극소(0.1~1.7m)인데 품질 GOOD로 발행되는 프레임 존재.

**실측 증거:** `f15 L: q=GOOD pos=-607.53 head=115.23 a=-5.32 da=-172.74 vr=0.1` / `f195 L: da=1620.22 vr=1.3` / `f210 R: da=-835.41 vr=1.3`.
→ 근거리 소수 픽셀만 잡혔을 때 3차 피팅이 ill-conditioned가 되어 고차 계수가 폭발. RMSE는 (점이 적어) 작게 나와 게이트 통과.
이 프레임들이 통계의 std를 수백~수천으로 키운 주범이며, **downstream에 순간적으로 수백 m offset을 던짐.**

- 관련: `validateAndSmooth`/품질 게이팅 (`LaneDetection.cpp`), 게이팅 파라미터 `span_min_m_*`, `valid_view_range`

### P3. 출력 필드 미구현 (width/type/color)
`lane_mark_width=0xff`, `lane_mark_type=0xff`, `left/right_lane_color_information=0xff` 하드코딩.
→ 차선폭/실선·점선/색상 정보를 하위단이 사용 불가.

- 위치: `helper_lane.cpp:61-65, 88-91`

### P4. 원거리 커버리지 부족
view_range 중앙값 ~10m (설정 `lane_marker_forward_max_m=30`, `roi_xmax=70` 대비 낮음).
→ 원거리에서 마스크/피팅이 약함. 고속 주행 예견거리 부족 가능.

### P5. 처리량/실시간성
처리량 약 **20 fps** (1318 frames / ~67s @rate2). 소스 native ~16.7fps → 1x는 여유, 1.5~2x에서 프레임 드롭(~40%).
QoS `keep_last(1)` 단일 스레드라 부하 시 최신 프레임만 처리(드롭). 실서비스 프레임레이트 확정 필요.

## 4. 해결안

### P1 (즉시)
- `buildCam2LD`의 계수→필드 매핑을 물리 정의에 맞게 교정:
  - `lane_mark_position = coeffs[3]` (d, 횡위치)
  - `lane_mark_heading_angle = coeffs[2]` (c, heading)
  - `lane_mark_model_a`, `model_da`는 곡률/곡률변화 정의를 메시지 스펙과 재확인 후 확정.
- **주의:** 하위 소비단(SF/planning)이 현재의 (잘못된) 매핑에 이미 맞춰져 있을 수 있으므로, 스펙 문서와 소비 코드(`ld_node.cpp:152-164`, SF 파서)를 함께 확인해 **한쪽만 고쳐 깨지지 않도록** 동시 반영.
- 검증: 본 기획서의 headless 분석 스크립트로 position std가 정상(≤ 수 m)으로 떨어지는지 재측정.

### P2 (즉시)
- degenerate 게이트 추가: **`view_range_m < valid_view_range`(예 5m) 또는 `span_m` 미달 시 NODET/availability=0 강제.**
- 계수 sanity 클램프: `|d| > lane_w_max*2`, `|c|`, `|a|` 임계 초과 시 reject(하드게이트 확장). `rmse_hard_bad_m`만으로 부족 → **저-span·고-곡률 조합 게이트** 추가.
- 근본: 피팅 조건수 개선 — 점 수/커버리지 부족 시 3차 대신 1~2차로 차수 자동 강등(overfit 방지).

### P3 (중기)
- `lane_mark_width`: 좌우 동시 검출 시 `|d_L - d_R|`로 차선폭 산출해 채움.
- `lane_mark_type`(실선/점선): 마스크의 세로 연속성(런길이/공백비)으로 분류.
- 색상: HLS 흰/황 게이트(`hls_white_*`/`hls_yellow_*`) 결과를 color 필드에 반영.

### P4 (중기)
- 원거리 커버리지: BEV 원거리 해상도/`strip_*` 파라미터 재튜닝, 마스크 far-field 임계 완화.
- `roi_xmax` 대비 실제 유효 range 갭 원인(마스크 소실 vs 피팅 조기 종료) 분리 진단.

### P5 (중기)
- 목표 프레임레이트 확정 후, 초과 시 추론 스레드 분리 or 입력 다운스케일 검토.
- 실시간 계측: `log_process_time` 활성화로 단계별 처리시간 프로파일.

## 5. 로드맵

| 단계 | 항목 | 산출물 |
|---|---|---|
| 1주차 | P1 매핑 교정 + 소비단 정합, P2 degenerate 게이트 | 정상 std 재측정 리포트 |
| 2주차 | P3 width/type/color 구현 | 필드 채움 + 검증 |
| 3~4주차 | P4 원거리 튜닝, P5 실시간 프로파일 | range↑ / fps 리포트 |

## 6. 부록 — 재현 방법

```bash
# headless 성능 측정 (구독 모드 + bag play + 통계)
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lane_detection yolopv2_cam2_node --ros-args -p debug_mode:=false &
ros2 bag play /home/wise/Desktop/7_to_hoamji --rate 2
# /camera/lane_result 를 구독해 quality/pos/heading/vr 집계 (analyze_lane.py)
```
- 프레임 단위 육안 검증은 [[lane_bag_verify_guide.md]]의 뷰어(`bag_path:=`) 사용.
- 계수/게이팅 이론·변수 의미는 [[lane_detection_params.md]], 파이프라인 위치는 [[lane_source_map.md]] 참고.
