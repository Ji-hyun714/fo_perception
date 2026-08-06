# ROI_lane 작업 로그 (changelog)

작성일: 2026-07-09

## 문서
- `references.md` — ROI 검출 개선 사례 5건 + 보조자료 한글 요약
- `roi_dev_plan.md` — 하단 ROI 개발 기획서(사례·설계·A/B·지표·S1~S5)

## 관련 소스 변경 이력

### 이번 세션(뷰어/시각화)에서 변경 — 4개 파일
| 파일 | 변경 |
|---|---|
| `src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp` | bag 뷰어 `runBagViewer` 신규(단일 창 이미지+정보패널, 재생/정지, ←/→ 프레임, `,`/`.` 배속±0.1, GOTO 입력, 트랙바, ▶/⏸·SPEED UI). `runFramePipeline` 공용 추출. `debug_window_scale` 파라미터 |
| `include/lane_detection/LaneDetection.hpp` | `GetConfig()` getter 추가 |
| `launch/cam2_yolopv2.launch.py` | 인자 `debug_window_scale`, `bag_path`, `bag_topic` 추가 |
| `CMakeLists.txt` + `package.xml` | `rosbag2_cpp`, `rosbag2_storage` 의존성 추가 |

### 세션 이전부터 있던 변경(내 작업 무관)
- `src/ros2_udp/.../UDP_node_v4.1.cpp`, `camera_stream/*rtsp_cam_node*`, `*.yaml`

## 반영 예정(미구현)
- P1 계수 매핑 교정 / P2 degenerate 게이트 (0709 기획서)
- S1 `roi_top_ratio` 마스크 상단 컷 (진행 시작)

---

## S1 진행 로그

### 2026-07-09 — S1 구현 완료 (빌드 성공)
하단 세로 ROI를 **lane_mask 후처리**로 구현 (추론 입력은 풀프레임 유지).

변경 파일:
| 파일 | 변경 |
|---|---|
| `include/lane_detection/LaneDetection.hpp` | LaneConfig에 `float roi_top_ratio=0.0` 추가(0=off, 0.5=상단 절반 제거→하단만 ROI). YAML 로더에 `roi_top_ratio` 파싱. `SetRoiTopRatio()` 런타임 오버라이드 setter |
| `src/LaneDetection.cpp` | `DetectFromMask`에서 640x360 리사이즈 직후 `mask_u8` 상단 `rows*roi_top_ratio`행을 0으로 setTo (사다리꼴 applyRoiMask와 독립·직교) |
| `src/yolopv2_cam2_node.cpp` | ROS 파라미터 `roi_top_ratio`(default -1=YAML값 사용) 선언, 생성 직후 `[0,1)`이면 `SetRoiTopRatio()`로 오버라이드+INFO 로그. 뷰어 LaneConfig 패널에 `roi_top_ratio` 표시(>0이면 초록) |
| `launch/cam2_yolopv2.launch.py` | 런치 인자 `roi_top_ratio`(default -1.0) 추가·전달 |

사용 예:
```bash
# 하단 절반만 ROI로 실행 (스윕은 값만 교체)
ros2 launch lane_detection cam2_yolopv2.launch.py roi_top_ratio:=0.5 bag_path:=/home/wise/Desktop/7_to_hoamji
```

다음: S2 A/B 분할 headless 측정 스크립트에 degenerate 카운트 추가 → S3 ratio 스윕(0.40~0.55).

---

## S2~S4 진행 로그

### 2026-07-10 — 결정적 dump 측정 + 스윕/hold-out 검증 완료

**핵심 결론: 정적 하단 ROI(0.40~0.55)는 이 bag에서 무효과.** off와 바이트 단위 동일.

#### 방법 교정 (S2)
이전 세션의 `bag play --rate 2` 구독 측정은 비결정적 프레임 드롭으로 off/on 프레임 수가
3687 vs 4508로 어긋나 paired 비교 불가였다. → 노드에 **결정적 headless dump 모드** 추가:
bag을 직접 순회해 전 프레임을 1회씩 처리하고 프레임별 지표를 CSV로 기록.

변경 파일:
| 파일 | 변경 |
|---|---|
| `src/yolopv2_cam2_node.cpp` | `runBagDump()` 신규(창 없이 전 프레임 1회 처리→CSV). 파라미터 `dump_csv`(출력 CSV 경로, 비면 뷰어), `dump_img_dir`(debug 모자이크 저장 폴더), `dump_img_every`(N프레임마다 저장). main 분기 `isBagMode()&&isDumpMode()` |
| `ROI_lane/analyze_roi_sweep.py` | A/B(frame<1118 tune / ≥1118 hold-out) 분할, ratio별 degenerate·pos/head std·GOOD%·avail·vr median 산출 + off 대비 상대변화표 |

#### 결과 (S3/S4) — `7_to_hoamji` 2237프레임, 6 pass
| roi_top_ratio | degenerate | L pos std | vr median(m) | 판정 |
|---|---|---|---|---|
| 0.0(off) | 245 | 1189.4 | 10.04 | 기준 |
| 0.40 | 245 | 1189.4 | 10.04 | off와 동일 |
| 0.45 | 245 | 1189.4 | 10.04 | off와 동일 |
| 0.50 | 245 | 1189.4 | 10.04 | off와 동일 |
| 0.55 | 245 | 1189.4 | 10.04 | off와 동일 |
| 0.90 | 0 | 0.0 | — | **검출 붕괴(전 NODET)** |

- A/B hold-out 양 구간 모두 off와 차이 0 → hold-out에서도 ROI 효과 없음 재확인.
- 0.90에서만 붕괴 → **컷 로직은 정상 동작**, 0.40~0.55 무변화는 버그가 아닌 씬 특성.

#### 원인 (시각 규명)
debug 모자이크에서 **차선 픽셀이 이미 마스크 하단 좁은 띠에 집중** → 상단 40~55% 컷은
하늘·건물만 제거하고 검출 입력을 바꾸지 못함. degenerate(P2)는 **근거리 ill-conditioned 3차 피팅**이
원인이라 상단 ROI로 해결 불가.

#### 산출물 위치
- 지표 차트: `ROI_lane/figs/sweep_metrics.png`
- 동작 스크린샷: `ROI_lane/figs/frame_1000_roi50.png`, `frame_500_roi0.png`, `frame_500_roi50.png`
- 원시 CSV(6 pass): `ROI_lane/data/dump_r{00,040,045,050,055,90}.csv` (각 2237행)
- 재현: `python3 ROI_lane/analyze_roi_sweep.py ROI_lane/data/dump_r*.csv`

#### 다음 행동 (방향 전환)
1. 정적 하단 ROI는 품질 개선 없음 → **S3 최적값 선정 불필요, S1 정적 ROI는 여기서 종료.**
2. 우선순위를 **0709 기획서 P1(계수 매핑 off-by-one)·P2(저-span/고-곡률 하드게이트)** 로 이동.
3. ROI를 살리려면 정적 컷 대신 **이전 프레임 차선 기반 적응 ROI**(기획서 사례 #1·#2, S5)로 전환.

### 2026-07-10 — 원인 정밀 규명(이분 탐색) + 개선안 문서화
0.55~0.90 이분 탐색으로 컷 효과 **경계 = 0.65↔0.70** 확정: **유효 차선 픽셀이 마스크 하단 ~35%(row≥234)에만 존재**.
0.0~0.65는 출력 변화 0(상단은 빈 영역), 0.70+는 근거리 신호를 깎아 vr 붕괴.
→ S1 가설("상단=노이즈원") 반증. degenerate(P2)는 상단 노이즈가 아닌 근거리 ill-conditioned 피팅이 원인.
- 원인 분석·개선안 상세: **`root_cause_and_improvement.md`**
- 시각자료: `figs/onset_analysis.png`. 데이터: `data/dump_r{060,065,070,075,080}.csv` 추가.

### 2026-07-10 — P2 degenerate 게이트 + 차수 강등(D2) 구현
활성 경로가 `resultsFromRawFit()`(검증 없이 raw 3차 계수를 GOOD 출력)임을 규명하고,
`LaneDetection.cpp`에 P2 게이트 추가: x_check(5m) 예측 횡위치가 lane_w_max*2(10m) 초과 시 degenerate.

- **C(버림)**: degenerate → NODET. std 1189→6.2 정상화되나 both_availability 97.5→74.6로 하락.
- **D0**: 기존 `use_geometry_kf=1` 켜도 availability 회복 안 됨(66.7).
- **D2(차수강등)**: degenerate 픽셀을 **1차 직선 재피팅**(sane하면 WEAK 유지, 아니면 NODET).
  → **pos_std 1189→3.2, head_std 149.7→0.4, max|pos| 29498→53, both_av 94.1%.** C·before 모두 능가.

진행 상황 체크리스트: **`progress_checklist.md`**. 데이터: `data/`(dump_p2gate3/geomkf/d2 등은 scratchpad).
남은 것: D3/D4(곡률 추세 prior, 선택), P1(외부 규약 확인), 커밋(지시 대기).
