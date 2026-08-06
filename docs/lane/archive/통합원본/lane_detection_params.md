# LaneDetection 내부 값 설명 (의미 + 이론적 근거)

`src/ws_cam2/src/lane_detection/include/lane_detection/LaneDetection.hpp`의 `LaneConfig` / `LaneResult` 변수들의 의미와 왜 사용하는지 정리.
슬라이딩 윈도우/계수 소스 위치는 [[lane_source_map.md]] 참고.

## lane_source_map.md 압축 요약

- 차선 로직은 `LaneDetection.hpp/.cpp` 두 파일에 집중.
- **슬라이딩 윈도우**(`computeSlidingWindowBase` + `runSlidingWindow`)는 정의만 있고 **현재 파이프라인 미사용(legacy)**.
- 실제 경로: `DetectFromMask` → **`findPixelsFromMask`**(차선 픽셀 추출) → **`fitPolynomial`** → **`calcLaneCoeffs`**(3차 가중최소자승, QR 분해) → **칼만 스무딩** → `coeffs_L/R`.
- 계수는 `cv::Vec4d` = **[x³, x², x, 1]** 순, 차량좌표계 횡방향 y. 최종값은 `LaneResult.coeffs_L/R`.

---

## 0. 좌표계 전제 (왜 이렇게 하나)

차선 인식은 이미지 픽셀이 아니라 **차량 지면 좌표계(m)**에서 다룬다. 카메라 원근 때문에 이미지에서는 평행한 차선이 소실점으로 모여 곡률·폭이 왜곡된다. 그래서 **BEV(IPM, Inverse Perspective Mapping)**로 지면을 위에서 본 평면으로 펴서 미터 단위로 다항식을 피팅한다 → 곡률/차선폭/거리 판정이 물리적으로 의미를 가진다.

| 변수 | 의미 | 이론 |
|---|---|---|
| `roi_xmin/xmax` (2~70m) | 지면 ROI 전방 범위 | 원거리는 픽셀 해상도가 급격히 나빠져 상한을 둠. X+ 전방 |
| `roi_ymin/ymax` (±10m) | 지면 ROI 횡 범위 | Y+ 우측 |
| `bev_size` (640×640) | BEV 이미지 해상도 | ROI를 이 격자에 매핑 |

---

## 1. 세그멘테이션 마스크 전처리 (`DetectFromMask`)

YOLOPv2가 준 차선 마스크를 정제.

| 변수 | 의미 | 이론 |
|---|---|---|
| `mask_sobel_thresh` (30) | Sobel-X 이진화 임계 | 차선은 세로로 길어 수평 밝기변화(세로 엣지)를 Sobel-X로 추출. 낮을수록 민감(노이즈↑) |
| `mask_close_kh` (9) | MORPH_CLOSE 수직 커널 높이 | 점선/끊긴 차선을 세로로 이어붙임(모폴로지 닫힘=팽창 후 침식) |

---

## 2. 색/엣지 기반 전처리 (`preprocess`, 카메라 원본 경로)

| 변수 | 의미 | 이론 |
|---|---|---|
| `gamma` (2.0) | 감마 보정 | out=in^(1/γ)로 어두운 노면 차선 대비 향상 |
| `sobel_thres` | Sobel abs 이진화 임계 | 세로 엣지 강조 |
| `adaptive_block_size`, `adaptive_C` | 적응형 이진화 파라미터 | 조명 불균일 시 국소평균(−C offset) 기준 이진화 → 그림자/역광에 강함 |
| `hls_white_*` (h/l/s min-max) | 흰 차선 색 게이트 | 흰색=고L·저S |
| `hls_yellow_*` (h/l/s min-max) | 황 차선 색 게이트 | HLS가 RGB보다 조명 변화에 강함. 황색=특정 H·고S |

---

## 3. SearchAround (SA) — 이전 프레임 기반 추적

**이론:** 매 프레임 전체를 훑는 대신 **직전 프레임 곡선 주변 좁은 밴드만** 재탐색 → 빠르고 인접 차선/노이즈 오검출 억제(temporal continuity). 추적 성공 시 기본 모드.

| 변수 | 의미 |
|---|---|
| `margin_sa` (30px) | 이전 곡선 주변 탐색 반폭 |
| `safe_gap_multiplier` (2.0) | fallback safe_gap = margin_sa × multiplier |
| `sa_anchor_x_start/end/step_m` | 근거리부터 일정 간격으로 앵커 창 배치 |
| `sa_anchor_win_w/h_px` | 앵커 창 크기 |
| `sa_anchor_strip_count/minpix` | 앵커 내 strip 개수/최소 픽셀 |
| `sa_anchor_min_valid_points` (12) | 유효 앵커 최소 개수. 미만이면 SW로 fallback |
| `sa_anchor_fallback_to_sw` | SA 실패 시 SW 전환 여부 |
| `sa_seed_use_prev_start`, `sa_seed_track_length_m`, `sa_seed_count_max` | 이전 계수로 seed 중심 생성 |
| `sa_init_win_w/h_px`, `sa_win_shrink_*`, `sa_min_win_*` | 원거리로 갈수록 창을 점점 축소(원근 반영) |
| `lane_edge_*` | centroid 기준 안쪽 엣지 탐색(run/hop) 파라미터 |
| `prev_seed_start_prior_half_width_px` | 시작상태 prior 반폭 |

---

## 4. Sliding Window (SW) — 초기/복구 탐색 (현재 strip 방식으로 대체)

**이론:** 이전 정보가 없을 때(초기화/추적 실패) 쓰는 고전 기법. BEV 하단 **히스토그램 좌/우 피크**를 시작점으로 잡고 위로 창을 쌓되 픽셀 무게중심으로 재중심화.

| 변수 | 의미 |
|---|---|
| `n_windows` (9) | 세로 분할 창 개수 |
| `win_h/w`, `margin_sw` | 창 크기/검색폭 |
| `minpix` (30) | 창 재중심 최소 픽셀 수(적으면 노이즈로 흔들림) |
| `sw_hist_bottom_divisor` (3) | 히스토그램 기준 하단 1/N |
| `sw_forward_limit_m` (25m) | SW 탐색 최대 전방 거리 |
| `min_window_gap` (150px) | 좌·우 창 최소 간격 |
| `expected_w` (300px≈3.5m), `tolerance`, `max_shift` | 기대 차선폭 prior — 한쪽만 검출돼도 반대편 추정 |
| `sw_use_strip_guided` | strip 기반 대체 방식 사용 |
| `strip_count`, `strip_empty_stop`, `strip_peak_min_value` | strip 분할/조기종료/피크 최소값 |
| `strip_noise_min_pixels`, `strip_noise_growth_ratio`, `strip_noise_max_occupancy_ratio` | 노이즈 블롭(넓게 퍼진 밝은 영역) 판정 → 차선과 구분 |
| `strip_min_guide_points`, `strip_band_margin` | 2차 가이드 피팅 최소 샘플 / 밴드 반폭 |

---

## 5. 다항식 피팅 (`fitPolynomial` / `calcLaneCoeffs`)

**이론:** 차선을 3차 다항식 y = ax³ + bx² + cx + d로 근사. 도로 곡선은 **클로소이드(곡률이 거리에 선형)**에 가깝고 3차식이 근거리에서 이를 잘 근사한다.
계수 의미: **d = 횡 offset, c ≈ heading, b ≈ 곡률, a ≈ 곡률변화율.**

- 근거리 가중 최소자승 + 미세 정규화(`kLambda=1e-9`)로 QR 분해(`cv::solve DECOMP_QR`) → 근거리(중요·정확) 점에 큰 가중치, 수치 안정성 확보.

| 변수 | 의미 |
|---|---|
| `lane_expected_w` (3.6m) | 기대 차선폭 |
| `lane_w_min/max` (2.5~5.0m) | 물리적으로 타당한 차선폭 범위 게이트 |

---

## 6. 4축 게이팅 + 검증 (`validateAndSmooth`)

검출 결과를 **NODET / BAD / WEAK / GOOD**로 판정.

| 변수 | 의미 | 이론 |
|---|---|---|
| `x_check` (5m) | 프레임 간 y 변화 평가 기준 거리 | |
| `span_min_m_weak/good` (5/10m) | 검출 커버리지(span) 하한 | 짧게만 잡히면 신뢰도↓ |
| `rmse_hard_bad_m` (0.5) | RMSE 치명 하드게이트 | 피팅오차 초과 시 즉시 BAD |
| `dy_hard_m` (1.0) | Δy 치명 하드게이트 @x_check | 튀는 오검출 차단 |
| `dkappa_hard` (0.05) | 곡률변화 Δκ 하드게이트 | |
| `min_rmse_ratio` (0.8) | RMSE 신뢰 비율 | 작을수록 엄격 |
| `quality_min_pixels_good/weak` (300/100) | 품질 등급 픽셀 기준 | |
| `quality_max_rmse_m_good/weak` (0.05/0.15) | 품질 등급 RMSE 기준 | RMSE=피팅이 픽셀에 맞는 정도 |
| `max_bad_frames` (10) | 연속 실패 시 KF 예측으로 버티는 한계 | |
| `valid_view_range` (5m) | Cam2LD view_range 유효 판정 | |
| `lane_sep_min_m` (1.5m) | 좌/우 최소 간격 게이트 | |
| `lr_slot_by_top_endpoint`, `lr_keep_closer_to_mid` | 좌/우 슬롯 재배치·충돌 처리 | |
| `weak_to_sw` (8) / `bad_to_sw` (3) | 연속 WEAK/BAD 누적 시 SA→SW 전환 임계 | 추적 실패 복구 |

---

## 7. 칼만 필터 스무딩 (KF)

**이론:** 프레임마다 계수가 튀는 것을 억제. 상태(계수/샘플)를 시간적으로 추정하고 **측정 신뢰도(R)**에 따라 측정 반영 정도를 조절.

| 변수 | 의미 | 이론 |
|---|---|---|
| `kf_R_good/weak` (0.1/1.0) | 품질별 측정노이즈 | WEAK일수록 R↑ → 측정 덜 믿고 예측 유지 |
| `use_geometry_kf`, `geom_kf_*` | 기하(y0/θ/κ) KF 옵션 | Q=프로세스노이즈, R=측정노이즈 |
| `use_sample_kf_masklite` | 샘플-벡터 KF 사용 | 계수 대신 특정 거리 횡위치 샘플을 상태로 추적 → 물리 의미·안정성↑ |
| `kf_sample_x_m` {5,8,12,16} | 샘플링 거리(m) | |
| `kf_sample_band_half_width_m` (0.5) | 슬롯 밴드 반폭 | |
| `kf_sample_min_points_per_slot` | 슬롯 최소 점 수 | |
| `kf_sample_Q_pos/vel_list` | 위치/속도 프로세스노이즈 | 모델 신뢰도 |
| `kf_sample_R_diag_list` {0.3,0.3,0.8,1.2} | 슬롯별 측정노이즈 | 원거리 샘플일수록 R↑(불확실) |
| `kf_sample_invalid_R` (100) | 측정 없을 때 큰 R | 사실상 예측만 |
| `rawfit/kf_sample/geom_kf_hold_frames` | 측정 없을 때 예측 유지 프레임 수 | |

---

## 8. ROI 마스크(사다리꼴) & 컨투어 필터

| 변수 | 의미 | 이론 |
|---|---|---|
| `roi_half_top_ratio` (0.45) | 상단(원거리) 반폭 비율 | 원근상 도로는 위가 넓음 |
| `roi_half_bot_ratio` (0.35) | 하단(근거리) 반폭 비율 | 아래가 좁음 → 사다리꼴 마스킹으로 도로 밖(하늘/연석) 제거 |
| `roi_mid_offset_px`, `roi_mask_enabled` | 중심 오프셋 / 사용 여부 | |
| `contour_min_area` (20) | 컨투어 최소 넓이 | 작은 블롭 제거 |
| `contour_min_ratio` (1.5) | 최소 종횡비(h/w) | 세로로 긴 것만 차선 후보 |
| `contour_merge_dist_x/y` | 컨투어 병합 거리 | 가까운 조각 병합 |

---

## 9. RViz 마커 (시각화, 검출 로직 무관)

`enable_lane_marker`, `lane_marker_frame_id`(camera_link), `lane_marker_forward_min/max_m`, `lane_marker_dx_m`, `lane_marker_line_width_m`, `lane_marker_origin_radius_m`, `lane_marker_z_m` — 마커를 그릴 전방 구간/샘플 간격/선 두께/높이.

---

## 10. LaneResult (프레임 출력)

| 필드 | 의미 |
|---|---|
| `coeffs_fit_L/R` | 피팅 직후 raw 계수 (품질 평가 기준) |
| `coeffs_L/R` | **KF 후 최종 계수 (발행/그리기용)** |
| `detected_fit_L/R` | 피팅 성공 여부 |
| `is_detected_L/R` | 검출 성립 여부 |
| `state_L/R` | NODET/BAD/WEAK/GOOD 최종 상태 |
| `quality_L/R` | GOOD/WEAK/BAD 품질(하위호환) |
| `rmse_fit_m_L/R` | 미터 단위 피팅 RMSE |
| `pixel_count_L/R` | 사용 픽셀 수 |
| `hard_fail_L/R` | 치명 하드게이트 위반 |
| `lane_start/end_m_L/R` | 차선 시작/끝 x (m) |
| `span_m_L/R` | 검출 커버리지 (x, m) |
| `view_range_m_L/R` | Cam2LD view range (m) |
| `availability_L/R` | Cam2LD availability (0/1) |
| `delta_y_m_L/R` | 이전 프레임 대비 Δy @x_check |
| `delta_kappa_L/R` | 곡률 변화량 |

---

## 파이프라인 한눈에

```
lane_mask (YOLOPv2)
  └─ DetectFromMask
       ├─ (마스크 전처리: Sobel-X, MORPH_CLOSE)
       ├─ findPixelsFromMask → pixels_L_/R_   (SA 추적 / SW·strip 복구)
       └─ fitPolynomial → calcLaneCoeffs       (3차 가중최소자승, QR)
            → coeffs_fit_L/R
            → validateAndSmooth (4축 게이팅: RMSE/span/Δy/Δκ)
            → 칼만 스무딩 (품질별 R, 샘플-벡터 KF)
            → coeffs_L/R  +  state_L/R
```
