# 차선 인식 ROI 개발 기획서

작성일: 2026-07-09
선행 문서: [[lane_perf_diagnosis.md]] (실측 성능·문제점 P1~P5), [[lane_detection_params.md]] (파라미터 이론), [[lane_bag_verify_guide.md]] (검증 절차)

## 1. 목표

**최종 목표: 차선 인식 성능 향상.**
이번 단계 목표: 보유 bag(`7_to_hoamji`, `/camera/image_rect` 1280x720, 2237 frames)을 기준으로 **이미지 상/하 절반을 나누어, 차선이 잘 보이는 하단 절반을 ROI로 지정**하고 그 안에서 차선 찾기를 수행하도록 파이프라인을 구성·검증한다.

근거(0709 실측):
- 검출 성립률은 높으나(GOOD 90%+), **degenerate fit(P2)**과 **원거리 커버리지 부족(P4)**이 문제.
- 상단 절반(하늘·건물·원거리)은 차선 픽셀이 거의 없고 노이즈원 → 하단 ROI 집중으로 오검출원 제거 + 연산 절감이 기대됨.

## 2. 웹 사례 조사 (ROI로 검출 성능을 높인 사례 5건)

| # | 사례 | ROI 방식 | 보고 성능 | 시사점 |
|---|---|---|---|---|
| 1 | **A-ROI** — Adaptive ROI Extraction for Two-Lane Detection (Int. J. Automotive Technology, 2021) | 검출 결과로 ROI를 프레임마다 적응 갱신 | 정확도·실시간성 동시 개선 보고 | 고정 하단 ROI로 시작 → 이전 프레임 차선 기반 적응 ROI로 확장하는 로드맵 타당 |
| 2 | **DROI** — Dynamic ROI 전역 탐색 (MDPI Applied Sciences, 2020) | 도로 에지 곡률+최대 안전거리로 동적 ROI 산출 | 곡선로에서 고정 ROI 대비 검출 성공률 개선 | 곡선 구간에서 하단 고정 ROI의 좌우 마진을 넓혀야 함 |
| 3 | **ACSNet** — Adaptive Cross-Scale ROI Fusion (MDPI Electronics, 2023) | 중요 앵커를 적응 선택해 스케일 간 ROI 특징 융합 | CULane에서 shadow/dazzle 카테고리 유의 개선, night 에러 0.18→0.09(−50%), noline 0.23→0.15(−35%) | ROI는 자르기뿐 아니라 "어디를 믿을지" 가중에도 사용 가능 — 근거리 가중 피팅과 일치 |
| 4 | **UFLD/row-anchor 계열** (TPAMI 2022–2024) | row anchor로 세로 방향 탐색 영역 자체를 하단 위주로 한정 | TuSimple ~96%, 300+FPS급 실시간 | "세로 범위 한정"이 속도·정확도 모두에 유효함을 대규모 벤치마크로 입증 |
| 5 | **YOLOPv2** (arXiv:2208.11434) — 현행 모델 기준선 | 전체 프레임 입력 (ROI 없음) | BDD100K lane acc 87.3%, 91FPS(V100) | 현행 모델은 풀프레임 학습 — **입력을 자르면 학습 분포와 어긋날 수 있어** 마스크 후처리 ROI가 안전 (4.2 참조) |

보조 참고: 고전 OpenCV 파이프라인들(하단 다각형 마스킹)은 "하단 절반 ROI"가 표준 관행임을 보여줌.

**Sources:**
- [A New Adaptive ROI Extraction Method for Two-Lane Detection (Springer)](https://link.springer.com/article/10.1007/s12239-021-0141-0)
- [Lane Detection Based on Global Search of Dynamic ROI (MDPI)](https://www.mdpi.com/2076-3417/10/7/2543)
- [Lane Detection Based on Adaptive Cross-Scale Region of Interest Fusion (MDPI)](https://www.mdpi.com/2079-9292/12/24/4911)
- [Ultra Fast Deep Lane Detection With Hybrid Anchor (IEEE TPAMI)](https://dl.acm.org/doi/abs/10.1109/TPAMI.2022.3182097)
- [YOLOPv2: Better, Faster, Stronger for Panoptic Driving Perception (arXiv)](https://ar5iv.labs.arxiv.org/html/2208.11434)
- (보조) [Real-Time Lane Detection with OpenCV (Medium)](https://medium.com/@chathuraun/real-time-lane-detection-with-opencv-and-python-with-object-detection-using-yolo-v5-f3809e2fbcf5), [Simple-Lane-Detection (GitHub)](https://github.com/NicoBenndorf/Simple-Lane-Detection)

## 3. 현행 코드의 ROI 현황

이미 부분적 ROI 장치가 존재한다 (LaneDetection.hpp):
- **사다리꼴 ROI 마스크** `applyRoiMask`: `roi_mask_enabled`(기본 on), `roi_half_top_ratio=0.45`, `roi_half_bot_ratio=0.35`, `roi_mid_offset_px` — 좌우 폭 제한 위주.
- **지면 ROI**: `roi_xmin/xmax=2~70m`, `roi_ymin/ymax=±10m` — BEV 변환 범위.
- 그러나 **이미지 세로(상/하) 절반 컷은 없음** — 마스크 전처리(Sobel/CLOSE)와 BEV 변환이 풀프레임 마스크를 대상으로 동작.

## 4. 설계

### 4.1 ROI 정의 (1단계: 고정 하단 절반)
- 입력 1280x720 기준 **y ∈ [360, 720) 하단 절반**을 유효 영역으로 지정.
- 파라미터화: `roi_top_ratio` (기본 0.5, YAML/launch로 조절) — 추후 0.45~0.6 스윕.

### 4.2 적용 지점 — 마스크 후처리 ROI (권장)
YOLOPv2 **추론 입력은 그대로 풀프레임** 유지하고, **추론이 낸 lane_mask에 ROI를 적용**한다.
- 이유(사례 #5): 모델이 풀프레임으로 학습되어 입력 크롭은 분포 불일치(스케일/소실점 위치 변화)로 세그 성능이 오히려 떨어질 수 있음. 마스크 후처리는 무위험.
- 구현: `DetectFromMask` 초입에서 `lane_mask(0 ~ h*roi_top_ratio, :) = 0` (기존 `applyRoiMask` 사다리꼴과 AND 결합).
- (선택 2단계) 입력 크롭+리사이즈 방식은 별도 실험으로 비교만 수행.

### 4.3 bag 절반 분할 실험 설계
bag 2237 프레임을 **A/B 절반으로 분할**:
- **A(전반 ~1118): 파라미터 튜닝용** — roi_top_ratio 스윕, 게이트 조정.
- **B(후반): 검증 전용(hold-out)** — 튜닝에 사용 금지, 최종 성능 보고용.
- 프레임 지정 재생은 기존 뷰어(`bag_path:=`)와 headless 스크립트 재사용.

### 4.4 평가 지표 (0709 측정계 재사용)
GT가 없으므로 proxy 지표로 비교 (before/after):
| 지표 | 기대 방향 |
|---|---|
| degenerate 프레임 수 (vr<2m & q≥WEAK) | **감소** (P2 완화) |
| position/heading 표준편차 | **감소** (극단값 소멸) |
| GOOD 비율, 좌우 동시 가용률 | 유지(≥ 현행 99.7%) |
| view_range 중앙값 | 유지~소폭 감소 허용 (하단 ROI는 원거리 희생 가능 — 임계 -1m 이내) |
| 처리 시간/프레임 | **감소** (마스크 연산량 절감) |

### 4.5 리스크
- **원거리 정보 손실**: 720p에서 y=360 근방이 대략 소실점 부근 — ROI를 너무 내리면 20m+ 차선이 잘림. → `roi_top_ratio` 스윕(0.40/0.45/0.50/0.55)으로 view_range 손실 vs 노이즈 제거 트레이드오프 확인.
- **오르막/내리막**: 피치 변화 시 소실점이 이동 → 고정 절반 ROI가 차선을 자를 수 있음. 1단계는 고정, 2단계에서 사례 #1/#2처럼 이전 프레임 차선 기반 적응 ROI로 보완.
- P1(계수 매핑)·P2(게이트)는 ROI와 독립 — 0709 계획대로 병행 수정해야 지표 개선이 온전히 보임.

## 5. 구현 계획

| 단계 | 작업 | 산출물 |
|---|---|---|
| S1 | `roi_top_ratio` 파라미터 추가(YAML+launch), `DetectFromMask` 마스크 상단 컷 구현 | 코드 + 빌드 |
| S2 | bag A/B 분할 headless 측정 스크립트 정비 (0709 분석기 확장: degenerate 카운트 추가) | 측정 스크립트 |
| S3 | A 절반에서 ratio 스윕(0.40~0.55) → 최적값 선정 | 스윕 리포트 |
| S4 | B 절반 hold-out 검증, before/after 표 작성 | 성능 리포트 |
| S5 | (확장) 이전 프레임 차선 기반 적응 ROI(사례 #1·#2), 곡선로 좌우 마진 동적화 | 2단계 설계서 |

## 6. 완료 기준 (Definition of Done)

1. `roi_top_ratio` 로 하단 ROI가 켜지고/조절되며 뷰어에서 육안 확인 가능.
2. B 절반(hold-out)에서: degenerate 프레임 수 감소 **and** GOOD 비율·동시 가용률 유지 **and** view_range 중앙값 손실 1m 이내.
3. before/after 비교표가 본 문서에 추가됨.

## 7. 실측 결과 (2026-07-10) — before/after

**결론: 정적 하단 ROI(0.40~0.55)는 이 bag에서 검출 지표를 개선하지 못함 (off와 동일).**

결정적 dump(전 2237프레임 1회 처리, `runBagDump`)로 6개 ratio pass 측정:

| roi_top_ratio | degenerate | L pos std | vr median(m) | 판정 |
|---|---|---|---|---|
| 0.0(off) | 245 | 1189.4 | 10.04 | 기준 |
| 0.40 / 0.45 / 0.50 / 0.55 | 245 | 1189.4 | 10.04 | **off와 바이트 동일** |
| 0.90 | 0 | 0.0 | — | 검출 붕괴(전 NODET) |

- DoD 2 미충족: degenerate **감소 없음**(변화 0). GOOD/avail/vr은 유지되나 개선 목표를 달성 못함.
- 원인: 차선 픽셀이 이미 마스크 하단에 집중 → 상단 컷은 하늘·건물만 제거(4.5 원거리 손실 리스크가
  실제로는 "제거할 정보 자체가 없음"으로 나타남). degenerate(P2)는 근거리 피팅 문제라 상단 ROI 무관.
- 메커니즘 검증: 0.90에서만 붕괴 → 컷 로직 정상, 무변화는 씬 특성.

**시각자료:** `figs/sweep_metrics.png`(지표 추이), `figs/frame_1000_roi50.png`·`figs/frame_500_roi{0,50}.png`
(동작 중 파이프라인 모자이크). **원시 데이터:** `data/dump_r*.csv`. **분석:** `analyze_roi_sweep.py`.

**방향 전환:** S1 정적 ROI 종료. 우선순위를 0709 기획서 P1·P2 수정으로 이동, ROI는 S5 적응형으로 재설계.
