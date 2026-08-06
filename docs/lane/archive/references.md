# ROI 기반 차선 인식 — 레퍼런스 한글 요약

작성일: 2026-07-09
목적: bag 영상 하단 절반을 ROI로 지정해 차선 인식 성능을 높이는 개발의 근거 자료.
아래는 웹에서 조사한 "ROI로 검출 성능을 개선한" 사례 5건 + 보조 자료를 한글로 정리한 것.

---

## 1. A-ROI — 2차선 검출용 적응형 ROI 추출

- **출처:** A New Adaptive ROI Extraction Method for Two-Lane Detection
  (International Journal of Automotive Technology, Springer, 2021)
  https://link.springer.com/article/10.1007/s12239-021-0141-0
- **ROI 방식:** 매 프레임 검출 결과를 이용해 **다음 프레임의 ROI를 적응적으로 갱신**한다.
  고정 ROI가 아니라, 직전에 찾은 차선 위치를 중심으로 관심영역을 좁혀가는 방식.
- **보고 성능:** 정확도와 실시간성을 동시에 개선했다고 보고. 고정 ROI 대비 불필요한
  영역 연산을 줄여 처리 속도를 높이면서 오검출을 감소.
- **우리 프로젝트 시사점:**
  1단계로 **고정 하단 ROI**를 도입한 뒤, 2단계에서 이 논문처럼 **이전 프레임 차선 기반
  적응형 ROI**로 확장하는 로드맵이 타당함. (본 개발기획서 S5 확장에 해당)

---

## 2. DROI — 동적 ROI 전역 탐색

- **출처:** Lane Detection Based on a Lightweight ... Global Search of Dynamic ROI
  (MDPI Applied Sciences, 2020, 10(7):2543)
  https://www.mdpi.com/2076-3417/10/7/2543
- **ROI 방식:** 도로 에지의 **곡률**과 **최대 안전거리(주행 속도 기반 예견거리)**를 이용해
  ROI를 동적으로 산출. 직선로에서는 좁고 길게, 곡선로에서는 좌우로 넓혀 잡음.
- **보고 성능:** 곡선 구간에서 고정 ROI 대비 검출 성공률이 개선됨. 커브에서 차선이
  ROI 밖으로 벗어나 놓치는 문제를 완화.
- **우리 프로젝트 시사점:**
  고정 하단 절반 ROI는 **곡선로에서 좌우 마진이 부족**해 차선을 자를 수 있음.
  → 곡선 구간에서는 ROI 좌우 폭을 동적으로 넓히는 보완이 필요(S5).

---

## 3. ACSNet — 적응형 Cross-Scale ROI 융합

- **출처:** Lane Detection Based on Adaptive Cross-Scale Region of Interest Fusion
  (MDPI Electronics, 2023, 12(24):4911)
  https://www.mdpi.com/2079-9292/12/24/4911
- **ROI 방식:** 중요한 앵커 지점을 **적응적으로 선택**하고, 서로 다른 스케일(해상도) 간의
  ROI 특징을 **융합(fusion)**한다. ROI를 단순히 "자르는" 용도가 아니라
  **"어느 영역을 얼마나 신뢰할지" 가중**하는 용도로 사용.
- **보고 성능(CULane 계열):**
  - 야간(night) 오차 0.18 → 0.09 (**약 −50%**)
  - 차선 없음(noline) 오차 0.23 → 0.15 (**약 −35%**)
  - shadow/dazzle(그림자·눈부심) 카테고리에서 유의한 개선.
- **우리 프로젝트 시사점:**
  ROI를 이진 마스킹(자르기)뿐 아니라 **근거리 가중 피팅**과 결합하면 효과적.
  현행 파이프라인의 근거리 우선 가중(weighted least squares)과 방향성이 일치함.

---

## 4. UFLD / Row-Anchor 계열 — 세로 탐색범위 한정

- **출처:** Ultra Fast Deep Lane Detection With Hybrid Anchor Driven Ordinal Classification
  (IEEE TPAMI, 2022–2024)
  https://dl.acm.org/doi/abs/10.1109/TPAMI.2022.3182097
- **ROI 방식:** 픽셀 단위 세그멘테이션 대신 **row anchor(행 앵커)** 방식으로,
  미리 정한 여러 가로줄 위에서만 차선 위치를 분류. 사실상 **세로 방향 탐색 영역을
  하단 위주로 한정**하는 것과 같은 효과.
- **보고 성능:** TuSimple 정확도 **약 96%**, **300+ FPS급** 초고속 실시간.
- **우리 프로젝트 시사점:**
  "세로 범위를 한정하면 속도와 정확도가 모두 좋아진다"를 대규모 벤치마크로 입증한 근거.
  하단 절반 ROI의 정당성을 뒷받침.

---

## 5. YOLOPv2 — 현행 모델 기준선 (ROI 없음)

- **출처:** YOLOPv2: Better, Faster, Stronger for Panoptic Driving Perception
  (arXiv:2208.11434)
  https://ar5iv.labs.arxiv.org/html/2208.11434
- **ROI 방식:** ROI 없이 **전체 프레임을 입력**으로 사용 (현재 우리가 쓰는 모델).
- **보고 성능:** BDD100K lane 정확도 **87.3%**, **91 FPS (V100)**.
- **우리 프로젝트 시사점(중요):**
  현행 모델은 **풀프레임으로 학습**되어 있음. 따라서 **추론 입력을 하단으로 크롭하면**
  학습 분포(스케일·소실점 위치)와 어긋나 **세그멘테이션 성능이 오히려 떨어질 위험**이 있음.
  → **ROI는 모델 입력을 자르지 말고, 추론 결과(lane_mask)에 후처리로 적용**해야 안전.
  (개발기획서 4.2 "마스크 후처리 ROI" 결정의 핵심 근거)

---

## 보조 참고 — 고전 OpenCV 파이프라인

하단 다각형(사다리꼴) 마스킹으로 하단 절반 ROI를 잡는 것이 **표준 관행**임을 보여주는 예:
- Real-Time Lane Detection with OpenCV (Medium)
  https://medium.com/@chathuraun/real-time-lane-detection-with-opencv-and-python-with-object-detection-using-yolo-v5-f3809e2fbcf5
- Simple-Lane-Detection (GitHub)
  https://github.com/NicoBenndorf/Simple-Lane-Detection

---

## 종합 정리

| # | 사례 | ROI 핵심 | 성능 요약 | 우리에게 주는 결론 |
|---|---|---|---|---|
| 1 | A-ROI | 이전 프레임 기반 적응 갱신 | 정확도+속도 개선 | 2단계 적응 ROI 로드맵 |
| 2 | DROI | 곡률+안전거리 동적 산출 | 곡선로 성공률↑ | 곡선 좌우 마진 확대 필요 |
| 3 | ACSNet | 스케일 간 ROI 가중 융합 | night −50%, noline −35% | ROI+근거리 가중 결합 유효 |
| 4 | UFLD | row-anchor 세로범위 한정 | TuSimple 96%, 300+FPS | 세로 한정의 정당성 입증 |
| 5 | YOLOPv2 | 풀프레임(ROI 없음) | lane 87.3%, 91FPS | **입력 크롭 금지 → 마스크 후처리 ROI** |

**두 줄 결론**
1. "하단 절반으로 세로 탐색범위를 한정"하는 것은 여러 논문·벤치마크가 성능·속도 이득을 입증한 검증된 방향이다.
2. 단, 현행 YOLOPv2는 풀프레임 학습 모델이므로 **입력을 자르지 말고 추론 결과 마스크에 ROI를 적용**하고, 곡선로 대비 좌우 마진과 적응 ROI로 확장하는 것이 안전한 경로다.
