# 차선 인식(Lane Detection) 연구 조사 보고서 (2021–2026)

> 작성일: 2026-07-06 · 대상: 최근 5년 딥러닝 기반 차선 인식 논문·연구자료·오픈소스
> 방법론: 다중 웹 검색 → 1차 출처(논문/공식 repo) 수집 → 주장별 적대적 3-표 검증 → 종합

---

## 0. 요약 (Executive Summary)

최근 5년 차선 인식 연구는 **2D monocular**(성숙 단계)와 **3D lane detection**(급성장 단계) 두 축으로 정리된다.

- **2D**: 2024년 종합 서베이(arXiv 2411.16316)가 정립한 분류 체계에 따라 ① segmentation 기반, ② anchor 기반(LaneATT, CLRNet, Polar R-CNN), ③ row-wise 분류(UFLD, CondLaneNet), ④ keypoint 기반(GANet), ⑤ parametric/curve 기반(PolyLaneNet, BézierLaneNet)으로 나뉜다. CULane 벤치마크 기준 **CondLSTR(80.36% F1), CLRNet(79.58%), CondLaneNet(78.14%)** 이 상위권을 형성한다.
- **3D**: **PersFormer(2022)** → **Anchor3DLane(2023)** → **LATR(2023)** 로 빠르게 발전했으며 모두 오픈소스로 공개됨. LATR는 OpenLane에서 F1을 크게 끌어올렸다(선행 대비 +11.4).
- **데이터셋**: 2D는 TuSimple / CULane / LLAMAS, 3D는 OpenLane(20만 프레임, 88만+ 차선) / ONCE-3DLanes(21.1만 이미지) / Apollo 3D Synthetic.
- **효율성**: curve 기반 **BézierLaneNet** 은 실시간성(2080 Ti, ResNet18 기준 212 FPS)과 정확도를 동시에 달성해 경량 대안으로 주목.

각 주장의 신뢰도는 §6 검증 표에 명시했다. **시점 상대적 SOTA 주장**과 검증에서 반박된 항목은 §7에 별도 표기했다.

---

## 1. 배경과 조사 범위

차선 인식은 ADAS·자율주행의 핵심 인지 모듈로, 카메라 이미지에서 주행 차선을 검출한다. 딥러닝 도입 이후 문제 정식화(formulation)가 다양하게 분화되었으며, 최근에는 평면 가정을 벗어나 실제 도로의 높이(경사·굴곡)를 복원하는 3D 차선 인식으로 무게중심이 이동하고 있다.

본 보고서는 2021~2026년 발표된 대표 논문과 공식 구현체를 대상으로, 방법론 계열·데이터셋·성능·오픈소스 라이선스를 정리한다.

---

## 2. 2D Monocular 차선 인식 분류 체계

2024년 서베이(arXiv 2411.16316)는 2D monocular 방법을 다음 관점으로 조직화한다: **task paradigm / lane modeling / global context / perspective effect elimination**. 대표 계열은 다음과 같다.

| 계열 | 핵심 아이디어 | 대표 방법 |
|------|--------------|-----------|
| Segmentation 기반 | 픽셀 단위 분류로 차선 마스크 예측 | SCNN, RESA |
| Anchor 기반 | 사전 정의된 line anchor를 회귀·분류 | LaneATT, CLRNet, Polar R-CNN |
| Row-wise 분류 | 행(row)마다 차선 위치를 분류 | UFLD, CondLaneNet |
| Keypoint 기반 | 차선을 keypoint 집합으로 검출 후 결합 | GANet, PINet |
| Parametric/Curve 기반 | 차선을 곡선 파라미터(다항식·Bézier)로 회귀 | PolyLaneNet, LSTR, BézierLaneNet |

**주목할 최신 anchor 방법 — Polar R-CNN(2024)**: local/global polar 좌표계를 도입해 anchor 수를 192 → 20으로 대폭 줄이고 NMS-free 추론을 지원, 효율과 정확도를 동시에 개선했다.

---

## 3. 3D Lane Detection의 부상

평면(BEV 평면) 가정의 한계를 넘어, front-view 이미지에서 3D 공간의 차선 형상을 직접 복원하는 연구가 2022년 이후 빠르게 성장했다.

### 3.1 PersFormer (ECCV 2022 Oral)
- **최초의 Transformer 기반 3D 차선 인식** 방법. 카메라 파라미터를 참조해 front-view local 영역에 attention하여 **BEV feature를 생성하는 Perspective Transformer**와 **통합 2D/3D anchor** 설계를 사용.
- OpenLane 및 Apollo 3D Synthetic에서 baseline을 능가, 2D 성능은 당시 SOTA와 동급.
- **대규모 실세계 3D 데이터셋 OpenLane을 함께 공개**(20만 프레임, 88만+ instance-level 차선, 14개 차선 카테고리).
- 코드: `github.com/OpenDriveLab/PersFormer_3DLane`

### 3.2 Anchor3DLane (CVPR 2023, 확장판 Anchor3DLane++ TPAMI 2024)
- **BEV-free** 방법. 3D 공간에 lane anchor(ray로 파라미터화)를 정의하고 front-view feature에 투영해 **직접 회귀**. 반복적 anchor refinement 및 multi-frame 확장 지원.
- ApolloSim / OpenLane / ONCE-3DLanes 3개 벤치마크에서 당시 BEV 기반 방법을 능가하는 SOTA 달성(※ 발표 시점 기준, 이후 LATR가 능가).
- 코드: `github.com/tusen-ai/Anchor3DLane`

### 3.3 LATR (ICCV 2023 Oral)
- **BEV 변환 없이** 3D-aware front-view feature를 쓰는 **end-to-end Transformer** 방법. lane-aware query generator + 동적 3D ground positional embedding 사용.
- OpenLane에서 선행 대비 **F1 +11.4** 향상, Apollo / OpenLane / ONCE-3DLanes SOTA. (공식 repo 기준 OpenLane-1000에서 F1 ≈ 0.63)
- 코드: `github.com/JMoonr/LATR` (**MIT 라이선스**)

---

## 4. 대표 데이터셋

| 데이터셋 | 차원 | 규모 | 특징 |
|----------|------|------|------|
| **TuSimple** | 2D | 고속도로 중심, 소규모 | 초기 표준 벤치마크 |
| **CULane** | 2D | 대규모, 9개 시나리오 | 야간·혼잡·그림자 등 난이도 다양, F1 주 지표 |
| **LLAMAS** | 2D | 대규모 자동 라벨링 | 고속도로, 리더보드 운영 |
| **OpenLane** | 3D | 20만 프레임, 88만+ 차선, 14 카테고리 | 최초급 대규모 실세계 3D (PersFormer) |
| **ONCE-3DLanes** | 3D | 21.1만 이미지 | ONCE 기반 실세계 3D (SALAD 함께 제안) |
| **Apollo 3D Synthetic** | 3D | 합성 | 3D 방법의 공통 비교 기준 |

> **ONCE-3DLanes 주의**: '21.1만 이미지' 표현은 다소 느슨하며 상당수가 unlabeled 시나리오를 포함한다. 함께 제안된 **SALAD**는 BEV 변환 없이 image view에서 3D 좌표를 회귀하는 extrinsic-free / anchor-free 방법이다.

---

## 5. 성능 비교 (SOTA)

### 5.1 CULane 벤치마크 (2D, F1 %)

| 방법 | 계열 | F1 (%) |
|------|------|--------|
| CondLSTR | row-wise/조건부 | **80.36** |
| CLRNet | anchor | 79.58 |
| CondLaneNet | row-wise | 78.14 |

> 서베이 Table IV 기준. 독립 문헌치(CLRNet DLA34 ≈ 80.47, CondLaneNet R101 ≈ 79.48)와도 정합. **backbone별 config 값**이므로 비교 시 동일 backbone 여부를 반드시 확인할 것.

### 5.2 효율성 — BézierLaneNet (CVPR 2022)
- 차선을 **cubic Bézier 곡선**으로 모델링하는 fully-convolutional curve 기반 방법. segmentation·point 기반의 효율적 대안.
- **실시간 성능(360×640, RTX 2080 Ti)**: ResNet18 **212.83 FPS** (4.10M params), ResNet34 **149.52 FPS** (9.49M params).
- LLAMAS에서 당시 SOTA, TuSimple/CULane에서 우수한 정확도. (LLAMAS SOTA는 2022 시점 기준, 이후 CLRNet 등이 능가)

---

## 6. 오픈소스 코드 및 라이선스

| 프로젝트 | 내용 | 저장소 | 라이선스 |
|----------|------|--------|----------|
| **lanedet** | 2D 통합 툴박스 (SCNN, RESA, UFLD, LaneATT, CondLane) | `github.com/Turoad/lanedet` | repo 공개 (종류 미확정) |
| **mmLaneDet** | 8개 모델 (SCNN·RESA·UFLD·LaneATT·CondLane·GANet·BezierLaneNet·CLRNet) | `github.com/Yzichen/mmLaneDet` | repo 공개 (종류 미확정) |
| **CLRNet** | CLRNet 공식 (CVPR 2022) | `github.com/Turoad/CLRNet` | repo 공개 (종류 미확정) |
| **pytorch-auto-drive** | BézierLaneNet 등 다수 재현·config | `github.com/voldemortX/pytorch-auto-drive` | repo 공개 |
| **PersFormer** | PersFormer 공식 + OpenLane | `github.com/OpenDriveLab/PersFormer_3DLane` | repo 공개 (종류 미확정) |
| **Anchor3DLane** | Anchor3DLane 공식 (CVPR 2023) | `github.com/tusen-ai/Anchor3DLane` | repo 공개 (종류 미확정) |
| **LATR** | LATR 공식 (ICCV 2023 Oral) | `github.com/JMoonr/LATR` | **MIT** |
| Awesome-Lane-Detection | 큐레이션 링크 모음 | `github.com/Core9724/Awesome-Lane-Detection` | — |
| Awesome-3D-Lane-Detection | 3D 전용 큐레이션 | `github.com/JiaweiZhao-git/Awesome-3D-Lane-Detection` | — |

> **라이선스 caveat**: LATR(MIT)를 제외한 대부분은 "공개 오픈소스 저장소 존재"만 확인되었고, 구체적 라이선스 종류(MIT / Apache-2.0 / 비상업용 여부)는 개별 검증되지 않았다. **상업적 활용 전 각 repo의 LICENSE 파일을 반드시 직접 확인**할 것.

---

## 7. 신뢰도 및 검증 결과

### 7.1 검증 통과 핵심 주장 (신뢰도 높음)
아래는 3-표 적대적 검증에서 확정(confirmed)된 항목이다.

- 2D 분류 체계(anchor/row-wise/keypoint/curve) 및 4대 설계 관점 — 확정
- CULane F1 (CondLSTR 80.36 / CLRNet 79.58 / CondLaneNet 78.14) — 3-0
- PersFormer = 최초 Transformer 3D + OpenLane 공개 (20만 프레임/88만+/14 카테고리) — 3-0
- Anchor3DLane = BEV-free, 3개 벤치마크 SOTA(당시) — 확정
- LATR = BEV-free Transformer, OpenLane F1 +11.4 — 3-0
- ONCE-3DLanes 21.1만 이미지 + SALAD extrinsic/anchor-free — 3-0
- BézierLaneNet Bézier 모델링 + 실시간 FPS 수치 — 3-0

### 7.2 반박된(신뢰 금지) 주장
- ❌ "3D 방법이 BEV 기반 vs direct query 기반으로 **이분된다**"는 단순 이분법 → **반박(1-2)**. 실제 지형은 더 복잡함.
- ❌ "BézierLaneNet ResNet34가 CULane 75.30 F1 (normal 91.59, arrows 76.74)"의 **세부 수치** → **반박(1-2)**. 세부 조건별 수치는 신뢰하지 말 것.

### 7.3 일반 주의사항
- **SOTA는 시점 상대적**: Anchor3DLane(2023)·BézierLaneNet(2022)의 SOTA 주장은 발표 당시 기준이며 이후 LATR(2023), Anchor3DLane++(2024) 등이 능가.
- CULane 수치는 backbone에 따라 달라짐 → 공정 비교 시 backbone 통일 필요.

---

## 8. 미해결 질문 (추가 조사 권장)

1. 각 오픈소스 repo(PersFormer, Anchor3DLane, CLRNet, UFLD 등)의 **정확한 라이선스 종류**(상업 이용 가능 여부)는?
2. **2025~2026년 최신 SOTA**(OpenLane·CULane 리더보드 최상위)는 어떤 방법이며 위 수치를 얼마나 능가했는가?
3. TuSimple/CULane/LLAMAS에서 anchor·row-wise·curve 계열 간 **정량적 성능–속도 트레이드오프** 종합 비교표.
4. 3D에서 BEV 기반 vs front-view 기반의 **정확도·연산량 비교** (반박된 이분법 대신 정확한 최신 분류).

---

## 9. 주요 출처

**서베이 / 종합**
- Monocular Lane Detection Based on Deep Learning: A Survey (2024) — arXiv 2411.16316
- arXiv 2404.06860 (3D lane 관련 정리), arXiv 2411.01499

**2D 방법 / 벤치마크**
- BézierLaneNet — arXiv 2203.02431 (CVPR 2022)
- Polar R-CNN (2024)

**3D 방법 / 데이터셋**
- PersFormer + OpenLane — arXiv 2203.11089 (ECCV 2022)
- Anchor3DLane — arXiv 2301.02371 (CVPR 2023)
- LATR — arXiv 2308.04583 (ICCV 2023)
- ONCE-3DLanes + SALAD — arXiv 2205.00301 (CVPR 2022)

**코드 저장소**: §6 표 참조.

---

*본 보고서의 정량 수치는 원 논문/서베이 보고값이며, 실제 재현 성능은 backbone·학습 설정에 따라 달라질 수 있습니다. 상업적 활용 전 각 저장소 라이선스를 직접 확인하시기 바랍니다.*
