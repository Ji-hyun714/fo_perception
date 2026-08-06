# ROI 필터 (사다리꼴 안쪽 유지) — 설계 · 사용법 · 동작 해설

작성일: 2026-07-23
브랜치: `#84-lane-roi-test`

---

## 0. 왜 다시 만드나 (기존 기능의 3대 문제)

1. **polarity 반대**: 기존 `applyRoiMask`는 사다리꼴(중앙)을 **제거**하고 좌/우 밴드를 유지.
   → 원하는 건 사다리꼴 **안쪽 유지 / 바깥 제거**.
2. **좌표계 다름**: 기존은 **BEV(640×640 탑뷰)** 이진영상에 적용.
   → 원하는 건 **원본 카메라 영상(원근)** 기준.
3. **죽은 경로**: `applyRoiMask`는 `Detect()`(카메라 파이프라인)에서만 호출.
   실제 노드는 `DetectFromMask()`(YOLOPv2 마스크) 사용 → 기존 ROI 파라미터 대부분 **무효과**.

→ 기존 `applyRoiMask`는 **건드리지 않고**(BEV 노이즈 제거 용도로 남겨둠),
   원본 영상 기준 **새 ROI 크롭 단계**를 노드에 추가한다.

## 1. 목표 (확정된 요구사항)

- **원본 bag 카메라 영상**에 **사다리꼴(임의 사각형) ROI**를 지정 → **안쪽만 유지, 바깥 제거**.
- 적용 단계 **둘 다 선택 가능** (파라미터로 on/off):
  - **PRE** : `engine_.Process(frame)` **추론 입력 전**에 원본 프레임 크롭.
  - **POST**: YOLOPv2 결과 `lane_mask`에 크롭(마스크 좌표계로 스케일).
- 꼭짓점 지정: **원본 화면에서 꼭짓점 4개를 마우스로 직접 드래그.**

## 2. 데이터 모델

```cpp
struct RoiQuad {
  cv::Point2f pts[4];   // TL, TR, BR, BL (원본 이미지 픽셀 좌표)
  bool  enabled_pre  = false;
  bool  enabled_post = false;
};
```
- 기본값: 이미지 크기 기준 사다리꼴(상단 좁게/하단 넓게) 자동 초기화.
- 좌표는 **원본 카메라 해상도** 기준. POST 적용 시 `lane_mask` 크기에 맞춰 스케일.

## 3. 구현 위치 & 흐름

```
callbackImage(frame)
  ├─ [PRE]  enabled_pre  → cropToQuad(frame, quad_px)      ← 원본 해상도
  ├─ engine_.Process(frame, infer_result)                   (추론)
  ├─ [POST] enabled_post → cropToQuad(lane_mask, quad_scaled) ← mask 해상도로 스케일
  ├─ DetectFromMask(lane_mask, frame)
  └─ [debug] ROI 편집 창: 원본 frame + 사다리꼴 + 꼭짓점 핸들 표시
```

- `cropToQuad(img, quad)`: `fillConvexPoly`로 마스크 생성 → 안쪽만 유지, 바깥 0.
- PRE/POST 각각 독립 on/off.

## 4. UI (원본 화면 편집)

- 전용 창 `"ROI Quad Editor"` — **원본 카메라 프레임**을 배경으로 표시(입력 없으면 회색 캔버스 + 안내).
- 사다리꼴 폴리라인 + 꼭짓점 4개를 원형 핸들로 표시.
- **마우스**: 클릭 지점에서 가장 가까운 꼭짓점을 잡아 드래그. (좌버튼)
- **키**: `s`=저장 / `l`=불러오기 / `p`=PRE 토글 / `o`=POST 토글 / `r`=기본값 리셋.
- 창 표시 배율: 큰 원본 해상도 대비 축소 표시 필요 시 `debug_window_scale` 유사 스케일 처리
  (마우스 좌표 → 원본 좌표 역변환 주의).
- **범례**: 영문/약어 (한글은 freetype 헤더 부재로 보류 — 문서로 대체).

## 5. 파라미터 / 영속성

- ROS 파라미터:
  - `roi_quad_enable_pre` (bool), `roi_quad_enable_post` (bool)
  - `roi_quad_pts` (int array[8] = x0,y0,...,x3,y3) — 숫자 직접 지정용
  - `roi_quad_preset_path` (string)
- 저장/불러오기: `cv::FileStorage`로 8좌표 + 두 enable 플래그 R/W (`s`/`l` 키).

## 6. 변경 파일 (예정)

| 파일 | 변경 |
|---|---|
| `yolopv2_cam2_node.cpp` | RoiQuad 상태, cropToQuad(), PRE/POST 적용, Quad Editor 창·마우스·키, 파라미터 |
| (선택) `helper_lane.hpp` 등 | cropToQuad 유틸을 공용화할 경우 |

- **기존 `applyRoiMask` / 사다리꼴 컨트롤 창은 그대로 유지**하거나, 혼동되면 별도 플래그로 숨김.
  (이번 재설계는 원본-크롭이 메인, 기존 BEV 중앙배제는 보조)

## 7. 확인 필요 / 리스크

- **기존 사다리꼴 컨트롤 창**(BEV 중앙배제)을 **남길지 / 제거할지**? (혼동 방지)
- POST 크롭 시 lane_mask 좌표계·해상도(현재 640×360 리사이즈) 매핑 정확도.
- 원본 해상도가 커서 편집 창이 화면을 넘으면 축소 표시 + 좌표 역변환 필요.
- 한글 범례는 환경 제약으로 보류 (freetype dev 헤더/​ttf 설치 시 재검토).

## 8. 다음 행동
계획 확정 시 구현 착수 → `colcon build lane_detection` → bag으로 PRE/POST 각각 검증.
