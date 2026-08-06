# 사다리꼴 ROI 사용자 조정 기능 — 상세 설명 (소스 포함)

작성일: 2026-07-23
브랜치: `#84-lane-roi-test`
대상 패키지: `lane_detection`

---

## 1. 개요

사다리꼴 ROI 마스크(`applyRoiMask`)의 범위를 **실행 중 사용자가 직접 조정**하고,
그 범위를 **가상의 경계선으로 실시간 시각화**하는 기능이다.

조정 경로는 3가지이며 모두 동일한 detector setter를 호출하고 서로 동기화된다.
1. **트랙바** — 디버그 창의 슬라이더
2. **마우스 드래그** — 컨트롤 창 위 직접 드래그
3. **ROS 파라미터(숫자 입력)** — `ros2 param set` / rqt_reconfigure

조정 대상 파라미터 4종:

| 변수 | 의미 | 기본값 |
|---|---|---|
| `roi_mask_enabled` | ROI 마스크 적용 on/off | true |
| `roi_mid_offset_px` | 중심 오프셋 (px, +오른쪽/−왼쪽) | 0 |
| `roi_half_top_ratio` | 상단(원거리) 반폭 비율 (0~0.5) | 0.45 |
| `roi_half_bot_ratio` | 하단(근거리) 반폭 비율 (0~0.5) | 0.35 |

---

## 2. ROI 마스크의 기하 (기존 로직)

`applyRoiMask()` — [LaneDetection.cpp:5285](../src/ws_cam2/src/lane_detection/src/LaneDetection.cpp#L5285)

BEV 이진영상(`bev_size`, 기본 640×640) 좌표계에서 각 행 `y`마다 좌/우 밴드만 남긴다.

```
t          = y / h                              // 0(상단/원거리) → 1(하단/근거리)
half_ratio = top_ratio + (bot_ratio-top_ratio)*t
half_width = w * half_ratio
mid_img    = w/2 + roi_mid_offset_px
left_end     = max(0,   mid_img - half_width)   // 좌측 밴드 끝
right_start  = min(w-1, mid_img + half_width)   // 우측 밴드 시작
```

`[0, left_end]` 와 `[right_start, w-1]` 만 유지, 중앙은 제거 → **사다리꼴** 모양.
상단이 넓고(top_ratio 큼) 하단이 좁은(bot_ratio 작음) 형태가 기본.

→ 사용자에게 보여줄 "가상의 선"은 결국 **좌/우 내측 경계(사다리꼴 빗변)** 와 **중앙선**이다.

---

## 3. 구현 구조

```
┌─────────────────────── Yolopv2Cam2Node (노드) ───────────────────────┐
│  setupRoiTuning()  : 파라미터 선언 + 창/트랙바/마우스/타이머 셋업       │
│                                                                        │
│  [입력 3경로] ──────────────┐                                          │
│   트랙바  onRoiTrackbar ────┤                                          │
│   마우스  onRoiMouse    ────┼──► LaneLineDetector Set*()  (값 갱신)     │
│   파라미터 onSetRoiParameters┘         │                                │
│                                        ▼                                │
│  syncRoiTrackbars() / syncRoiParameters()  ← 3경로 상호 동기화          │
│                                                                        │
│  onRoiUiTimer() (30ms) ──► renderRoiMaskOverlay() ──► imshow + waitKey  │
└────────────────────────────────────────────────────────────────────────┘
                                 │
              LaneLineDetector.renderRoiMaskOverlay()  (시각화 캔버스 생성)
```

---

## 4. Detector 측 구현 (LaneDetection)

### 4-1. 런타임 조정 API — [LaneDetection.hpp:551~564](../src/ws_cam2/src/lane_detection/include/lane_detection/LaneDetection.hpp#L551)

```cpp
/* ---- 사다리꼴 ROI 마스크 런타임 조정 (사용자 변경 가능) ---- */
cv::Size GetBevSize() const { return config_.bev_size; }
int   GetRoiMidOffsetPx()   const { return config_.roi_mid_offset_px; }
float GetRoiHalfTopRatio()  const { return config_.roi_half_top_ratio; }
float GetRoiHalfBotRatio()  const { return config_.roi_half_bot_ratio; }
bool  GetRoiMaskEnabled()   const { return config_.roi_mask_enabled; }

void SetRoiMidOffsetPx(int px)   { config_.roi_mid_offset_px = px; }
void SetRoiHalfTopRatio(float r) { config_.roi_half_top_ratio = std::clamp(r, 0.0f, 0.5f); }
void SetRoiHalfBotRatio(float r) { config_.roi_half_bot_ratio = std::clamp(r, 0.0f, 0.5f); }
void SetRoiMaskEnabled(bool e)   { config_.roi_mask_enabled = e; }

cv::Mat renderRoiMaskOverlay() const;
```

- 기존엔 YAML 로드 후 변경 불가였던 `config_` 값을 **런타임에 수정**할 수 있게 setter 추가.
- 비율 setter는 `std::clamp(0~0.5)`로 안전 범위를 강제(사다리꼴이 화면을 넘지 않게).
- `<algorithm>` 헤더를 추가([LaneDetection.hpp:2](../src/ws_cam2/src/lane_detection/include/lane_detection/LaneDetection.hpp#L2)).

### 4-2. 시각화 캔버스 — `renderRoiMaskOverlay()` [LaneDetection.cpp:5337](../src/ws_cam2/src/lane_detection/src/LaneDetection.cpp#L5337)

`applyRoiMask`와 **동일한 half_ratio/half_width 식**으로 `bev_size` 캔버스에 그린다.

```cpp
cv::Mat LaneLineDetector::renderRoiMaskOverlay() const
{
    const int w = config_.bev_size.width;
    const int h = config_.bev_size.height;
    cv::Mat vis = cv::Mat::zeros(h, w, CV_8UC3);

    const int mid_img = w / 2 + config_.roi_mid_offset_px;
    const float top_ratio = config_.roi_half_top_ratio;
    const float bot_ratio = config_.roi_half_bot_ratio;

    std::vector<cv::Point> left_inner, right_inner;
    for (int y = 0; y < h; ++y) {
        const float t = static_cast<float>(y) / static_cast<float>(h);
        const float half_ratio = top_ratio + (bot_ratio - top_ratio) * t;
        const int half_width = static_cast<int>(w * half_ratio);
        const int left_end = std::max(0, mid_img - half_width);
        const int right_start = std::min(w - 1, mid_img + half_width);

        if (config_.roi_mask_enabled) {   // 유지 영역(좌/우 밴드) 회색 음영
            cv::line(vis, {0, y}, {left_end, y}, cv::Scalar(60,60,60), 1);
            cv::line(vis, {right_start, y}, {w-1, y}, cv::Scalar(60,60,60), 1);
        }
        left_inner.emplace_back(left_end, y);     // 사다리꼴 좌측 빗변
        right_inner.emplace_back(right_start, y); // 사다리꼴 우측 빗변
    }

    // 경계선(가상의 선): 활성=녹색 / 비활성=주황, 중앙선=노랑
    const cv::Scalar edge = config_.roi_mask_enabled ? cv::Scalar(0,255,0)
                                                     : cv::Scalar(0,165,255);
    // left_inner / right_inner 를 polylines 로, mid_img 세로선 그리기
    // + 파라미터 텍스트, 조작 안내 텍스트
    return vis;
}
```

시각화 요소 (모두 우상단 **범례(legend) 박스**에 색·의미가 정리되어 표시됨 — `drawRoiLegend()`):
- **배경(하늘색 픽셀)**: 마스킹 직전 BEV 이진영상(전체 차선 픽셀) — `last_roi_bev_bin_` 스냅샷
- **회색 밴드**: 실제로 검사에 사용되는 좌/우 밴드(=ROI 유지 영역, 반투명)
- **녹색/주황 곡선**: 사다리꼴 내측 경계(빗변) — 사용자가 조정하는 "가상의 선"
- **노란 세로선**: 중심(`mid_img`)
- **상단 텍스트**: `enabled / mid / top / bot` 현재값
- **범례 하단**: `[s]ave / [l]oad`, `L-drag=half / R-drag=center` 조작 안내

> ⚠️ **주의**: 이 함수의 `half_ratio`·`half_width` 식은 `applyRoiMask()`를 복제한 것이다.
> 마스크 로직을 바꾸면 **양쪽을 반드시 함께 수정**해야 시각화와 실제 절단이 일치한다.

---

## 5. 노드 측 구현 (yolopv2_cam2_node)

### 5-1. 셋업 — `setupRoiTuning()` [yolopv2_cam2_node.cpp:493](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L493)

생성자 끝에서 호출([:177](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L177)). 하는 일:

1. **ROS 파라미터 선언**(4종) + `add_on_set_parameters_callback` 등록 → 숫자 입력 경로.
   - `debug_mode:=false` 여도 이 부분은 동작하므로, 창 없이 파라미터만으로도 조정 가능.
2. `debug_mode`일 때만:
   - `namedWindow("ROI Mask Control")`
   - **트랙바 4종** 생성 ([:518~521](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L518))
   - `setMouseCallback` 등록 ([:523](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L523))
   - **UI 타이머(30ms)** 생성 ([:527](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L527))

```cpp
// 트랙바: OpenCV는 0~max 정수만 → 오프셋은 +w/2 시프트, 비율은 ×100 정수
cv::createTrackbar("enabled",        kRoiControlWindowName, &tb_enabled_, 1,          onRoiTrackbar, this);
cv::createTrackbar("mid_offset+w/2", kRoiControlWindowName, &tb_mid_,     w,          onRoiTrackbar, this);
cv::createTrackbar("half_top x100",  kRoiControlWindowName, &tb_top_,     kRatioTbMax /*50*/, onRoiTrackbar, this);
cv::createTrackbar("half_bot x100",  kRoiControlWindowName, &tb_bot_,     kRatioTbMax,        onRoiTrackbar, this);
cv::setMouseCallback(kRoiControlWindowName, onRoiMouse, this);
```

트랙바 표현 규칙:
- `mid_offset+w/2`: 실제 오프셋 = `트랙바값 − w/2` (0=최좌, w/2=중앙, w=최우)
- `half_* x100`: 실제 비율 = `트랙바값 / 100` (0~50 → 0.00~0.50)

### 5-2. UI 타이머 — `onRoiUiTimer()` [yolopv2_cam2_node.cpp:531](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L531)

```cpp
void onRoiUiTimer()
{
    if (!debug_mode_ || !lld_) return;
    const cv::Mat overlay = lld_->renderRoiMaskOverlay();
    if (!overlay.empty()) cv::imshow(kRoiControlWindowName, overlay);
    cv::waitKey(1);  // GUI 이벤트 펌프 (트랙바/마우스 콜백 동작)
}
```

**핵심 역할**: 원래 `imshow`/`waitKey`는 이미지 콜백 안에서만 불렸다.
→ 카메라/bag 입력이 없으면 콜백이 안 돌아 **창이 안 뜨는** 문제가 있었다.
이 타이머가 프레임과 무관하게 30ms마다 오버레이를 그리고 `waitKey`로 GUI를 펌프하므로,
**입력이 없어도 컨트롤 창이 뜨고 트랙바/마우스가 즉시 반응**한다.

### 5-3. 트랙바 콜백 — `onRoiTrackbar` / `applyRoiFromTrackbars`
[yolopv2_cam2_node.cpp:583](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L583), [:544](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L544)

```cpp
static void onRoiTrackbar(int, void * userdata)
{
    auto * self = static_cast<Yolopv2Cam2Node *>(userdata);
    if (self->roi_updating_) return;  // 프로그램적 setTrackbarPos 재귀 방지
    self->applyRoiFromTrackbars();
}

void applyRoiFromTrackbars()
{
    lld_->SetRoiMaskEnabled(tb_enabled_ != 0);
    lld_->SetRoiMidOffsetPx(tb_mid_ - roi_bev_w_ / 2);
    lld_->SetRoiHalfTopRatio(static_cast<float>(tb_top_) / 100.0f);
    lld_->SetRoiHalfBotRatio(static_cast<float>(tb_bot_) / 100.0f);
    syncRoiParameters();   // 파라미터 서버에도 반영
}
```

### 5-4. 마우스 콜백 — `onRoiMouse` [yolopv2_cam2_node.cpp:593](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L593)

컨트롤 창은 **1:1 BEV 캔버스**이므로 마우스 좌표 `(x,y)`가 곧 BEV 좌표다.

```cpp
static void onRoiMouse(int event, int x, int y, int flags, void * userdata)
{
    auto * self = static_cast<Yolopv2Cam2Node *>(userdata);
    const int w = self->roi_bev_w_;
    const int mid = w / 2 + self->lld_->GetRoiMidOffsetPx();

    const bool ldrag = (event == cv::EVENT_LBUTTONDOWN) ||
      (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON));
    const bool rdrag = (event == cv::EVENT_RBUTTONDOWN) ||
      (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_RBUTTON));

    if (ldrag) {  // 좌드래그: 반폭 조절
        const float half_ratio = std::clamp(std::abs(x - mid)/static_cast<float>(w), 0.0f, 0.5f);
        const int h = self->lld_->GetBevSize().height;
        if (y < h/2) self->lld_->SetRoiHalfTopRatio(half_ratio);  // 위쪽 = 원거리(top)
        else         self->lld_->SetRoiHalfBotRatio(half_ratio);  // 아래쪽 = 근거리(bot)
    } else if (rdrag) {  // 우드래그: 중심 이동
        self->lld_->SetRoiMidOffsetPx(std::clamp(x - w/2, -w/2, w/2));
    } else return;

    self->syncRoiTrackbars();    // 트랙바 위치 갱신
    self->syncRoiParameters();   // 파라미터 서버 갱신
}
```

- **좌버튼 드래그**: 중심에서의 수평 거리 `|x−mid|/w`가 반폭 비율.
  마우스 y가 화면 위쪽(<h/2)이면 `top_ratio`, 아래쪽이면 `bot_ratio`를 조절.
- **우버튼 드래그**: `x−w/2`를 중심 오프셋으로.

### 5-5. 숫자 입력(ROS 파라미터) — `onSetRoiParameters` [yolopv2_cam2_node.cpp:624](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L624)

```cpp
rcl_interfaces::msg::SetParametersResult onSetRoiParameters(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (roi_updating_ || !lld_) return result;   // 내부 동기화 콜백은 무시
    bool changed = false;
    for (const auto & p : params) {
        if      (p.get_name() == "roi_mask_enabled")   { lld_->SetRoiMaskEnabled(p.as_bool());              changed = true; }
        else if (p.get_name() == "roi_mid_offset_px")  { lld_->SetRoiMidOffsetPx((int)p.as_int());          changed = true; }
        else if (p.get_name() == "roi_half_top_ratio") { lld_->SetRoiHalfTopRatio((float)p.as_double());    changed = true; }
        else if (p.get_name() == "roi_half_bot_ratio") { lld_->SetRoiHalfBotRatio((float)p.as_double());    changed = true; }
    }
    if (changed) syncRoiTrackbars();   // 트랙바 위치도 맞춤
    return result;
}
```

### 5-6. 동기화 & 재귀 방지

세 경로가 서로를 갱신하므로, `setTrackbarPos`/`set_parameters` 호출이 다시 콜백을
발화시키는 무한루프를 막아야 한다. → `roi_updating_` 플래그로 차단.

- `syncRoiTrackbars()` [:554](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L554): detector 값 → 트랙바 위치
- `syncRoiParameters()` [:571](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L571): detector 값 → 파라미터 서버
- 두 함수 모두 시작에 `roi_updating_=true`, 끝에 `false`.
- `onRoiTrackbar` / `onSetRoiParameters`는 `roi_updating_`이면 즉시 반환.

### 5-7. 관련 멤버 변수

| 멤버 | 의미 |
|---|---|
| `tb_enabled_ / tb_mid_ / tb_top_ / tb_bot_` | 트랙바 백킹 정수값 |
| `roi_bev_w_` | BEV 폭 캐시 (마우스/트랙바 좌표 변환용) |
| `roi_updating_` | 동기화 재귀 방지 플래그 |
| `roi_param_cb_handle_` | 파라미터 콜백 핸들 (수명 유지) |
| `roi_ui_timer_` | 30ms UI 렌더/펌프 타이머 |

---

## 6. 사용법

### 실행 (창 + 트랙바 + 마우스)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lane_detection yolopv2_cam2_node --ros-args -p debug_mode:=true
```
- `ROI Mask Control` 창이 뜬다(입력 프레임 없어도 표시됨).
- 트랙바 조절 / 좌드래그(반폭)·우드래그(중심)로 사다리꼴 변경 → 경계선 실시간 갱신.
- `Lane Detection Process`(모자이크) 창은 실제 이미지 입력 시 표시.

### 숫자로 직접 지정 (런타임)
```bash
ros2 param set /yolopv2_cam2_node roi_mask_enabled true
ros2 param set /yolopv2_cam2_node roi_mid_offset_px 20
ros2 param set /yolopv2_cam2_node roi_half_top_ratio 0.40
ros2 param set /yolopv2_cam2_node roi_half_bot_ratio 0.30
ros2 param get /yolopv2_cam2_node roi_half_top_ratio   # 현재값 확인
# rqt_reconfigure 로 GUI 슬라이더 사용도 가능
```

### 창 없이 파라미터만
```bash
ros2 run lane_detection yolopv2_cam2_node --ros-args -p debug_mode:=false
# 이후 ros2 param set 으로 조정
```

---

## 7. 변경 파일 요약

| 파일 | 변경 |
|---|---|
| [LaneDetection.hpp](../src/ws_cam2/src/lane_detection/include/lane_detection/LaneDetection.hpp) | `<algorithm>`, ROI getter/setter 9종, `renderRoiMaskOverlay()` 선언 |
| [LaneDetection.cpp](../src/ws_cam2/src/lane_detection/src/LaneDetection.cpp) | `renderRoiMaskOverlay()` 구현 (`applyRoiMask` 기하 복제) |
| [yolopv2_cam2_node.cpp](../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp) | `<chrono>`, `<rcl_interfaces/.../set_parameters_result.hpp>`, 컨트롤 창·트랙바·마우스·파라미터 콜백·UI 타이머, 콜백 내 오버레이 표시, 소멸자 창 정리 |

---

## 8. 주의사항 / 한계

- `renderRoiMaskOverlay`의 기하식은 `applyRoiMask`의 **복제본** → 마스크 로직 변경 시 동기화 필요.
- 조정 값은 **런타임 한정**이며 YAML로 영구 저장되지 않는다(재시작 시 YAML 기본값 복귀).
- OpenCV HighGUI는 스레드 안전하지 않으나, 단일 스레드 executor(`rclcpp::spin`)에서
  타이머·구독 콜백이 같은 스레드로 실행되므로 안전.
- 실행 시 뜨는 `canberra-gtk-module` / `Using 'value' pointer ... deprecated` 경고는
  동작에 영향 없는 관례적 경고.

## 9. BEV 배경 오버레이 · 범례 · YAML 저장/불러오기 (구현 완료)

### 9-1. BEV 이진영상 배경
- `applyRoiMask()` 진입 시 마스킹 **직전** 이진영상을 `last_roi_bev_bin_`에 `clone()` 저장
  ([LaneDetection.cpp](../../src/ws_cam2/src/lane_detection/src/LaneDetection.cpp), `applyRoiMask` 상단).
- `renderRoiMaskOverlay()`가 이 스냅샷을 하늘색(B강·G중·R0) 톤으로 배경에 깔아,
  차선 픽셀 분포 대비 사다리꼴이 어디를 자르는지 한눈에 보이게 함.
- disable 상태에서도 스냅샷은 갱신되므로 배경은 항상 최신.

### 9-2. 범례(legend) — `drawRoiLegend()`
- 우상단 반투명 박스에 모든 표시 요소(색 스와치 + 설명)를 정리.
- §4-2의 "시각화 요소" 목록이 그대로 범례 항목이 됨.

### 9-3. YAML 저장/불러오기 (키 입력)
- 컨트롤 창 포커스 상태에서 **`s`=저장, `l`=불러오기**.
- 경로: ROS 파라미터 `roi_preset_path` (기본 `.../lane_detection/yaml/roi_mask_preset.yaml`).
- `saveRoiPreset()` / `loadRoiPreset()` — `cv::FileStorage`로 4개 값 R/W.
  불러오기 후 `syncRoiTrackbars()` + `syncRoiParameters()`로 트랙바·파라미터 동기화.
- 키 처리는 `handleRoiKey()`가 담당하며, UI 타이머와 이미지 콜백의 `waitKey` 양쪽에서 호출.

```bash
# 경로 바꿔서 프리셋 여러 개 관리도 가능
ros2 run lane_detection yolopv2_cam2_node --ros-args \
  -p debug_mode:=true -p roi_preset_path:=/home/wise/my_roi.yaml
# 창에서 s(저장) / l(불러오기)
```

> 저장 형식 예시 (`roi_mask_preset.yaml`):
> ```yaml
> %YAML:1.0
> roi_mask_enabled: 1
> roi_mid_offset_px: 20
> roi_half_top_ratio: 4.0000000000000002e-01
> roi_half_bot_ratio: 3.0000000000000004e-01
> ```

## 10. 향후 확장 (선택)
- 프리셋 저장 시 타임스탬프 히스토리 남기기.
- 범례 on/off 토글 키.
