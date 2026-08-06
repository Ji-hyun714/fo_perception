# [디버깅] 레이어 토글 OFF→ON 즉시 OFF 복귀 이슈

작성일: 2026-07-24
대상: `src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp`
관련: [03_설계.md](03_설계.md)(시각화 구조), [01_사용법.md](01_사용법.md)(Control Panel)

## 1. 증상

- Control Panel 에서 레이어 버튼을 **OFF→ON 으로 누르면 잠깐 켜졌다가 즉시 off 로 되돌아감.**
- 창이 프레임 넘길 때만 순간적으로 보이고 유지되지 않음("프레임 단위 출력만").
- 대상 레이어: `syncClosedWindows` 가 감시하는 팝업창들 — ROI Editor / Zoom / BEV / Process / Info / Structure.

## 2. 재현 경로

1. BAG 뷰어 모드 실행(`bag_path` 지정), `debug_mode:=true`.
2. 정지 상태에서 Control Panel 의 `BEV`(또는 `Zoom`/`Info` 등) 버튼 클릭.
3. 해당 창이 떴다가 한두 프레임 만에 사라지고 버튼이 off 로 복귀.

## 3. 가설 (우선순위순)

| # | 가설 | 근거 | 판정 |
|---|---|---|---|
| H1 | `syncClosedWindows` 가 **갓 생성돼 아직 WM 매핑 전인 창**을 `WND_PROP_VISIBLE < 1.0` 으로 읽어 "사용자가 닫음"으로 오판 → 즉시 토글 off | 코드 트레이스(§4). `!open_windows_.count()` 가드는 첫 사이클만 보호, "한 번도 보인 적 없는 창"은 미보호 | ★ 유력 |
| H2 | sync 가 `needs_redraw_` 게이트보다 **먼저 매 사이클 실행**되어 유휴 중에도 오판 기회가 매 프레임 발생 | [:1025](../../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L1025) sync → [:1026](../../src/ws_cam2/src/lane_detection/src/yolopv2_cam2_node.cpp#L1026) 게이트 순서 | H1의 증폭 요인 |
| H3 | `getWindowProperty` 백엔드(GTK/Qt) 별 VISIBLE 반환값 차이 — 최소화/포커스 손실 시에도 <1 | 백엔드 의존 | 확인 필요 |
| H4 | 버튼 히트박스/토글 자체 오작동(클릭이 두 번 먹힘) | `onPanelMouse` 는 `break` 로 1회만 토글, 반증됨 | ✗ 기각 |

## 4. 코드 트레이스 (H1 확정 논리)

클릭 → 즉시 OFF 까지의 사이클:

```
[클릭]  onPanelMouse: flag=true, needs_redraw_=true, panel_dirty_=true   (:796)
[K+1]   renderAllViews:
          syncClosedWindows()  → 창이 아직 open_windows_ 에 없음 → skip   (:1004 가드)
          showLayer(name,...)  → namedWindow + imshow + open_windows_.insert  (:859-870)
        waitKeyEx(30)          → GUI 1회 펌프 (WM 매핑 시작, 완료 보장 X)
[K+2]   renderAllViews:
          syncClosedWindows()  → 창이 open_windows_ 에 있음
                                 getWindowProperty(VISIBLE) 가 아직 <1.0 이면
                                 → flag=false, destroyWindow, panel_dirty_=true   (:1008-1012)
        ⇒ 레이어 즉시 OFF, 버튼 off 복귀
```

핵심: **"방금 만들어 아직 화면에 realize 안 된 창"과 "사용자가 X 로 닫은 창"을 구별하지 못한다.** 둘 다 `VISIBLE < 1.0` 로 보인다.

## 5. 검증 방법

### (a) 계측 로그 (이번에 반영) — 실기 확인용
`syncClosedWindows` 가 창을 강제 종료하기 직전에 창 이름 + VISIBLE 실제값 + open 이후 경과 사이클을 로그로 남긴다.
- 클릭 직후 몇 사이클 내에 `[sync] force-close ... visible=-1/0` 로그가 뜨면 → **H1 확정**.
- 실제 X 클릭으로 닫았을 때만 로그가 뜨면 → H1 반증(정상 동작).

실행:
```bash
ros2 run lane_detection yolopv2_cam2_node --ros-args \
  -p bag_path:=/home/wise/fo_perception/7_to_hoamji -p debug_mode:=true
# BEV 버튼 클릭 → 콘솔의 [sync] force-close 로그 관찰
```

### (b) 독립 프로브 (이번에 실행, 결정적 증거) ✅ 완료
노드의 `showLayer → waitKey(30) → getWindowProperty` 시퀀스를 그대로 재현하는 최소 OpenCV 프로그램을 컴파일·실행.

```
[K+1] created+imshow, VISIBLE(pump 전) = -1.0
[K+2] after waitKey(30), VISIBLE = -1.0  <-- sync 가 '닫힘'으로 오판!
...  (K+9 까지 전부 -1.0)
```

**결과 해석:** 이 GTK OpenCV 백엔드는 `WND_PROP_VISIBLE` 을 지원하지 않고 **항상 -1.0** 을 반환한다.
→ `syncClosedWindows` 의 `< 1.0` 판정이 **모든 레이어 창에 대해 항상 참** → 생성 다음 사이클마다 무조건 force-close.
→ 프레임 갱신 시 `needs_redraw_` 로 창이 재생성됐다가 다음 사이클에 또 죽어 **"프레임 단위 깜빡"** 증상 재현.

> 단서: -1.0 지속은 검증 환경의 헤드리스 `:1` 디스플레이 특성이 일부 반영됐을 수 있다. 정상 WM 에선 realize 후 1.0 을 반환할 수도 있다(그 경우 §3 H1 의 "realize 지연" 형태). **어느 쪽이든 force-close 오판이 발생한다는 사실은 확정**이며 수정안이 두 경우를 모두 커버.

## 6. 수정 방안 (적용됨)

**VISIBLE 속성이 실제로 신뢰 가능한 백엔드에서, 한 번이라도 VISIBLE≥1 로 확인된 창만** close 대상으로 삼는다. (capability 프로브 + realize 유예 결합)

멤버 추가:
```cpp
bool visible_prop_supported_{false};      // VISIBLE 속성 신뢰 가능 백엔드인지
std::set<std::string> confirmed_visible_; // 한 번이라도 VISIBLE≥1 로 본 창
```

`syncClosedWindows` 로직:
- 열린 창의 VISIBLE≥1 → `visible_prop_supported_=true`, `confirmed_visible_` 등록, 계속.
- VISIBLE<1 이지만 **(a) 속성 미지원(전부 -1인 백엔드) 또는 (b) 아직 confirmed 안 된 갓 생성 창** → **건드리지 않음**(오판 방지).
- VISIBLE<1 && 속성 지원됨 && 예전에 confirmed → **진짜 사용자 X 종료** → 토글 off + 두 set 에서 제거.

효과:
- 이 백엔드(항상 -1): 어떤 창도 confirmed 안 됨 → force-close 안 함 → **레이어 유지**(버그 해소). X 종료 자동감지는 이 백엔드 한정 비활성(버튼으로 토글).
- VISIBLE 지원 백엔드: realize 후 confirmed → X 종료 감지 정상 작동, 갓 생성 창은 유예.

## 7. 체크리스트

- [x] 독립 프로브로 force-close 발화(VISIBLE<1) 재현 → H1/H2 확정
- [x] capability 프로브 + confirmed_visible_ 가드 적용
- [x] 빌드 성공
- [ ] 실기(실제 디스플레이)에서 OFF→ON 유지 확인 / X 로 닫으면 off 되는지 회귀 확인
