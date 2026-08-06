# ROI_lane — 하단 ROI 차선 인식 사용법

하단 세로 ROI(`roi_top_ratio`)로 차선 인식을 실행/검증하는 명령어 모음.

## 폴더 구성
- `usage.md` — 본 문서 (명령어·사용법)
- `references.md` — ROI 개선 사례 5건 한글 요약
- `roi_dev_plan.md` — 개발 기획서 (설계·A/B·지표·S1~S5, §7 실측결과)
- `root_cause_and_improvement.md` — **하단 ROI 무효과 원인 분석 + 개선 방안 (2026-07-10)**
- `changelog.md` — 작업/구현 이력
- `analyze_roi_sweep.py` — A/B 스윕 분석 스크립트
- `figs/` — 지표 차트·동작 스크린샷 / `data/` — dump CSV(11 pass)

## 사전 준비 (매번 새 터미널)
```bash
cd /home/wise/fo_perception
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 빌드
```bash
cd /home/wise/fo_perception
colcon build --packages-select lane_detection --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## roi_top_ratio 파라미터
- `lane_mask` 상단에서 잘라낼 비율. 하단만 ROI로 사용.
- `-1`(기본): YAML 설정값 사용 (기존 동작, ROI 컷 없음)
- `0.0`: 컷 없음(전체 사용)
- `0.5`: 상단 절반 제거 → **하단 절반만 ROI**
- `0.40 ~ 0.55`: 스윕 권장 범위 (원거리 손실 vs 노이즈 제거 트레이드오프)

## 전체 런치 인자 (cam2_yolopv2.launch.py)
| 인자 | 기본값 | 의미 |
|---|---|---|
| `bag_path` | "" | 값이 있으면 **뷰어 모드**(bag 직접 읽기), 비어있으면 **라이브 구독 모드** |
| `bag_topic` | /camera/image_rect | 뷰어가 bag에서 읽을 이미지 토픽 |
| `debug_mode` | true | 결과 시각화 창 on/off. **뷰어 모드에서는 항상 창이 뜨므로 사실상 무관** |
| `debug_window_scale` | 1.0 | 결과창 크기 배율 (예 1.5 = 1.5배 확대) |
| `roi_top_ratio` | -1.0 | 하단 ROI 상단 컷 비율 (-1=YAML값, 0.5=하단 절반만) |
| `input_topic` | /camera/image_rect | 라이브 구독 모드에서 구독할 토픽 |
| `yaml_path`, `ld_cfg` | "" | 카메라 캘리브 / LaneConfig YAML 경로 |

> ⚠️ **문법 주의:** ROS2 런치 인자는 반드시 `이름:=값` (콜론+등호). `debug_window_scale:1.5`처럼
> `:`만 쓰면 인자로 인식되지 않고 **조용히 무시**되어 기본값 1.0으로 실행됩니다. → `debug_window_scale:=1.5`

### 두 명령의 차이
```bash
# (A) 질문에 나온 명령 — debug_window_scale:1.5 는 오타라 무시됨 → 배율 1.0
ros2 launch lane_detection cam2_yolopv2.launch.py \
  debug_mode:=true bag_path:=/home/wise/Desktop/7_to_hoamji debug_window_scale:1.5

# (B) 아래 readme 뷰어 모드 명령 — roi_top_ratio:=0.5 로 하단 ROI 적용
ros2 launch lane_detection cam2_yolopv2.launch.py \
  bag_path:=/home/wise/Desktop/7_to_hoamji roi_top_ratio:=0.5
```
| 항목 | (A) 질문 명령 | (B) readme 뷰어 명령 |
|---|---|---|
| `bag_path` | 지정됨 → 뷰어 모드 | 지정됨 → 뷰어 모드 (동일) |
| `debug_mode:=true` | 명시 (뷰어 모드에선 어차피 창 뜸 → 효과 없음) | 생략 (동일하게 창 뜸) |
| 창 배율 | `:1.5` 오타 → **1.0(기본)** | 미지정 → 1.0 (결과적으로 동일) |
| ROI | 미지정 → `-1`(YAML값, ROI 컷 없음) | `0.5` → **하단 절반만 ROI 적용** |
| 결론 | ROI 없이 원본 파이프라인 뷰어 | ROI 적용 후 파이프라인 뷰어 |

즉 (A)는 **ROI 미적용** + 창 확대도 오타로 미적용(1.0배)이고, (B)는 **하단 ROI가 적용**됩니다.
창을 키우려면 `debug_window_scale:=1.5`로 고쳐 쓰고, ROI까지 함께 보려면:
```bash
ros2 launch lane_detection cam2_yolopv2.launch.py \
  bag_path:=/home/wise/Desktop/7_to_hoamji debug_window_scale:=1.5 roi_top_ratio:=0.5
```

## 1) 뷰어 모드 (프레임 단위 육안 검증)
bag을 직접 읽어 이미지+값 패널을 한 창에 표시.
```bash
ros2 launch lane_detection cam2_yolopv2.launch.py \
  bag_path:=/home/wise/Desktop/7_to_hoamji \
  roi_top_ratio:=0.5
```
키 조작:
| 키 | 기능 |
|---|---|
| Space | 재생 / 일시정지 |
| ← / → | 이전 / 다음 프레임 |
| , / . | 배속 -0.1 / +0.1 (0.1~4.0) |
| 숫자 + Enter | 해당 프레임으로 이동 (GOTO) |
| Backspace | GOTO 입력 지우기 |
| q / Esc | 종료 |
| 트랙바 | 프레임 드래그 이동 |

- 정보 패널의 `roi_top_ratio`가 >0이면 초록색으로 표시(ROI 활성 확인).

## 2) Headless 성능 측정 (구독 모드)
결과창 없이 노드 실행 → bag 재생 → `/camera/lane_result` 집계.
```bash
# 터미널 A: 노드 (하단 절반 ROI)
ros2 run lane_detection yolopv2_cam2_node --ros-args \
  -p debug_mode:=false \
  -p input_topic:=/camera/image_rect \
  -p roi_top_ratio:=0.5

# 터미널 B: bag 재생
ros2 bag play /home/wise/Desktop/7_to_hoamji --rate 2

# 터미널 C: 통계 집계
python3 /home/wise/.claude/jobs/5d546aa3/tmp/analyze_lane.py
```

## 3) before / after 비교 (ROI off vs on)
같은 절차를 `roi_top_ratio:=0.0`(off)과 `:=0.5`(on)로 각각 돌려 통계 비교.
비교 지표: degenerate 프레임 수, position/heading 표준편차, GOOD 비율, view_range 중앙값.

## 참고
- 파라미터 이론/변수 의미: `../lane_detection_params.md`
- 파이프라인 소스 위치: `../lane_source_map.md`
- 실측 문제점(P1~P5): `../lane_perf_diagnosis.md`
