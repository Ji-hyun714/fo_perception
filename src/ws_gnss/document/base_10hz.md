# base GNSS 출력 레이트 10Hz 개선 (1Hz → 10Hz)

> `/base/ubx_nav_pvt` 발행 레이트를 1Hz → 10Hz 로 끌어올린 작업 기록.
> MRD-1000v2 라디오로 10Hz 운용 가능해짐에 따라 base F9P 를 10Hz 로 전환.
> 관련: [gnss_heading_issue_report.md](gnss_heading_issue_report.md) (heading 레이트 정합)

---

## 1. 목표 & 배경

- MRD-1000v2 라디오가 10Hz RTCM 전달 가능 → base 를 10Hz 로 올리고자 함.
- 기존: base/rover 모두 1Hz (heading moving-baseline 정합 위해 1Hz 로 맞춰뒀었음).
- `base_rtk_flag2.py` 는 base UART1 을 read-only 로 열어 NAV-PVT 를 파싱·발행하는
  **수동(passive) reader** → 발행 레이트 = base F9P 의 UART1 NAV-PVT 출력 레이트 그대로.
  즉 이 노드는 레이트 제한이 없고, 병목은 항상 F9P/링크 쪽.

### 핵심 개념 정리
- **MRD-1000v2 의 "10Hz"** = 무선 링크가 RTCM 보정을 10Hz 로 전달하는 것 (rover 가 10Hz
  보정/heading 을 받게 함). **base 자신의 NAV-PVT 출력 레이트와는 별개** (라디오와 무관하게
  UART1 로 로컬 출력).
- `/base/ubx_nav_pvt` 10Hz 에 필요한 3요소:
  1. base F9P `CFG_RATE_MEAS=100`(10Hz)
  2. rover 도 같은 레이트 (heading 정합, [gnss_heading_issue_report.md] 참고)
  3. UART1 대역폭 + nav 엔진 연산능력이 10Hz 를 감당

---

## 2. 진행 순서와 이슈 (실측 기반 단계 진단)

### STEP 0 — 검증 도구 준비: `base_rtk_flag3.py`
`base_rtk_flag2.py` 기반. 10Hz 도달 여부를 눈으로 확인하려고 두 가지 추가:
- **실측 발행 Hz 로깅** (`--report-sec` 마다, 기본 2초)
- **status 로그 throttle** (10Hz 콘솔 도배 방지, carrSoln 변화 시만 로깅; `--log-status` 로 매번)

### STEP 1 — 초기 실측: 1Hz
```
/base/ubx_nav_pvt 실측 rate = 1.00 Hz
```
→ base F9P 가 아직 1Hz 출력. **rover 를 10Hz 로 바꿔도 소용없음** (base_rtk_flag3 은
base 를 읽음; rover 레이트는 heading 정합용일 뿐 base 발행 레이트와 무관).

### STEP 2 — base 를 10Hz 로 설정 → 5Hz 로 clamp
`set_base_rate.py` 로 `CFG_RATE_MEAS=100` 기록 (RAM+BBR+Flash).
```
CFG_RATE_MEAS = 100, CFG_RATE_NAV = 1   # 설정은 10Hz
→ 실측 rate = ~5 Hz                       # 그런데 5Hz
```
→ **이슈: 설정은 10Hz 인데 실측 5Hz.** 대역폭 또는 nav 엔진 한계 의심.

### STEP 3 — UART1 baud 상향 (115200 → 460800) → 여전히 5Hz
`set_base_rate.py --uart1-baud 460800` (baud 는 항상 마지막에 적용, 변경 후 새 baud 로 재접속 검증).
`base_rtk_flag3.py --baud 460800` 로 확인:
```
→ 실측 rate = ~5 Hz  (그대로)
```
→ **대역폭이 원인이 아님.** 460800 에서 NAV-PVT 가 깨끗이 파싱되므로 baud 변경은 정상 반영됨
  (mismatch 였으면 CRC 깨져 거의 안 잡힘). 4배 올려도 그대로 → **nav 엔진 자체가 5Hz clamp**.

### STEP 4 — 원인 진단: 위성군 과다
`diag_base_rate.py` (read-only VALGET) 로 확인:
```
CFG_RATE_MEAS = 100, CFG_RATE_NAV = 1, CFG_UART1_BAUDRATE = 460800
CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1        # 출력 분주 정상(1)
활성 위성군 = GPS(L1+L2) + GAL(E1+E5b) + BDS(B1+B2) + GLO(L1+L2) + QZSS + SBAS  # 전부 ON
```
→ **원인 확정: 위성군을 최대로 다 켜서 ZED-F9P 가 RTK 10Hz 연산을 못 따라가 5Hz 로 clamp.**
  (분주값·baud·rate 설정은 모두 정상. 순수 nav 엔진 연산 한계.)

### STEP 5 — 위성군 축소 → 10Hz 달성
`set_base_constellations.py --keep gps,gal,bds` (GLO/QZSS/SBAS OFF):
```
/base/ubx_nav_pvt 실측 rate = 10.0~10.5 Hz   # 달성
carrSoln: NONE → FLOAT, sv = 23
```
→ **성공.** clamp 원인이 위성군 부하였음이 확정됨.

### STEP 6 — QZSS 추가 시도 → 8~10Hz (경계)
`--keep gps,gal,bds,qzss` → 실측 8~10Hz 로 출렁임.
→ QZSS 를 더하니 연산이 10Hz 한계에 걸려 종종 8~9 로 하락.
→ **QZSS 소폭 이득보다 안정적 10Hz 가 우선** → `gps,gal,bds` 로 복귀 권장.

---

## 3. 최종 구성

| 항목 | 값 | 비고 |
|------|-----|------|
| base `CFG_RATE_MEAS` | 100 (10Hz) | RAM+BBR+Flash |
| base `CFG_RATE_NAV` | 1 | 매 측정마다 solution |
| base `CFG_MSGOUT_UBX_NAV_PVT_UART1` | 1 | 출력 분주 (모든 에폭 출력) |
| base UART1 baud | 460800 | 호스트(base_rtk_flag3/socat)도 동일해야 함 |
| 활성 위성군 | **GPS + Galileo + BeiDou** | GLONASS/QZSS/SBAS OFF |
| rover `CFG_RATE_MEAS` | 100 | heading moving-baseline 정합용 |
| 결과 | `/base/ubx_nav_pvt` 안정 **10Hz**, sv≈23, FLOAT→FIXED 수렴 |

---

## 4. 명령어 모음 (재현/복구용)

작업 디렉토리: `~/fo_perception/src/ws_gnss`

```bash
# --- 0. 실측 Hz 확인 (검증 도구) ---
python3 base_rtk_flag3.py --baud 460800          # 2초마다 실측 Hz 로깅

# --- 1. base 진단 (read-only) ---
python3 utils/diag_base_rate.py                  # rate/분주/baud/위성군 조회

# --- 2. base 를 10Hz 로 설정 ---
#     ※ socat / base_rtk_flag 잠깐 멈추고 실행 (같은 UART1 포트 write 충돌 방지)
python3 utils/set_base_rate.py                   # CFG_RATE_MEAS=100 (10Hz), Flash 저장
python3 utils/set_base_rate.py --verify-only     # 쓰지 않고 현재값만 조회

# --- 3. UART1 baud 상향 (필요 시) ---
python3 utils/set_base_rate.py --uart1-baud 460800   # baud 는 마지막에 적용됨
#     이후 호스트 쪽도 460800 로: base_rtk_flag3.py --baud 460800, socat baud 도 460800

# --- 4. 위성군 축소 (10Hz clamp 해소, 핵심) ---
python3 utils/set_base_constellations.py --keep gps,gal,bds        # 최종 채택 (안정 10Hz)
python3 utils/set_base_constellations.py --keep gps,gal            # 더 가볍게
python3 utils/set_base_constellations.py --keep gps,gal,bds,qzss   # QZSS 추가(→8~10, 비추천)
python3 utils/set_base_constellations.py --verify-only            # 현재 위성군 조회

# --- 5. rover 레이트 정합 (heading 용) ---
ros2 param set /rover/ublox_dgnss CFG_RATE_MEAS 100
#     영구: ublox_mb+r_rover.launch.py 의 CFG_RATE_MEAS 를 100 으로

# --- 복구: 위성군 전부 되돌리기 ---
python3 utils/set_base_constellations.py --keep gps,gal,bds,glo,qzss,sbas
# --- 복구: 1Hz 로 되돌리기 ---
python3 utils/set_base_rate.py --meas-ms 1000
```

---

## 5. 트레이드오프 & 주의사항

- **위성군 ↓ = nav rate ↑ 이지만 RTK 품질 ↓ 가능**: 사용 위성 수가 줄어 FIXED 도달이
  느려지거나 heading 품질이 떨어질 수 있음. `gps,gal,bds` (sv≈23) 는 개활지에서 충분.
  차폐 심한 환경에서 fix 불안하면 위성군 조합 재검토.
- **위성군 우선순위**: GLONASS 가 가장 무거움(FDMA) → 10Hz clamp 의 주범. QZSS/SBAS 는
  가볍지만 SBAS 는 RTK 기여 없음, QZSS 는 소폭 이득(고각·아태 지역).
- **baud 변경은 항상 마지막**: `CFG_UART1_BAUDRATE` 를 Flash 에 바꾸는 순간 기존 baud
  연결이 끊김 → 호스트(base_rtk_flag3, socat)도 즉시 새 baud 로 재접속해야 함.
- **설정 write 시 포트 충돌**: base UART1 은 socat 가 RTCM 을 write, base_rtk_flag 가
  read 함. VALSET write 중 RTCM 이 섞이면 프레임이 깨질 수 있으니 잠깐 멈추고 실행 권장.
  (VALGET "응답 없음"/CRC 에러가 뜨면 이 때문 — 변경 후 재조회로 반영 여부 확인.)
- **rover heading 재확인 필요**: 10Hz 전환 후 rover 도 10Hz 정합 상태에서 `is_moving=true`,
  `rel_pos_heading` 정상값 나오는지 확인. ([gnss_heading_issue_report.md] 참고)
- **UART1 115200 대역폭 한계 참고**: NAV-PVT + rover 로 나가는 RTCM(4072+MSM7)까지 실려
  10Hz 풀 부하면 115200 포화 → 460800 로 상향. (단 이번 5Hz 는 대역폭이 아니라 nav clamp 였음.)

---

## 6. 산출 파일

| 파일 | 역할 |
|------|------|
| `base_rtk_flag3.py` | base RTK/위치/heading 발행 + **실측 Hz 로깅**(10Hz 검증용) |
| `utils/set_base_rate.py` | base `CFG_RATE_MEAS`/`CFG_RATE_NAV` + UART1 baud 설정 |
| `utils/diag_base_rate.py` | rate/분주/baud/위성군 read-only 진단 |
| `utils/set_base_constellations.py` | base 활성 위성군 조정 (clamp 해소 핵심) |
