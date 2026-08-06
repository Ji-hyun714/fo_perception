# rover heading 5Hz 문제 분석 (heading 동작 vs 10Hz 트레이드오프)

> base 위치는 10Hz 달성([hz_개선.md](hz_개선.md))했으나, **rover heading(relposned)은 5Hz** 에 머무름.
> 그 원인을 단계별로 추적한 기록.
> 관련: [hz_개선.md](hz_개선.md) (base 위치 10Hz), [gnss_heading_issue_report.md](gnss_heading_issue_report.md) (heading 최초 동작)

---

## 1. 상황

base 위치(`/base/ubx_nav_pvt`)를 10Hz 로 올린 뒤 heading 을 확인:

| 토픽 | 레이트 |
|------|--------|
| `/base/ubx_nav_pvt` (위치) | **10Hz** ✅ |
| `/rover/ubx_nav_rel_pos_ned` (heading) | **5.00Hz** ❌ (칼같이 5.00, std dev ~0.003) |

heading 값 자체는 **정상 동작**:
```
carr_soln.status = 2 (FIXED)
is_moving = true, rel_pos_valid = true, rel_pos_heading_valid = true
rel_pos_heading = 20660554 → ×1e-5 = 206.6°
```
→ **heading 은 완벽히 되는데 레이트만 5Hz.**

핵심 단서: 5.00Hz 가 **너무 깨끗함**(std dev≈0.003). 대역폭 포화면 rate 가 들쭉날쭉해야 하는데
칼같은 5.00 → **어딘가 5Hz 로 "결정"되어 있다**(랜덤 드롭/대역폭 문제 아님).

---

## 2. 단계별 진단

### STEP 1 — heading 이 처음엔 아예 0 이었음 (rate 불일치)
base 10Hz 로 올린 직후 relposned:
```
is_moving = false, rel_pos_valid = false, carr_soln = 0, rel_pos_heading = 0
```
→ moving-baseline 미동작. [gnss_heading_issue_report.md] 와 동일한 **base/rover 레이트 불일치**.
  base=10Hz, rover=위성군 다 켠 채로 ~8Hz 로 clamp → 불일치.

### STEP 2 — rover 위성군을 base 와 맞춤 → heading 동작
```bash
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_GLO_ENA false
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_QZSS_ENA false
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_SBAS_ENA false
```
※ 주의: 이 파라미터는 **bool 타입** → `0` 이 아니라 `false` 로 줘야 함
  (`0` 주면 "Wrong parameter type ... is of type {bool}" 에러).

→ 이후 heading FIXED 동작 시작. **단, 이 과정에서 `diff_soln` 이 잠깐 false 로 떨어짐** (아래 STEP 3).

### STEP 3 — diff_soln=false 회귀: socat baud 불일치 + 중복
`ps aux | grep socat` 확인 결과:
- socat 이 base UART1(`DU0F4U14`)에 **`b115200` 으로 보정 주입**하는데, 앞서 base UART1 을
  **460800 으로 바꿔놨음** → base 가 MRD 절대보정을 못 알아들어 RTK 깨짐 → `diff_soln=false`.
- socat 프로세스가 **2개** 떠서 같은 base 포트에 동시 write → RTCM 깨짐.

**중요 교훈: base UART1 baud(460800)는 애초에 불필요했음.**
5Hz 의 원인은 UART 대역폭이 아니라 nav clamp 였고([hz_개선.md] STEP 3), UART1 은
NAV-PVT 출력 + 보정 입력만 실어 115200 으로 충분. → **115200 으로 되돌림.**

```bash
python3 utils/set_base_rate.py --baud 460800 --uart1-baud 115200   # 460800 접속→115200 로 복귀
kill <socat-pid1> <socat-pid2>                                     # 중복 socat 종료
socat -u /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0,b115200,raw \
        /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0,b115200,raw &
```
→ `diff_soln=true` 회복, heading **FIXED** 정상.

### STEP 4 — 5Hz 원인 추적: base RTCM 출력? XBee? rover?
- base UART2(XBee) RTCM 출력 = **전부 0** → heading 보정은 UART2 아님.
  (heading 리포트대로 **base UART1 이 rover 로 나가는 RTCM 링크**. diag 스트림에 0xD3=RTCM3 preamble 섞여나옴.)
- rover 자체 레이트 실측:
  ```bash
  ros2 topic hz /rover/ubx_nav_pvt        # → 5.00Hz !!
  ```
  → **rover nav 자체가 5Hz.** base RTCM 이 아니라 rover 가 5Hz 로 도는 게 원인.

### STEP 5 — rover 설정 확인: 10Hz 인데 5Hz 출력 (연산 clamp)
```bash
ros2 param get /rover/ublox_dgnss CFG_RATE_MEAS          # 100 (10Hz)
ros2 param get /rover/ublox_dgnss CFG_RATE_NAV           # 1
ros2 param get /rover/ublox_dgnss CFG_MSGOUT_UBX_NAV_PVT_USB        # 1
ros2 param get /rover/ublox_dgnss CFG_MSGOUT_UBX_NAV_RELPOSNED_USB  # 1
```
→ 설정은 전부 10Hz(분주 1)인데 실측 5Hz. **출력분주 문제 아님 → rover nav 엔진 clamp.**

### STEP 6 — 원인 확정: moving-baseline RTK 연산 부하
결정적 관찰:
- **heading 이 안 되던 때(STEP 1)** rover 는 10Hz 였음 → moving-baseline RTK 를 **안 돌려서** 가벼웠음.
- **heading 이 FIXED 로 동작하는 지금** rover 는 5Hz → **base 관측치로 상대해(RTK FIXED)를
  계산하느라 부하 급증** → 5Hz clamp.

> **결론: rover 는 base 와 달리 moving-baseline RTK 까지 돌려서 훨씬 무겁다.**
> "heading 동작 + 10Hz" 를 동시에 하기엔 rover 연산이 부족 → 5Hz 로 clamp.
> (base 10Hz ≥ rover 5Hz 라 heading 자체는 FIXED 로 잘 됨. 레이트 요구조건은 base rate ≥ rover rate.)

---

## 3. 현재 상태 & 남은 선택지

### 확정된 사실
- base 위치 = **10Hz** (달성)
- rover heading = **5Hz, FIXED 정상 동작** (레이트만 5Hz)
- 5Hz 원인 = **rover 의 moving-baseline RTK 연산 clamp** (대역폭/설정/분주 아님)

### heading 도 10Hz 로 올리려면 (검증 중)
rover 부하를 더 낮춰 10Hz 여유 확보 시도:
```bash
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_BDS_ENA false   # BeiDou 도 끄고 gps,gal 만
ros2 topic hz /rover/ubx_nav_rel_pos_ned                     # 10Hz 되나
ros2 topic echo /rover/ubx_nav_rel_pos_ned --once            # is_moving=true, FIXED 유지되나
```
판정:
- **10Hz + is_moving=true + FIXED** → 성공 (둘 다 달성)
- **10Hz 인데 FLOAT/불안정** → 위성 부족, BeiDou 복구 후 **heading 5Hz 수용**
- **여전히 5Hz** → ZED-F9P RTK rover 의 실질 상한(≈5~8Hz), **heading 5Hz 수용** (위치 10Hz 유지)

### 트레이드오프
- rover 위성군 ↓ = 연산 여유 ↑(rate↑) 이지만 **RTK FIXED 도달/heading 품질 ↓**.
- 많은 응용에서 **위치 10Hz + heading 5Hz 로 충분** (차량 자세는 위치만큼 빨리 안 변함).
- L2 밴드까지 끄면(단밴드) 더 가볍지만 RTK 품질 급락 → 비추천.

---

## 4. 명령어 요약

```bash
# --- 레이트 실측 ---
ros2 topic hz /base/ubx_nav_pvt              # base 위치 (10Hz 목표)
ros2 topic hz /rover/ubx_nav_pvt             # rover nav (heading rate 상한)
ros2 topic hz /rover/ubx_nav_rel_pos_ned     # heading
ros2 topic echo /rover/ubx_nav_rel_pos_ned --once   # heading 플래그/값

# --- rover 위성군 (bool! false 로) ---
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_GLO_ENA false
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_QZSS_ENA false
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_SBAS_ENA false
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_BDS_ENA false      # 10Hz heading 시도용
ros2 param set /rover/ublox_dgnss CFG_SIGNAL_BDS_ENA true       # 복구(heading 품질 우선)

# --- rover 레이트 확인 ---
ros2 param get /rover/ublox_dgnss CFG_RATE_MEAS      # 100
ros2 param get /rover/ublox_dgnss CFG_RATE_NAV       # 1

# --- base UART1 baud 되돌리기 (460800→115200) + socat 정리 ---
python3 utils/set_base_rate.py --baud 460800 --uart1-baud 115200
kill <socat-pid...>
socat -u /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0,b115200,raw \
        /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0,b115200,raw &

# --- base UART1/UART2 RTCM 출력 진단 ---
python3 utils/diag_base_uart1_rtcm.py    # rover 로 나가는 RTCM(UART1) 분주값
python3 utils/diag_base_uart2.py         # UART2(XBee) — 이 구성은 전부 0
```

---

## 5. 핵심 교훈

1. **base 위치 10Hz 와 rover heading 10Hz 는 별개 병목.**
   - base 위치: nav clamp(위성군) → 위성군 축소로 해결.
   - rover heading: moving-baseline RTK 연산 부하 → 더 어려움.
2. **heading 이 "동작"하면 rover 부하가 급증**한다. heading OFF 상태의 10Hz 는 의미 없음.
3. **rover 파라미터는 bool** → `false`/`true` (숫자 0/1 아님).
4. **base UART1 baud 는 건드리지 말 것** — socat/base_rtk_flag 와 baud 정합 깨지고, 대역폭
   이득도 없음(5Hz 는 nav clamp 였음). 115200 유지.
5. **레이트 요구조건**: moving-baseline 은 **base rate ≥ rover rate** 면 동작
   (base 10Hz > rover 5Hz 에서도 FIXED 정상). 최초 미동작은 base 1Hz < rover 5Hz 였기 때문.
