# GNSS Heading 미동작 이슈 분석 및 해결

> Moving-baseline heading (`/rover/ubx_nav_rel_pos_ned`) 미동작 문제
> ArduSimple simpleRTK2B Heading 킷 (moving base + rover) / ublox_dgnss 드라이버

---

## 1. 이슈 (Issue)

- `/rover/ubx_nav_rel_pos_ned` 토픽은 발행되지만 **heading 값이 전부 0**.
- 방위(heading)를 얻을 수 없어 차량 자세 추정 불가.

**증상 (rover relposned 실측):**

| 필드 | 값 | 의미 |
|------|-----|------|
| `diff_soln` | true | 절대 RTK 보정은 수신 중 ✅ |
| `carr_soln.status` | 1 (FLOAT) | RTK Float |
| `is_moving` | **false** | moving-baseline 미동작 ❌ |
| `rel_pos_valid` | **false** | 상대위치 무효 ❌ |
| `rel_pos_heading_valid` | **false** | heading 무효 ❌ |
| `rel_pos_heading` | 0 | 방위 없음 |

→ **절대위치 RTK는 되지만 heading만 안 됨.**

---

## 2. 원인 분석 (Root Cause Analysis)

### 2-1. 최초 가설 (틀림)
> "moving base가 moving-base 모드(RTCM 4072 출력)로 설정되지 않았다"

→ base 설정을 **직접 실측(pyubx2 CFG-VALGET)** 하여 검증한 결과 **가설은 틀림.**

### 2-2. 실측 결과 — base는 이미 정상 설정

base UART1 (rover로 나가는 출력 링크) 설정:

| 항목 | 값 | 판정 |
|------|-----|------|
| `RTCM 4072.0` 출력 | **ON** | heading 핵심 메시지, 이미 켜짐 ✅ |
| `RTCM 1077/1087/1097/1127` (MSM7) | ON | 4개 위성군 관측치 ✅ |
| `RTCM 1230` | ON (10에폭) | GLONASS bias ✅ |
| `TMODE_MODE` | 0 (disabled) | moving base라 정상 ✅ |
| UART1 baud / 프로토콜 | 115200 / UBX+RTCM3X | 정상 ✅ |

→ pyrtcm CRC 검증으로 **4072 + MSM7 실제 출력까지 확인.**
→ 즉 **base는 손댈 필요 없음.**

### 2-3. 진짜 원인 — 측정 레이트 불일치

| 장치 | `CFG_RATE_MEAS` | 측정 레이트 |
|------|-----------------|-------------|
| moving base | 1000 ms | **1 Hz** |
| rover | 200 ms | **5 Hz** |

> **moving-baseline heading은 rover와 base의 측정 레이트가 같아야 동작한다.**
> 레이트가 어긋나면 rover가 상대해를 풀지 못해 `is_moving=false`, relposned 전부 0.

---

## 3. 해결 (Solution)

### 조치
**rover 측정 레이트를 base(1 Hz)에 맞춤:**

```bash
ros2 param set /rover/ublox_dgnss CFG_RATE_MEAS 1000   # 200ms(5Hz) → 1000ms(1Hz)
```

영구 적용 — 런치파일 수정 (`ublox_mb+r_rover.launch.py`):
```python
{'CFG_RATE_MEAS': 0x3e8},   # 1000 ms = 1 Hz (base와 정합)
```

### 결과 (해결 후 실측)

| 필드 | 변경 전 | 변경 후 |
|------|---------|---------|
| `is_moving` | false | **true** ✅ |
| `rel_pos_valid` | false | **true** ✅ |
| `rel_pos_heading_valid` | false | **true** ✅ |
| `rel_pos_heading` | 0 | **≈ 78~81°** ✅ |
| `rel_pos_length` (baseline) | 0 | **≈ 92 cm** ✅ |

→ **heading 정상 동작 확인.** (FLOAT 상태에서도 heading 출력, FIXED면 더 정밀)

---

## 4. 정리 & 트레이드오프

- **핵심 교훈:** heading 미동작 = base 설정 문제가 아니라 **rover/base 레이트 정합 문제.**
- **트레이드오프:** rover를 1 Hz로 낮춰 위치 출력도 1 Hz가 됨.
  - 더 높은 heading 레이트가 필요하면 → base도 동일 레이트로 올리고 Flash 저장
    (단 UART1 115200 대역폭이 5 Hz 풀 MSM7을 감당하는지 확인 필요).
- **진단 근거 파일:** `base_cfg_before.txt` (변경 전 base/rover 전체 설정 덤프).

---

### 참고 — RTCM 4072란?
- u-blox **독자(proprietary)** 메시지, **moving baseline 전용.**
- **4072.0**: base 자신의 실시간 위치(PVT)를 rover에 전달 → rover가 두 안테나 상대벡터·heading 계산.
- **4072.1**: 부가 기준국 정보 (선택).
- 일반 고정 base RTK(1005/1006)와 달리, base가 움직이므로 매 순간 위치를 4072로 전송.
