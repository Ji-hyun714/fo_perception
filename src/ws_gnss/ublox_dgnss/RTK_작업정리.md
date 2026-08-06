# ublox_dgnss로 MRD-1000 외부 RTK + heading 작업 정리

> ArduSimple simpleRTK2B **Heading 킷**(moving base + rover)에 Synerex **MRD-1000 v2**(방송 RTK 동글)로
> 외부 보정을 넣는 작업. **ublox_dgnss** 드라이버 사용 (NTRIP 대신 MRD, USB 연결).
> 작업일: 2026-07-21

---

## 1. 목표

- **lat/long, heading, RTK 여부(none/dgps/float/fixed), x/y 속도** 를 rover에서 얻기
- 외부 보정은 **MRD-1000**(무선 RTK 동글)에서 받음 — NTRIP 안 씀

---

## 2. 왜 ublox_dgnss 인가

| | ros2-ublox-zedf9p | **ublox_dgnss** |
|---|---|---|
| 외부 RTCM 주입(ROS) | ❌ (UART 직결만) | ✅ `/ntrip_client/rtcm` → USB 주입 |
| RTCM 사용 확인 | diff_soln만(간접) | **`msg_used=2` 직접 확인** |
| device 접근 | CDC (`/dev/ttyACM0`) | **libusb raw USB** |
| 값 포맷 | raw 비트마스크 | 디코딩된 필드 (carr_soln 등) |

> ⚠️ ublox_dgnss는 libusb로 F9P를 잡으므로 **ros2-ublox-zedf9p 노드와 동시 실행 금지** (장치 점유 충돌).

---

## 3. 하드웨어 / 포트 매핑

by-id 사용 권장 (ttyUSB 번호는 재연결 시 뒤바뀜):

| 정체 | by-id | 이번 세션 |
|------|-------|-----------|
| **rover** (F9P) | `usb-u-blox_AG_..._GNSS_receiver-if00` | libusb (ttyACM 아님), USB id `1546:01a9` |
| **MRD-1000 v2** | `usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0` | ttyUSB0 |
| **moving base UART1** | `usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0` | ttyUSB1 |

- MRD-1000 v2: RS232→USB, RTCM3, **115200 8N1**, 1006/1008/1019/1020/1033/1074~1124(MSM4)/1230
- moving base → rover 는 **보드 간 UART 배선**(오린 안 거침). moving base의 4072를 rover **UART2**로 전달.

---

## 4. 실행 준비

### 4-1. udev 규칙 (libusb 접근, 최초 1회)
```bash
sudo cp src/ws_gnss/99-ublox-dgnss.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
`99-ublox-dgnss.rules`:
```
SUBSYSTEM=="usb", ATTR{idVendor}=="1546", MODE="0666"
```

### 4-2. 런치 선택
moving base + rover 이므로 **`ublox_mb+r_rover.launch.py`**.
(`_navsatfix` 런치는 HPPOSLLH/STATUS/COV/RXM_RTCM만 켜서 heading·RTK상태·속도 안 나옴 → 부적합)

---

## 5. 겪은 문제 & 해결 (launch 수정, CLAUDE.md 규칙대로 주석+TODO 보존)

### 5-1. `usb init error: ... specified serial string ... "P�" was found`
- 원인: 런치 기본값 `device_serial_string="Test Rover"` vs F9P의 깨진 iSerial ("Reliable iSerial: NO")
- 해결: 기본값 **`""`** (빈 문자열 = 첫 장치 매칭)

### 5-2. `parameter supplied: CFG-UART2-BAUDRATE is not recognised. Ignoring!`
- 원인: 파라미터명이 하이픈 `CFG-UART2-BAUDRATE`
- 해결: 언더스코어 **`CFG_UART2_BAUDRATE`**

### 5-3. ★heading 안 됨 + RTK 안 붙음 → UART2 baud 불일치★
- 증상: `is_moving: false`, relposned 전부 0, nav_pvt `diff_soln: false`, `carr_soln: 0`
- 원인: 런치가 UART2를 **460800**으로 설정. 이 하드웨어 moving base는 UART2 링크로 4072를 **115200**으로 출력 → rover가 RTCM 못 받음
- 해결: **`CFG_UART2_BAUDRATE` = 115200**
- 검증(런타임 즉시 확인):
  ```bash
  ros2 param set /rover/ublox_dgnss CFG_UART2_BAUDRATE 115200
  # → nav_pvt: diff_soln=true, carr_soln=1(FLOAT)
  ```
- 런치 반영 후 `colcon build --packages-select ublox_dgnss` (install은 심링크 아니라 복사본이라 재빌드 필요)

### 5-4. `ros2 topic echo /rover/ubx_*` 가 조용히 빈 값
- 원인: ubx_* 토픽은 커스텀 msg(`ublox_ubx_msgs`). 워크스페이스 미소싱이면 역직렬화 실패로 빈 출력. `/rover/fix`(표준 NavSatFix)만 보임.
- 해결: **`source install/setup.bash`** 후 echo

---

## 6. 확인 토픽 (`/rover/` 네임스페이스)

```bash
source install/setup.bash
```

| 필요 | 토픽 | 필드 | 변환 |
|------|------|------|------|
| **lat** | `/rover/ubx_nav_pvt` | `lat` | ÷ 1e7 |
| **long** | `/rover/ubx_nav_pvt` | `lon` | ÷ 1e7 |
| **heading** | `/rover/ubx_nav_rel_pos_ned` | `rel_pos_heading` | ÷ 1e5 (NED, `rel_pos_heading_valid=true`일 때만) |
| **RTK 여부** | `/rover/ubx_nav_pvt` | `carr_soln`+`diff_soln` | 아래 표 |
| **x/y 속도** | `/rover/ubx_nav_pvt` | `vel_e`, `vel_n` | ÷ 1000 = m/s (NED) |

**RTK 상태 판정** (CarrSoln: 0=none,1=float,2=fixed):

| carr_soln.status | diff_soln | 상태 |
|:---:|:---:|---|
| 2 | - | RTK FIXED (cm) |
| 1 | - | RTK FLOAT |
| 0 | true | DGPS |
| 0 | false | NONE (단독) |

---

## 7. MRD → moving base 주입 (절대위치 RTK)

이 런치는 `CFG_USBINPROT_RTCM3X=False`라 **USB 주입 안 먹힘**. RTCM 입력은 **UART2(=moving base)** 전용.
따라서 MRD는 **moving base에 socat 단방향 주입**:
```bash
MRD=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0
MB=/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0
socat -u $MRD,b115200,raw $MB,b115200,raw
```
- `-u`(단방향) 필수, `-x`/양방향 금지
- (만든 `src/ws_gnss/mrd_rtcm_bridge.py`는 /ntrip_client/rtcm→USB 경로라 이 구성에선 미사용)

---

## 8. 현재 상태 (2026-07-21)

| 항목 | 상태 |
|------|------|
| rover 3D fix (위성 8) | ✅ |
| **절대위치 RTK (MRD)** | ✅ FLOAT (FIXED 수렴 대기) |
| **heading (moving-baseline)** | ❌ `is_moving=false` |

### heading 미해결 원인
`is_moving=false` = moving base가 **moving-base 모드(4072.0/4072.1 출력)로 설정 안 됨**. 표준 base RTCM만 rover로 보내서 rover는 일반 RTK만 하고 상대벡터(heading) 계산 안 함.

---

## 9. 남은 작업 (heading 복구)

moving base(ttyUSB1의 별도 F9P, rover 노드가 관리 안 함)를 pyubx2로 직접 설정:
1. moving base 현재 설정 조회 (UBX-CFG-VALGET)
2. moving-base 출력 켜기: `CFG_MSGOUT_RTCM_3X_TYPE4072_0/4072_1`, MSM7(1077/1087/1097/1127), 1230 을 rover 연결 포트로
3. moving base TMODE=0 확인 (survey-in/fixed 아님)
4. flash 저장
5. 검증: rover `/rover/ubx_nav_rel_pos_ned`에서 `is_moving=true`, `rel_pos_heading_valid=true`

---

## 10. 도구 / 참고

- pyubx2/pyrtcm/pyserial 설치됨
- `src/ws_gnss/mrd_rtcm_bridge.py` — MRD 시리얼 → /ntrip_client/rtcm 브릿지 (USB 주입 구성용, 현재 미사용)
- `src/ws_gnss/99-ublox-dgnss.rules` — libusb udev 규칙
- 관련: `../ros2-ublox-zedf9p/GNSS_HEADING_RTK_정리.md` (다른 드라이버 경로 기록)
