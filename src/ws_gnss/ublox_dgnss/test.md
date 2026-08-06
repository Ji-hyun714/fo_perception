# 1. (최초 1회) udev 규칙 — 안 하면 권한 에러
sudo cp src/ws_gnss/99-ublox-dgnss.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. 런치 실행
ros2 launch ublox_dgnss ublox_mb+r_rover.launch.py


ros2 topic echo /rover/ubx_nav_pvt
ros2 topic echo /rover/ubx_nav_rel_pos_ned
ros2 topic echo /rover/ubx_nav_pvt --field carr_soln
ros2 topic echo /rover/ubx_nav_pvt --field diff_soln



# 3. 토픽
/rover/ubx_nav_pvt
    : lat, long (고정밀은 /ubx_nav_hp_pos_llh)
    : rtk 여부 (carr_soln, diff_soln 조합으로 판단)
        carr_soln 1: float / 2: fixed
        diff_soln true: dgps / false: none(단독)
/rover/ubx_nav_rel_pos_ned
    : rel_pos_heading (방위각(도), NED 기준 (0°=북, 시계방향))
    : rel_pos_heading_valid (true 일 때만 신뢰. false면 heading 안 도는 상태 (moving base 연결/RTCM 확인 필요))



# 4. 순서
1. 런치 및 디버깅  
ros2 launch ublox_dgnss ublox_mb+r_rover.launch.py
ros2 topic echo /rover/ubx_nav_pvt
ros2 topic echo /rover/ubx_nav_rel_pos_ned --field is_moving
ros2 topic echo /rover/ubx_nav_rel_pos_ned --field rel_pos_heading
ros2 topic echo /rover/ubx_nav_pvt --field carr_soln
ros2 topic echo /rover/ubx_nav_pvt --field diff_soln

2. rtk 설정
sudo stty -F /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0 \
  115200 cs8 -cstopb -parenb -ixon -ixoff

3. 데이터 확인
sudo cat /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0

4. 브릿지 실행
python3 src/ws_gnss/mrd_rtcm_bridge.py

5. 토픽으로 검증
ros2 topic echo /rover/ubx_rxm_rtcm --field msg_used
    -> 2 나와야함
ros2 topic echo /rover/ubx_nav_pvt --field carr_soln
    -> # 1=float → 2=fixed 로 올라감




** 참고 명령어
sudo cat /dev/ttyUSB1
ls -l /dev/serial/by-id/
** socat 열어주기
sudo socat -u \
  /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0,b115200,raw \
  /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0,b115200,raw


python3 src/ws_gnss/mrd_rtcm_bridge.py
# 검증:
ros2 topic echo /ubx_rxm_rtcm   # msg_used: 2 → RTK 보정 적용됨



MRD → moving base 주입 (단방향 socat):

MRD=/dev/serial/by-id/usb-FTDI_USB_Seria
MRD=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0
MB=/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0
socat -u $MRD,b115200,raw $MB,b115200,raw


===================================================================
# MRD 절대보정 반영 확인 (★ 중요: base를 봐야 함, rover 아님)
===================================================================

MRD-1000은 moving base의 '절대'해만 바꾼다. rover 토픽(diff_soln/carr_soln)은
moving-base 내부 UART2 링크가 만드는 '상대'해라, MRD를 뽑아도 안 변한다.
→ MRD 반영 여부는 반드시 base의 carr_soln으로 판정한다.

# 배선 (by-id 고정. ttyUSB 번호는 재연결 시 뒤바뀜)
#   MRD-1000 v2      = usb-FTDI_USB_Serial_Converter_FTGI4BH8 (RTCM 소스)
#   moving base UART1 = usb-FTDI_FT230X_Basic_UART_DU0F4U14   (주입 지점 = 위 보드)
#   rover            = u-blox 네이티브 USB (ublox_dgnss가 libusb로 점유)

# 1) MRD → moving base 주입 (연속). sudo 불필요(dialout).
socat -u \
  /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0,b115200,raw \
  /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0,b115200,raw &

# 2) base RTK 상태를 토픽으로 노출하는 노드 실행 (read-only라 socat과 충돌 없음)
python3 src/ws_gnss/base_rtk_monitor.py

# 3) 토픽으로 직관 확인
ros2 topic echo /base/rtk_status          # 사람이 읽는 한 줄 요약
ros2 topic echo /base/carr_soln --field data   # 0=NONE 1=FLOAT 2=FIXED

## 토픽 설명
/base/carr_soln   (std_msgs/UInt8)  : base 반송파해. 0=NONE(단독) / 1=FLOAT / 2=FIXED.
                                      ★ MRD 반영 판정: 0 → 1/2 로 바뀌면 반영됨.
/base/rtk_status  (std_msgs/String) : "FLOAT carrSoln=1 hAcc=128mm vAcc=.. sv=32 fixType=3 diffSoln=1"
                                      MRD 반영 시 hAcc가 수백mm → cm 로 급감.

# 전제(1회 설정, 이미 완료·Flash 저장): base UART1이 NAV-PVT 자동출력해야 함
#   pyubx2로 CFG_MSGOUT_UBX_NAV_PVT_UART1=1 (layers RAM+Flash) 설정됨.

## rover 쪽 참고 토픽 (mb+r 런치에서 출력 켬)
/rover/ubx_rxm_rtcm  : rover가 UART2(base 링크)로 받은 RTCM 사용여부(msg_used).
                       base 링크 지표지 MRD 지표 아님. 런치에 CFG_MSGOUT_UBX_RXM_RTCM_USB=1 추가함.
                       (반영하려면: colcon build --packages-select ublox_dgnss 후 런치 재기동)
