#!/usr/bin/env python3
"""moving base F9P의 UBX-NAV-PVT를 파싱해 /base/ubx_nav_pvt 로 발행 (USB 재연결 대응 버전).

★ basePvt_rtkFlag.py 에 "USB 재연결(핫플러그) 대응" 을 추가한 버전.
   - 기존: 포트를 __init__ 에서 딱 한 번만 열고, run() 은 read() 예외를 전부
           `except Exception: continue` 로 삼켜버렸다. → USB를 뽑으면 read()가
           SerialException 을 계속 던지는데 그냥 busy-loop 만 돌고, 다시 꽂아도
           죽은 파일 핸들을 그대로 들고 있어 발행이 재개되지 않았다.
   - 변경: 시리얼 연결 끊김(SerialException/OSError)을 별도로 감지해 포트를
           다시 연다(open_stream). /dev/serial/by-id/... 심볼릭 링크는 고정이라
           다시 꽂으면 같은 경로로 재열거되어 발행이 재개된다.

base 는 rover 와 달리 ublox_dgnss(USB)가 잡지 않는 별도 F9P라 ROS 토픽이 없다.
이 노드가 base UART1(FTDI)을 READ-ONLY로 열어 base가 자동 출력하는 UBX-NAV-PVT를 파싱해 노출한다.

heading(head_veh) 은 base 자신의 값이 아니라, rover 가 moving-base 모드로 발행하는
/rover/ubx_nav_rel_pos_ned (base→rover 상대위치 NED 벡터)로부터 계산해 채운다.
(base F9P 는 센서퓨전 모드가 아니라 자체 head_veh 가 의미 없기 때문.)
base NAV-PVT 의 itow 와 rel_pos_ned 의 itow 가 같을 때만 heading 을 계산해 head_veh 에 싣는다.

★ MRD 반영 판정은 base로 확인:
    /base/carr_soln 이 0(NONE) → 1(FLOAT)/2(FIXED) 로 바뀌면 MRD 반영됨.

전제: base UART1이 NAV-PVT를 자동 출력해야 함
      (CFG_MSGOUT_UBX_NAV_PVT_UART1=1, layers RAM+Flash로 이미 설정 완료).
      socat가 같은 포트에 write(주입)해도 이 노드는 read-only라 충돌 없음.

토픽:
  /base/rtk_status  (std_msgs/String)             : 사람이 읽는 한 줄 요약
  /base/carr_soln   (std_msgs/UInt8)              : 0=NONE, 1=FLOAT, 2=FIXED  ← MRD RTK 반영 판정용
  /base/ubx_nav_pvt (ublox_ubx_msgs/UBXNavPVT)    : NAV-PVT 전 필드 (위치/속도/시각/정확도/RTK flag)
                                                    + head_veh 에 base→rover 계산 heading (deg×1e5)
                                                    + header.stamp = GNSS fix UTC(unix)
구독:
  /rover/ubx_nav_rel_pos_ned (ublox_ubx_msgs/UBXNavRelPosNED) : base→rover 상대위치 (heading 계산용)
"""

import argparse
import calendar
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import String, UInt8
from ublox_ubx_msgs.msg import UBXNavPVT, UBXNavRelPosNED
import serial
from pyubx2 import UBXReader

# moving base UART1: FTDI FT230X (serial DU0F4U14), 115200/8/N/1
DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0F4U14-if00-port0"
DEFAULT_BAUD = 115200
CARR = {0: "NONE", 1: "FLOAT", 2: "FIXED"}


def heading_from_ned(n_cm, e_cm):
    """NED 벡터(cm) → 방위각(deg, 북=0, 동=90, 0~360)."""
    return (math.degrees(math.atan2(e_cm, n_cm)) + 360.0) % 360.0


def pvt_utc_to_ros_time(parsed):
    """NAV-PVT 페이로드의 UTC(year..sec + nano) → 데이터 취득 시각 (builtin_interfaces/Time).

    header.stamp 는 unix epoch(1970-01-01 UTC 기준) 형식의 sec/nanosec 이다.
    calendar.timegm() 으로 UTC 달력값을 unix 초로 환산하고 nano(초의 소수, -1e9~1e9)를 더한다.
    UTC 가 아직 확정되지 않았으면(validDate/validTime/fullyResolved 미설정) None 반환.
    """
    if not (getattr(parsed, "validDate", 0) and
            getattr(parsed, "validTime", 0) and
            getattr(parsed, "fullyResolved", 0)):
        return None
    try:
        # timegm: UTC struct_time → unix seconds (로컬 타임존 영향 없음)
        epoch_sec = calendar.timegm((
            int(parsed.year), int(parsed.month), int(parsed.day),
            int(parsed.hour), int(parsed.min), int(parsed.second),
            0, 0, 0))
    except (ValueError, TypeError):
        return None
    # nano(음수 가능) 반영 후 sec/nanosec 정규화
    total_ns = epoch_sec * 1_000_000_000 + int(getattr(parsed, "nano", 0))
    sec = total_ns // 1_000_000_000
    nanosec = total_ns % 1_000_000_000  # 파이썬 // 는 항상 0<=nanosec<1e9 보장
    return Time(sec=int(sec), nanosec=int(nanosec))


def _i(parsed, name, default=0):
    """정수 필드 안전 추출 (스케일 없는 raw 값)."""
    return int(getattr(parsed, name, default) or 0)


def _b(parsed, name):
    """bool 플래그 안전 추출."""
    return bool(getattr(parsed, name, 0))


def _scaled_to_int(parsed, name, factor):
    """pyubx2 가 스케일 적용해 준 값(예: deg)을 메시지의 정수 필드로 되돌린다.

    예) headMot 는 pyubx2 에서 deg(scale 1e-5 적용됨) → 메시지 head_mot(deg×1e5 정수)로 복원:
        _scaled_to_int(parsed, "headMot", 1e5)
    """
    val = getattr(parsed, name, None)
    if val is None:
        return 0
    return int(round(float(val) * factor))


class BaseRtkMonitor(Node):
    def __init__(self, port, baud):
        super().__init__("base_rtk_monitor")
        self.pub_status = self.create_publisher(String, "/base/rtk_status", 10)
        self.pub_carr = self.create_publisher(UInt8, "/base/carr_soln", 10)
        self.pub_pvt = self.create_publisher(UBXNavPVT, "/base/ubx_nav_pvt", 10)
        # rover 상대위치(NED) 구독 (heading 계산용)
        self.sub_rel = self.create_subscription(
            UBXNavRelPosNED, "/rover/ubx_nav_rel_pos_ned", self.cb_rel, 10)
        # rel_pos 를 itow 별로 버퍼링해 base itow 와 매칭한다. {itow: (n_cm, e_cm)}
        self.rel_buf = deque(maxlen=50)  # (itow_ms, n_cm, e_cm)
        self.port = port
        self.baud = baud
        self.stream = None
        self.reader = None
        self.last_carr = -1
        self.open_stream()  # 최초 개방 (USB 미연결이면 연결될 때까지 재시도)

    def open_stream(self):
        """포트를 (재)개방한다.

        USB를 뽑았다 다시 꽂으면 이전 파일 핸들은 죽어 있으므로 반드시 새 serial.Serial 로 다시 연다.
        열릴 때까지 1초 간격 재시도.
        """
        if self.stream is not None:
            try:
                self.stream.close()
            except Exception:
                pass
            self.stream = None
            self.reader = None
        while rclpy.ok():
            try:
                self.get_logger().info(f"base 포트 열기(read-only): {self.port} @ {self.baud}")
                self.stream = serial.Serial(self.port, self.baud, timeout=0.5)
                self.reader = UBXReader(self.stream, protfilter=2)  # UBX만
                self.get_logger().info("base 포트 열림 → 발행 재개")
                return
            except (serial.SerialException, OSError) as e:
                # 아직 USB가 안 꽂혔거나 재열거 진행 중 → 잠시 대기 후 재시도
                self.get_logger().warn(f"포트 열기 실패, 1s 후 재시도: {e}")
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(1.0)

    def cb_rel(self, msg):
        # 고정밀 NED (cm 단위, hp 는 0.1mm 이므로 *1e-2 cm 환산)
        n_cm = msg.rel_pos_n + msg.rel_pos_hp_n * 1e-2
        e_cm = msg.rel_pos_e + msg.rel_pos_hp_e * 1e-2
        self.rel_buf.append((int(msg.itow), n_cm, e_cm))

    def rel_at(self, itow):
        """base itow(ms)와 같은 itow 의 rel_pos NED 를 반환. 없으면 None."""
        for r_itow, n_cm, e_cm in self.rel_buf:
            if r_itow == itow:
                return (n_cm, e_cm)
        return None

    def run(self):
        while rclpy.ok():
            # rel_pos 구독 콜백 처리 (대기 없이 큐만 비움 → 시리얼 루프 속도 영향 없음)
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                raw, parsed = self.reader.read()
            except (serial.SerialException, OSError) as e:
                # USB 분리/디바이스 사라짐 → 포트 재개방(다시 꽂힐 때까지 대기 후 재개)
                self.get_logger().warn(f"시리얼 연결 끊김 → 재연결 시도: {e}")
                self.open_stream()
                continue
            except Exception:
                # RTCM/UBX 혼재 스트림에서 간헐적 파싱오류 → 무시하고 계속
                continue
            if parsed is None or parsed.identity != "NAV-PVT":
                continue

            cs = int(getattr(parsed, "carrSoln", 0))
            ha = _i(parsed, "hAcc")
            va = _i(parsed, "vAcc")
            nsv = _i(parsed, "numSV")
            ft = _i(parsed, "fixType")
            ds = int(getattr(parsed, "diffSoln", 0))
            label = CARR.get(cs, str(cs))

            self.pub_carr.publish(UInt8(data=cs))
            self.pub_status.publish(String(
                data=f"{label} carrSoln={cs} hAcc={ha}mm vAcc={va}mm "
                     f"sv={nsv} fixType={ft} diffSoln={ds}"))

            lat = getattr(parsed, "lat", None)
            lon = getattr(parsed, "lon", None)
            if lat is None or lon is None:
                continue

            pvt = UBXNavPVT()

            # ── header ───────────────────────────────────────────────────────
            # header.stamp = 데이터 취득 시각(GNSS fix UTC → unix). UTC 미확정 시 현재 시간 fallback.
            acq_time = pvt_utc_to_ros_time(parsed)
            pvt.header.stamp = acq_time if acq_time is not None \
                else self.get_clock().now().to_msg()
            pvt.header.frame_id = "base"

            # ── 시각 / UTC ───────────────────────────────────────────────────
            pvt.itow = _i(parsed, "iTOW")
            pvt.year = _i(parsed, "year")
            pvt.month = _i(parsed, "month")
            pvt.day = _i(parsed, "day")
            pvt.hour = _i(parsed, "hour")
            pvt.min = _i(parsed, "min")
            pvt.sec = _i(parsed, "second")   # uint8=0~59 '분의 초'
            pvt.nano = _i(parsed, "nano")     # 초의 소수 (ns, 음수 가능)
            pvt.t_acc = _i(parsed, "tAcc")
            pvt.valid_date = _b(parsed, "validDate")
            pvt.valid_time = _b(parsed, "validTime")
            pvt.fully_resolved = _b(parsed, "fullyResolved")
            pvt.valid_mag = _b(parsed, "validMag")
            pvt.confirmed_avail = _b(parsed, "confirmedAvai")
            pvt.confirmed_date = _b(parsed, "confirmedDate")
            pvt.confirmed_time = _b(parsed, "confirmedTime")

            # ── fix / RTK 상태 ───────────────────────────────────────────────
            pvt.gps_fix.fix_type = ft
            pvt.gnss_fix_ok = _b(parsed, "gnssFixOk")
            pvt.diff_soln = bool(ds)
            pvt.carr_soln.status = cs
            pvt.psm.state = _i(parsed, "psmState")
            pvt.num_sv = nsv
            pvt.invalid_llh = _b(parsed, "invalidLlh")

            # ── 위치 (pyubx2 scaling=True → lat/lon 은 deg) ───────────────────
            pvt.lat = int(round(float(lat) * 1e7))   # deg → 1e-7
            pvt.lon = int(round(float(lon) * 1e7))   # deg → 1e-7
            pvt.height = _i(parsed, "height")         # mm (타원체고)
            pvt.hmsl = _i(parsed, "hMSL")             # mm (평균해수면고)
            pvt.h_acc = ha                            # mm
            pvt.v_acc = va                            # mm

            # ── 속도 (도플러 기반, 스케일 없음. lat/lon 차분보다 정확) ─────────
            pvt.vel_n = _i(parsed, "velN")            # mm/s (NED 북)
            pvt.vel_e = _i(parsed, "velE")            # mm/s (NED 동)
            pvt.vel_d = _i(parsed, "velD")            # mm/s (NED 하)
            pvt.g_speed = _i(parsed, "gSpeed")        # mm/s (2D 지면속도)
            pvt.head_mot = _scaled_to_int(parsed, "headMot", 1e5)   # deg → deg×1e5
            pvt.s_acc = _i(parsed, "sAcc")            # mm/s
            pvt.head_acc = _scaled_to_int(parsed, "headAcc", 1e5)   # deg → deg×1e5

            # ── DOP / 자기편차 ───────────────────────────────────────────────
            pvt.p_dop = _scaled_to_int(parsed, "pDOP", 100)         # scale 0.01
            pvt.mag_dec = _scaled_to_int(parsed, "magDec", 100)     # scale 0.01
            pvt.mag_acc = _scaled_to_int(parsed, "magAcc", 100)     # scale 0.01

            # ── head_veh: base→rover rel_pos NED 로 계산한 heading 을 싣는다 ──
            # (base 자신의 headVeh 는 센서퓨전 모드가 아니라 의미 없으므로 계산값으로 대체)
            rel = self.rel_at(pvt.itow)
            if rel is not None:
                n_cm, e_cm = rel
                heading = heading_from_ned(n_cm, e_cm)
                pvt.head_veh = int(heading * 1e5)   # deg×1e5
                pvt.head_veh_valid = True
            else:
                # 같은 itow 의 rel_pos 없음(토픽 미수신 또는 itow 불일치) → heading 미발행
                pvt.head_veh = 0
                pvt.head_veh_valid = False

            self.pub_pvt.publish(pvt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args()

    rclpy.init()
    node = BaseRtkMonitor(args.port, args.baud)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.stream is not None:
            node.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
