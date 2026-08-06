#!/usr/bin/env python3
"""base/rover 의 NAV-PVT lat/lon 으로 heading 을 계산해 /cal_heading 로 발행.

/base/ubx_nav_pvt 와 /rover/ubx_nav_pvt 의 lat/lon 을 itow 로 매칭해
base→rover 방위각(위·경도 기반 great-circle bearing)을 구한다.
rel_pos_ned(NED 벡터) 대신 두 PVT 의 절대위치만 사용하는 대비용 방식이다.

구독:
  /base/ubx_nav_pvt   (ublox_ubx_msgs/UBXNavPVT)  : base 절대위치 (lat/lon raw 1e-7)
  /rover/ubx_nav_pvt  (ublox_ubx_msgs/UBXNavPVT)  : rover 절대위치 (lat/lon raw 1e-7)

발행:
  /cal_heading  (std_msgs/Int32)  : base→rover 방위각 (deg*1e5, 북=0, 시계방향, 0~360)

heading 계산:
  base itow 와 같은 itow 의 rover 위치가 있을 때만 compute_heading 으로 계산해 발행한다.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ublox_ubx_msgs.msg import UBXNavPVT


def compute_heading(lat1, lon1, lat2, lon2):
    """base(lat1,lon1) → rover(lat2,lon2) 방위각(deg, 0~360, 북=0, 시계방향)."""
    if None in (lat1, lon1, lat2, lon2):
        return None
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


class CalculateHeading(Node):
    def __init__(self):
        super().__init__("calculate_heading")
        self.pub = self.create_publisher(Int32, "/cal_heading", 10)
        self.create_subscription(
            UBXNavPVT, "/base/ubx_nav_pvt", self.cb_base, 10)
        self.create_subscription(
            UBXNavPVT, "/rover/ubx_nav_pvt", self.cb_rover, 10)
        # base/rover 위치를 itow 별로 버퍼링한다. 어느 토픽이 먼저 도착할지 모르므로
        # 두 콜백 모두에서 상대편 버퍼를 조회해 itow 가 매칭되면 계산한다. {itow: (lat, lon)}
        self.base_buf = deque(maxlen=50)   # (itow_ms, lat, lon)
        self.rover_buf = deque(maxlen=50)  # (itow_ms, lat, lon)
        self.last_itow = None              # 중복 발행 방지 (같은 itow 재계산 방지)
        self.get_logger().info(
            "calculate_heading 시작: base/rover PVT lat/lon 으로 /cal_heading 발행")

    @staticmethod
    def _lookup(buf, itow):
        """buf 에서 itow 가 같은 (lat, lon) 을 반환. 없으면 None."""
        for b_itow, lat, lon in buf:
            if b_itow == itow:
                return (lat, lon)
        return None

    def cb_base(self, msg):
        itow = int(msg.itow)
        self.base_buf.append((itow, msg.lat * 1e-7, msg.lon * 1e-7))
        self.try_publish(itow)

    def cb_rover(self, msg):
        itow = int(msg.itow)
        self.rover_buf.append((itow, msg.lat * 1e-7, msg.lon * 1e-7))
        self.try_publish(itow)

    def try_publish(self, itow):
        """같은 itow 의 base/rover 가 모두 도착하면 heading 을 계산해 발행한다."""
        if itow == self.last_itow:
            return  # 같은 itow 는 한 번만 발행
        base = self._lookup(self.base_buf, itow)
        rover = self._lookup(self.rover_buf, itow)
        if base is None or rover is None:
            # 아직 한쪽만 도착 → 상대편 콜백에서 다시 시도
            return

        base_lat, base_lon = base
        rover_lat, rover_lon = rover
        heading = compute_heading(base_lat, base_lon, rover_lat, rover_lon)
        if heading is None:
            return

        self.last_itow = itow
        # deg×1e5 로 스케일 후 정수만 발행 (UBX rel_pos_heading)
        self.pub.publish(Int32(data=int(heading * 1e5)))
        # self.get_logger().info(
        #     f"heading={heading:.2f}deg (itow={itow})", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = CalculateHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
