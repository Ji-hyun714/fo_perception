#!/usr/bin/env python3
"""/base/ubx_nav_pvt 에 이미 들어있는 속도(vel_n/vel_e)로 속도를 계산·발행하는 노드.

cal_velocity.py 는 lat/lon 을 차분(differentiation)해 속도를 추정하지만,
이 노드는 수신기가 도플러(Doppler)로 계산해 NAV-PVT 에 담아 준 vel_n/vel_e 를 그대로 쓴다.
  → 차분보다 정확하고(수 cm/s), 저지연(단일 epoch), Δt/래핑 처리 불필요.

메시지의 vel_n/vel_e/vel_d 는 mm/s (NED 북/동/하). x=북, y=동 으로 매핑한다.
  x속도 = vel_n,  y속도 = vel_e,  합속도 = hypot(vel_n, vel_e)

토픽:
  구독:
    /base/ubx_nav_pvt (ublox_ubx_msgs/UBXNavPVT)
  발행 (둘 다 std_msgs/Float32MultiArray, data = [합속도, x속도, y속도]):
    /cal_velocity_km_h : [xy합 속도, x(북) 속도, y(동) 속도]  단위 km/h
    /cal_velocity_m_s  : [xy합 속도, x(북) 속도, y(동) 속도]  단위 m/s
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ublox_ubx_msgs.msg import UBXNavPVT


class VelocityFromPvt(Node):
    def __init__(self):
        super().__init__("cal_velocity2")
        self.pub_kmh = self.create_publisher(Float32MultiArray, "/cal_velocity_km_h", 10)
        self.pub_ms = self.create_publisher(Float32MultiArray, "/cal_velocity_m_s", 10)
        self.sub = self.create_subscription(
            UBXNavPVT, "/base/ubx_nav_pvt", self.cb_pvt, 10)
        self.get_logger().info(
            "cal_velocity2 시작: /base/ubx_nav_pvt vel_n/vel_e(도플러) → 속도 발행")

    def cb_pvt(self, msg):
        # NAV-PVT 속도 필드는 mm/s → m/s
        vel_n = msg.vel_n * 1e-3   # x (북)
        vel_e = msg.vel_e * 1e-3   # y (동)
        speed = math.hypot(vel_n, vel_e)   # xy 합 속도 [m/s]

        # data = [xy합 속도, x(북) 속도, y(동) 속도]
        self.pub_ms.publish(Float32MultiArray(
            data=[float(speed), float(vel_n), float(vel_e)]))
        self.pub_kmh.publish(Float32MultiArray(
            data=[float(speed * 3.6), float(vel_n * 3.6), float(vel_e * 3.6)]))

        self.get_logger().info(
            f"speed={speed:6.3f} m/s ({speed * 3.6:6.2f} km/h)  "
            f"vel_n={vel_n:+.3f} vel_e={vel_e:+.3f}",
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = VelocityFromPvt()
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
