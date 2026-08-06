import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from ublox_ubx_msgs.msg import UBXNavPVT

GNSS_TOPIC   = '/base/ubx_nav_pvt'
STATUS_TOPIC = '/gnss_general'

DATA_DELAY   = 0.2   # /base/ubx_nav_pvt 9~10hz (0.1~0.11초에 1번씩 들어옴)
DATA_TIMEOUT = 0.3
PUBLISH_HZ   = 20.0  # 상태 발행 주기 (20hz)


class GnssMonitorNode(Node):

    def __init__(self):
        super().__init__('gnss_monitor_node')

        # 시작은 fail 상태 (아직 데이터 없음)
        self.fail_flag = 1
        self.alive_counter = 0
        self.failure_state = 15
        self.last_rx = None          # 마지막 PVT 수신 시각

        self.create_subscription(UBXNavPVT, GNSS_TOPIC, self.on_pvt, 10)
        self.pub = self.create_publisher(UInt8MultiArray, STATUS_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)

        self.get_logger().info(
            f"GnssMonitorNode started. sub={GNSS_TOPIC} pub={STATUS_TOPIC}")

    def on_pvt(self, msg):
        # 판정은 하지 않고 수신 시각만 기록 (데이터 유무만 판정)
        self.last_rx = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        elapsed = (float('inf') if self.last_rx is None
                   else (now - self.last_rx).nanoseconds / 1e9)

        if elapsed < DATA_DELAY:
            # 정상
            self.fail_flag = 0
            self.failure_state = 0
            self.alive_counter = (self.alive_counter + 1) % 128
        elif elapsed < DATA_TIMEOUT:
            # delay: 0.3초 ~ 0.49초
            self.fail_flag = 1
            self.failure_state = 1
            self.alive_counter = (self.alive_counter + 1) % 128
        else:
            # fail: 0.5초 ~
            self.fail_flag = 1
            self.failure_state = 15
            self.alive_counter = (self.alive_counter + 1) % 128

        msg = UInt8MultiArray()
        msg.data = [self.fail_flag, self.alive_counter, self.failure_state]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GnssMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
