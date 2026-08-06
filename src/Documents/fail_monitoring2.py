import subprocess
import time
import threading
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

# LiDAR IP
# lidar_ips = ['192.168.11.201', '192.168.12.201', '192.168.13.201']
lidar_ips = ['192.168.11.201', '192.168.11.202', '192.168.11.203']

running = True


def is_lidar_alive(ip):
    try:
        result = subprocess.run(
            ['ping', '-c', '1', '-w', '1', ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except Exception:
        return False


class LidarMonitorNode(Node):

    def __init__(self):
        super().__init__('lidar_monitor_node')

        self.get_logger().info("LidarMonitorNode started.")

        self.publisher_ = self.create_publisher(UInt8MultiArray, '/lidar2_general', 10)

        self.fail_flags = [0, 0, 0]
        self.alive_counters = [0, 0, 0]
        self.failure_states = [0, 0, 0]

        self.thread_ = threading.Thread(target=self.monitor_loop)
        self.thread_.start()

    def monitor_loop(self):
        global running

        while running:
            for i, ip in enumerate(lidar_ips):
                alive = is_lidar_alive(ip)

                if alive:
                    self.fail_flags[i] = 0
                    self.failure_states[i] = 0
                    self.alive_counters[i] = (self.alive_counters[i] + 1) % 128
                else:
                    self.fail_flags[i] = 1
                    self.failure_states[i] = 15

            # 메시지 생성
            msg = UInt8MultiArray()

            data = []
            for i in range(3):
                data.extend([
                    self.fail_flags[i],
                    self.alive_counters[i],
                    self.failure_states[i]
                ])

            msg.data = data

            self.publisher_.publish(msg)

            # print(
            #     f"L1: F{self.fail_flags[0]} A{self.alive_counters[0]} | "
            #     f"L2: F{self.fail_flags[1]} A{self.alive_counters[1]} | "
            #     f"L3: F{self.fail_flags[2]} A{self.alive_counters[2]}"
            # )

            time.sleep(0.01)

    def destroy_node(self):
        global running
        running = False
        self.thread_.join()
        super().destroy_node()


def signal_handler(sig, frame):
    rclpy.shutdown()


signal.signal(signal.SIGINT, signal_handler)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
