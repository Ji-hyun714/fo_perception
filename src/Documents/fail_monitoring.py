import os
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# LiDAR IP 설정
lidar_ips = ['192.168.11.201', '192.168.12.201', '192.168.13.201']

# 전역 변수 초기화
alive_counters = [0, 0, 0]
fail_flags = [0, 0, 0]
running = True

# LiDAR에 Ping을 보내 응답 확인
#def is_lidar_alive(ip):
#    response = os.system(f"ping -c 1 -w 1 {ip} > /dev/null 2>&1")
#    return response == 0

# subprocess를 이용한 ping 함수
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



# ROS2 노드 정의
class LidarMonitorNode(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        
        # /lidar2_general Publisher 설정
        self.general_publisher = self.create_publisher(String, '/lidar2_general', 2)
        
        # 스레드 시작
        self.threads = []
        for i in range(3):
            t = threading.Thread(target=self.monitor_lidar, args=(i,))
            self.threads.append(t)
            t.start()

    def monitor_lidar(self, index):
        global alive_counters, fail_flags, running
        while running:
            if is_lidar_alive(lidar_ips[index]):
                fail_flags[index] = 0
                alive_counters[index] = (alive_counters[index] + 1) % 128  # 0~127로 범위 제한
            else:
                fail_flags[index] = 1
            
            # 각 값을 3자리로 고정하여 문자열로 변환
            general_msg = String()
            # general_msg.data = (
            #     f"{alive_counters[0]:03}{fail_flags[0]:03}"
            #     f"{alive_counters[1]:03}{fail_flags[1]:03}"
            #     f"{alive_counters[2]:03}{fail_flags[2]:03}"
            # )
            general_msg.data = (
                f"{fail_flags[0]:03}{alive_counters[0]:03}"
                f"{fail_flags[1]:03}{alive_counters[1]:03}"
                f"{fail_flags[2]:03}{alive_counters[2]:03}"
            )
            # print("alive_counters[0] : ",alive_counters[0])
            # print("alive_counters[1] : ",alive_counters[1])
            # print("alive_counters[2] : ",alive_counters[2])

            self.general_publisher.publish(general_msg)

            # 상태 출력
            # self.get_logger().info(
            print(
                f"Lidar 1: Alive = {alive_counters[0]}, Fail = {fail_flags[0]}, "
                f"Lidar 2: Alive = {alive_counters[1]}, Fail = {fail_flags[1]}, "
                f"Lidar 3: Alive = {alive_counters[2]}, Fail = {fail_flags[2]}"
            )
            
            # 10ms 대기
            time.sleep(0.01)
    
    def destroy_node(self):
        global running
        running = False
        for t in self.threads:
            t.join()
        super().destroy_node()

# Ctrl+C 시그널 핸들러
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
        print("Program ended.")

if __name__ == '__main__':
    main()
