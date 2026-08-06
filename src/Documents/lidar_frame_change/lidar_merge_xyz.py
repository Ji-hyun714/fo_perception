import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')
        
        # 여러 LiDAR로부터 토픽을 구독
        self.lidar1_sub = Subscriber(self, PointCloud2, '/TFnonground/lidar_11')
        self.lidar2_sub = Subscriber(self, PointCloud2, '/TFnonground/lidar_12')
        self.lidar3_sub = Subscriber(self, PointCloud2, '/TFnonground/lidar_13')

        # 메시지 동기화
        self.sync = ApproximateTimeSynchronizer([self.lidar1_sub, self.lidar2_sub, self.lidar3_sub], 10, 0.1)
        self.sync.registerCallback(self.merge_callback)

        # 병합된 포인트 클라우드를 발행할 퍼블리셔
        self.pub = self.create_publisher(PointCloud2, '/TFnonground/merged_points', 10)

        self.get_logger().info("Merge LiDAR points")


    def merge_callback(self, lidar1_msg, lidar2_msg, lidar3_msg):
        
        # 각 LiDAR의 메시지의 frame_id를 vehicle로 명시적으로 설정
        # lidar1_msg.header.frame_id = 'vehicle'
        # lidar2_msg.header.frame_id = 'vehicle'
        # lidar3_msg.header.frame_id = 'vehicle'

        # 각 포인트 클라우드를 병합
        merged_cloud = self.merge_pointclouds(lidar1_msg, lidar2_msg, lidar3_msg)
        self.pub.publish(merged_cloud)

    def merge_pointclouds(self, *pointclouds):
        merged_points = []

        for pc in pointclouds:
            # 각 PointCloud2 메시지에서 포인트 추출
            # points = list(point_cloud2.read_points(pc, field_names=("x", "y", "z", "intensity"),skip_nans=True))
            points = list(point_cloud2.read_points(pc, field_names=("x", "y", "z"),skip_nans=True))
            merged_points.extend(points)

        # 새로운 PointCloud2 메시지로 변환하여 반환
        merged_cloud_msg = point_cloud2.create_cloud(pointclouds[0].header, pointclouds[0].fields, merged_points)
        # merged_cloud_msg.header.frame_id = 'vehicle'
        return merged_cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
