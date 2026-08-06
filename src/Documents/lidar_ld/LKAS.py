import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from collections import Counter
from geometry_msgs.msg import Point
import json

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # 하나의 토픽 구독
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/merge_201/points',
            self.pointcloud_callback,
            10
        )

        self.filtered_points_pub = self.create_publisher(PointCloud2, '/filter_ld_points', 10)
        self.left_coeff_pub = self.create_publisher(String, '/left_lane_coefficients', 10)
        self.right_coeff_pub = self.create_publisher(String, '/right_lane_coefficients', 10)

        self.lidar_points = None
        self.left_coeffs = None
        self.right_coeffs = None
        self.left_lane_quality = 0
        self.left_lane_type = 0
        self.right_lane_quality = 0
        self.right_lane_type = 0
        self.left_heading_angle = 0.0
        self.right_heading_angle = 0.0
        self.left_normal_distance = 0.0
        self.right_normal_distance = 0.0

        self.create_timer(0.02, self.publish_coefficients)  # 20ms 주기 타이머

    def pointcloud_callback(self, cloud_msg):
        self.lidar_points = self.pointcloud2_to_xyz_intensity_array(cloud_msg)
        self.process_pointcloud_data()

    def process_pointcloud_data(self):
        # LiDAR 포인트 데이터가 수신된 경우 처리
        if self.lidar_points is not None:
            # 바닥 포인트 추출
            ground_points = self.lidar_points[(self.lidar_points[:, 2] >= -2.50) & (self.lidar_points[:, 2] <= 0.0)]
            lane_points = ground_points[ground_points[:, 3] > 50]
            # print(ground_points[:, 3])
            lane_cloud_msg = self.create_point_cloud2(lane_points, 'vehicle')  # vehicle 프레임 사용

            # 필터링된 포인트 퍼블리시
            self.filtered_points_pub.publish(lane_cloud_msg)

            # 차선 검출
            self.detect_lanes(lane_points)

    def detect_lanes(self, lane_points):
        left_y_range = (1.5, 2.0)
        right_y_range = (-2.0, -1.5)
        x_range = (-20, 20)

        left_central_points = self.find_central_points(lane_points, left_y_range, x_range)
        if left_central_points is not None:
            self.left_coeffs = self.fit_quadratic_with_roi_constraints(left_central_points, left_y_range)
            self.left_lane_quality, self.left_lane_type = self.evaluate_lane_quality(self.left_coeffs)
            self.left_heading_angle = self.calculate_heading_angle(self.left_coeffs, 0)
            self.left_normal_distance = self.calculate_normal_distance(self.left_coeffs, 0)

        right_central_points = self.find_central_points(lane_points, right_y_range, x_range)
        if right_central_points is not None:
            self.right_coeffs = self.fit_quadratic_with_roi_constraints(right_central_points, right_y_range)
            self.right_lane_quality, self.right_lane_type = self.evaluate_lane_quality(self.right_coeffs)
            self.right_heading_angle = self.calculate_heading_angle(self.right_coeffs, 0)
            self.right_normal_distance = self.calculate_normal_distance(self.right_coeffs, 0)

    def publish_coefficients(self):
        if self.left_coeffs is not None:
            left_coeff_msg = self.create_coefficient_message(
                self.left_coeffs, self.left_lane_quality, self.left_lane_type,
                self.left_heading_angle, self.left_normal_distance, is_left=True
            )
            self.left_coeff_pub.publish(String(data=left_coeff_msg))

        if self.right_coeffs is not None:
            right_coeff_msg = self.create_coefficient_message(
                self.right_coeffs, self.right_lane_quality, self.right_lane_type,
                self.right_heading_angle, self.right_normal_distance, is_left=False
            )
            self.right_coeff_pub.publish(String(data=right_coeff_msg))

    def pointcloud2_to_xyz_intensity_array(self, cloud_msg):
        points_list = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([point[0], point[1], point[2], point[3]])
        return np.array(points_list)

    def create_point_cloud2(self, points, parent_frame):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.height = 1
        msg.width = points.shape[0]

        # PointField 정의 (각 필드의 이름, 오프셋, 데이터 타입, 개수)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = np.asarray(points, np.float32).tobytes()
        
        return msg

    def create_coefficient_message(self, coeffs, lane_quality, lane_type, heading_angle, normal_distance, is_left=True):
        id_prefix = "Lidar2_Left_Curb_A" if is_left else "Lidar2_Right_Curb_A"
        message = {
            f"{id_prefix}_Curb_Mark_Model_A": coeffs[0],
            f"{id_prefix}_Curb_Mark_Model_dA": 2 * coeffs[0],
            f"{id_prefix}_Curb_Mark_Heading_Angle": heading_angle,
            f"{id_prefix}_Curb_Mark_Quality": lane_quality,
            f"{id_prefix}_Curb_Mark_Type": lane_type,
            f"{id_prefix}_Curb_Mark_Position": normal_distance
        }
        data_strings = [f"{key}:{value}" for key, value in message.items()]
        return ";".join(data_strings)

    def calculate_normal_distance(self, coeffs, x_value=0):
        y_value = np.polyval(coeffs, x_value)
        normal_distance = np.abs(y_value)
        return normal_distance

    def calculate_heading_angle(self, coeffs, x_value):
        slope = 3 * coeffs[0] * x_value**2 + 2 * coeffs[1] * x_value + coeffs[2]
        heading_angle = np.arctan(slope)
        return heading_angle

    def find_central_points(self, points, y_range, x_range, density_threshold=5, y_deviation_threshold=0.1):
        x_values = np.linspace(x_range[0], x_range[1], num=100)
        central_points = []

        filtered_points_in_y_range = points[(points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])]
        if len(filtered_points_in_y_range) == 0:
            return None
        most_common_y = Counter(filtered_points_in_y_range[:, 1]).most_common(1)[0][0]

        for x in x_values:
            filtered_points = points[(points[:, 0] >= x - 0.1) & (points[:, 0] <= x + 0.1) &
                                     (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])]

            if len(filtered_points) >= density_threshold:
                y_mean = np.mean(filtered_points[:, 1])
                if abs(y_mean - most_common_y) <= y_deviation_threshold:
                    central_points.append((x, y_mean))

        if len(central_points) < 2:
            return None

        return np.array(central_points)

    def fit_quadratic_with_roi_constraints(self, points, y_range):
        X = points[:, 0]
        y = points[:, 1]
        coeffs = np.polyfit(X, y, 2)
        y_fit = np.polyval(coeffs, X)
        y_fit = np.clip(y_fit, y_range[0], y_range[1])
        coeffs = np.polyfit(X, y_fit, 2)
        return coeffs

    def evaluate_lane_quality(self, coeffs, curvature_threshold=0.1):
        curvature = 2 * abs(coeffs[0])
        if curvature > curvature_threshold:
            return 0, 0  # Low quality, not a valid lane
        else:
            return 3, 1  # High quality, valid lane


def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
