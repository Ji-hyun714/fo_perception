#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class RingGroundRemovalNode(Node):
    def __init__(self):
        super().__init__('ring_ground_removal_lidar_11')

        # -----------------------------
        # Topics
        # -----------------------------
        self.declare_parameter('input_topic', '/lidar_11_201/points')
        self.declare_parameter('ground_topic', '/ring_ground/lidar_11_201')
        self.declare_parameter('nonground_topic', '/ring_nonground/lidar_11_201')

        # -----------------------------
        # Range filter
        # min_range는 0.0 유지 권장
        # 4.5m 이내를 처리하고 싶으면 min_range를 키우면 안 됨
        # -----------------------------
        self.declare_parameter('min_range', 0.0)
        self.declare_parameter('max_range', 80.0)

        # -----------------------------
        # Ground z candidate
        # -----------------------------
        self.declare_parameter('ground_z_min', -5.0)
        self.declare_parameter('ground_z_max', -0.2)

        # -----------------------------
        # Ring neighbor condition
        # -----------------------------
        self.declare_parameter('max_neighbor_z_diff', 0.25)
        self.declare_parameter('max_slope_deg', 12.0)
        self.declare_parameter('max_neighbor_xy_dist', 1.0)

        # -----------------------------
        # Near low-z ground override
        # range <= near_low_z_range 이고
        # z < near_low_z_threshold 이면 ground로 강제 처리
        # -----------------------------
        self.declare_parameter('use_near_low_z_ground', True)
        self.declare_parameter('near_low_z_range', 4.5)
        self.declare_parameter('near_low_z_threshold', -0.4)

        # -----------------------------
        # Ring field option
        # -----------------------------
        self.declare_parameter('allow_no_ring', False)

        # -----------------------------
        # Load parameters
        # -----------------------------
        self.input_topic = self.get_parameter('input_topic').value
        self.ground_topic = self.get_parameter('ground_topic').value
        self.nonground_topic = self.get_parameter('nonground_topic').value

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.ground_z_min = float(self.get_parameter('ground_z_min').value)
        self.ground_z_max = float(self.get_parameter('ground_z_max').value)

        self.max_neighbor_z_diff = float(
            self.get_parameter('max_neighbor_z_diff').value
        )
        self.max_slope_deg = float(
            self.get_parameter('max_slope_deg').value
        )
        self.max_neighbor_xy_dist = float(
            self.get_parameter('max_neighbor_xy_dist').value
        )

        self.max_slope_rad = math.radians(self.max_slope_deg)

        self.use_near_low_z_ground = bool(
            self.get_parameter('use_near_low_z_ground').value
        )
        self.near_low_z_range = float(
            self.get_parameter('near_low_z_range').value
        )
        self.near_low_z_threshold = float(
            self.get_parameter('near_low_z_threshold').value
        )

        self.allow_no_ring = bool(
            self.get_parameter('allow_no_ring').value
        )

        # 현재 코드 스타일 유지: qos = 1
        qos = 1

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        self.ground_pub = self.create_publisher(
            PointCloud2,
            self.ground_topic,
            qos
        )

        self.nonground_pub = self.create_publisher(
            PointCloud2,
            self.nonground_topic,
            qos
        )

        self.get_logger().info('Ring Ground Removal Node started')
        self.get_logger().info(f'Subscribe : {self.input_topic}')
        self.get_logger().info(f'Ground    : {self.ground_topic}')
        self.get_logger().info(f'NonGround : {self.nonground_topic}')

        self.get_logger().info(
            f'near_low_z_ground={self.use_near_low_z_ground}, '
            f'range<={self.near_low_z_range}, '
            f'z<{self.near_low_z_threshold}'
        )

    def cloud_callback(self, msg: PointCloud2):
        field_names = [field.name for field in msg.fields]

        if 'ring' not in field_names:
            self.get_logger().warn('PointCloud2에 ring 필드가 없습니다.')

            if not self.allow_no_ring:
                self.publish_empty(msg.header)
                print("no ring!")
                return

        points = self.pointcloud2_to_xyzir_array(msg)

        if points.shape[0] == 0:
            self.publish_empty(msg.header)
            return

        xyz = points[:, 0:3]
        ranges = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 1] ** 2)

        valid_mask = (
            np.isfinite(xyz[:, 0]) &
            np.isfinite(xyz[:, 1]) &
            np.isfinite(xyz[:, 2]) &
            (ranges >= self.min_range) &
            (ranges <= self.max_range)
        )

        valid_points = points[valid_mask]

        if valid_points.shape[0] == 0:
            self.publish_empty(msg.header)
            return

        ground_mask_valid = self.classify_ground_by_ring(valid_points)

        ground_points = valid_points[ground_mask_valid]
        nonground_points = valid_points[~ground_mask_valid]

        ground_msg = self.xyzir_array_to_pointcloud2(
            ground_points,
            msg.header
        )
        nonground_msg = self.xyzir_array_to_pointcloud2(
            nonground_points,
            msg.header
        )

        self.ground_pub.publish(ground_msg)
        self.nonground_pub.publish(nonground_msg)

    def pointcloud2_to_xyzir_array(self, msg: PointCloud2) -> np.ndarray:
        """
        PointCloud2에서 x, y, z, intensity, ring을 읽어서 Nx5 배열로 변환.
        Output:
            [x, y, z, intensity, ring]
        """
        field_names = [field.name for field in msg.fields]

        has_intensity = 'intensity' in field_names
        has_ring = 'ring' in field_names

        points = []

        if has_intensity and has_ring:
            read_fields = ('x', 'y', 'z', 'intensity', 'ring')

            for p in pc2.read_points(
                msg,
                field_names=read_fields,
                skip_nans=False
            ):
                x, y, z, intensity, ring = p

                if not self.is_finite_xyz(x, y, z):
                    continue

                if intensity is None or not math.isfinite(float(intensity)):
                    intensity = 0.0

                points.append([
                    float(x),
                    float(y),
                    float(z),
                    float(intensity),
                    float(ring)
                ])

        elif has_intensity and not has_ring:
            read_fields = ('x', 'y', 'z', 'intensity')

            for p in pc2.read_points(
                msg,
                field_names=read_fields,
                skip_nans=False
            ):
                x, y, z, intensity = p

                if not self.is_finite_xyz(x, y, z):
                    continue

                if intensity is None or not math.isfinite(float(intensity)):
                    intensity = 0.0

                points.append([
                    float(x),
                    float(y),
                    float(z),
                    float(intensity),
                    0.0
                ])

        elif not has_intensity and has_ring:
            read_fields = ('x', 'y', 'z', 'ring')

            for p in pc2.read_points(
                msg,
                field_names=read_fields,
                skip_nans=False
            ):
                x, y, z, ring = p

                if not self.is_finite_xyz(x, y, z):
                    continue

                points.append([
                    float(x),
                    float(y),
                    float(z),
                    0.0,
                    float(ring)
                ])

        else:
            read_fields = ('x', 'y', 'z')

            for p in pc2.read_points(
                msg,
                field_names=read_fields,
                skip_nans=False
            ):
                x, y, z = p

                if not self.is_finite_xyz(x, y, z):
                    continue

                points.append([
                    float(x),
                    float(y),
                    float(z),
                    0.0,
                    0.0
                ])

        if len(points) == 0:
            return np.empty((0, 5), dtype=np.float32)

        return np.asarray(points, dtype=np.float32)

    def classify_ground_by_ring(self, points: np.ndarray) -> np.ndarray:
        """
        points: Nx5 [x, y, z, intensity, ring]

        처리 순서:
            1. 같은 ring끼리 분리
            2. 각 ring 내부에서 azimuth 기준 정렬
            3. range <= near_low_z_range and z < near_low_z_threshold 이면 ground
            4. 나머지는 기존 ring neighbor 기반으로 ground/nonground 판단
        """
        num_points = points.shape[0]
        ground_mask = np.zeros(num_points, dtype=bool)

        rings = points[:, 4].astype(np.int32)
        unique_rings = np.unique(rings)

        for ring in unique_rings:
            ring_indices = np.where(rings == ring)[0]

            if ring_indices.shape[0] < 3:
                continue

            ring_points = points[ring_indices]

            x = ring_points[:, 0]
            y = ring_points[:, 1]

            azimuth = np.arctan2(y, x)

            sorted_order = np.argsort(azimuth)
            sorted_indices = ring_indices[sorted_order]
            sorted_points = points[sorted_indices]

            local_ground = np.zeros(sorted_points.shape[0], dtype=bool)

            for i in range(sorted_points.shape[0]):
                curr = sorted_points[i]

                curr_x = float(curr[0])
                curr_y = float(curr[1])
                curr_z = float(curr[2])

                curr_range = math.sqrt(
                    curr_x * curr_x +
                    curr_y * curr_y
                )

                # ----------------------------------------------------
                # 1. 근거리 낮은 z 포인트 강제 ground 처리
                #
                # 의도:
                #   range <= 4.5m and z < -0.4  → ground
                #   그 외 포인트                  → 기존 ring 판단으로 이동
                # ----------------------------------------------------
                if self.use_near_low_z_ground:
                    if (
                        curr_range <= self.near_low_z_range and
                        curr_z < self.near_low_z_threshold
                    ):
                        local_ground[i] = True
                        continue

                # ----------------------------------------------------
                # 2. 절대 z 기준
                # ----------------------------------------------------
                z_candidate = (
                    self.ground_z_min <= curr_z <= self.ground_z_max
                )

                if not z_candidate:
                    local_ground[i] = False
                    continue

                # ----------------------------------------------------
                # 3. 같은 ring의 이전/다음 포인트와 비교
                # ----------------------------------------------------
                neighbor_ground_count = 0
                neighbor_count = 0

                for offset in [-1, 1]:
                    j = i + offset

                    if j < 0 or j >= sorted_points.shape[0]:
                        continue

                    neighbor = sorted_points[j]

                    nx = float(neighbor[0])
                    ny = float(neighbor[1])
                    nz = float(neighbor[2])

                    dx = curr_x - nx
                    dy = curr_y - ny
                    dz = curr_z - nz

                    xy_dist = math.sqrt(dx * dx + dy * dy)

                    if xy_dist < 1e-6:
                        continue

                    if xy_dist > self.max_neighbor_xy_dist:
                        continue

                    slope = abs(math.atan2(dz, xy_dist))
                    z_diff = abs(dz)

                    neighbor_count += 1

                    if (
                        z_diff <= self.max_neighbor_z_diff and
                        slope <= self.max_slope_rad
                    ):
                        neighbor_ground_count += 1

                # ----------------------------------------------------
                # 4. 최종 ground 판단
                # ----------------------------------------------------
                if neighbor_count > 0 and neighbor_ground_count > 0:
                    local_ground[i] = True
                else:
                    local_ground[i] = False

            ground_mask[sorted_indices] = local_ground

        return ground_mask

    def xyzir_array_to_pointcloud2(
        self,
        points: np.ndarray,
        header
    ) -> PointCloud2:
        """
        Nx5 [x, y, z, intensity, ring] 배열을 PointCloud2로 변환.
        intensity, ring 유지.
        """
        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='ring',
                offset=16,
                datatype=PointField.UINT16,
                count=1
            ),
        ]

        cloud_data = []

        for p in points:
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])
            intensity = float(p[3])
            ring = int(p[4])

            cloud_data.append([x, y, z, intensity, ring])

        return pc2.create_cloud(header, fields, cloud_data)

    def publish_empty(self, header):
        empty = np.empty((0, 5), dtype=np.float32)

        ground_msg = self.xyzir_array_to_pointcloud2(empty, header)
        nonground_msg = self.xyzir_array_to_pointcloud2(empty, header)

        self.ground_pub.publish(ground_msg)
        self.nonground_pub.publish(nonground_msg)

    @staticmethod
    def is_finite_xyz(x, y, z) -> bool:
        return (
            math.isfinite(float(x)) and
            math.isfinite(float(y)) and
            math.isfinite(float(z))
        )


def main(args=None):
    rclpy.init(args=args)

    node = RingGroundRemovalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()