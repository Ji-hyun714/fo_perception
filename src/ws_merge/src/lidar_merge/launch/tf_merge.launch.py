from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR 11 변환 노드
        Node(
            package='lidar_merge',
            executable='tf_11',
            name='tf_11',
            output='screen',
            emulate_tty=True
        ),

        # LiDAR 12 변환 노드
        Node(
            package='lidar_merge',
            executable='tf_12',
            name='tf_12',
            output='screen',
            emulate_tty=True
        ),

        # LiDAR 13 변환 노드
        Node(
            package='lidar_merge',
            executable='tf_13',
            name='tf_13',
            output='screen',
            emulate_tty=True
        ),

        # LiDAR 병합 노드
        Node(
            package='lidar_merge',
            executable='merge',
            name='lidar_merge_node',
            output='screen',
            emulate_tty=True
        ),
    ])
