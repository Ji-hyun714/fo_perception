import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('velo16_ground')

    lidar11_config = os.path.join(pkg_share, 'config', 'lidar_11.yaml')
    lidar12_config = os.path.join(pkg_share, 'config', 'lidar_12.yaml')
    lidar13_config = os.path.join(pkg_share, 'config', 'lidar_13.yaml')

    lidar11_node = Node(
        package='velo16_ground',
        executable='ground_removal_node',
        name='ground_removal_lidar_11',
        output='screen',
        parameters=[lidar11_config]
    )

    lidar12_node = Node(
        package='velo16_ground',
        executable='ground_removal_node',
        name='ground_removal_lidar_12',
        output='screen',
        parameters=[lidar12_config]
    )

    lidar13_node = Node(
        package='velo16_ground',
        executable='ground_removal_node',
        name='ground_removal_lidar_13',
        output='screen',
        parameters=[lidar13_config]
    )

    return LaunchDescription([
        lidar11_node,
        lidar12_node,
        lidar13_node,
    ])