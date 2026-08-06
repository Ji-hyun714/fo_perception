from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_can',
            executable='RadarNodeC',
            name='radar_c_node',
            output='both',
        ),
        Node(
            package='radar_can',
            executable='RadarNodeF',
            name='radar_f_node',
            output='both',
        ),
        Node(
            package='radar_can',
            executable='RadarNodeR',
            name='radar_r_node',
            output='both',
        ),
    ])