#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_stream')
    
    fps = 25.0
    timeout = 150

    # ===== Camera Configurations =====
    cameras = [
        # {'id': '1', 'host': '192.168.0.197', 'yaml': 'cam_197.yaml'},  # 필요시 주석 해제
        # {'id': '2', 'host': '192.168.0.101', 'yaml': 'cam_101.yaml'},
        {'id': '3', 'host': '192.168.0.106', 'yaml': 'cam_106.yaml'},
        # {'id': '4', 'host': '192.168.0.122', 'yaml': 'cam_122.yaml'},
    ]

    nodes = []
    for cam in cameras:
        cam_yaml = PathJoinSubstitution([pkg_share, 'yaml', cam['yaml']])
        namespace = f"cam{cam['id']}"
        
        # GStreamer Publisher Node
        nodes.append(Node(
            package='camera_stream',
            executable='gst_ipcam_node',
            name='gst_ipcam',
            namespace=namespace,
            output='screen',
            parameters=[{
                'yaml_path': cam_yaml,
                'frame_id': f"camera_{cam['id']}",
                'image_topic': 'image_raw',  # → /camX/image_raw
                'status_topic': 'camera_general',
                'timeout_ms': timeout,
                'expected_fps': fps,
                'host': cam['host'],
            }]
        ))
        
        # Rectifier Node
        nodes.append(Node(
            package='camera_stream',
            executable='rectify_node',
            name='undistorted',
            # namespace=namespace,
            output='screen',
            parameters=[{
                'yaml_path': cam_yaml,
                'input_topic': f"cam{cam['id']}/image_raw",
                'output_topic': 'image_rect',
                'image_width': 1280,
                'image_height': 720,
                'debug_mode': False,
                'fisheye_balance': 0.0,
            }]
        ))

    return LaunchDescription(nodes)