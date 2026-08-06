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

    # ===== Camera Configuration Paths (Relative to package) =====
    cam_configs = {
        'cam1': PathJoinSubstitution([pkg_share, 'yaml', 'cam_197.yaml']),
        'cam2': PathJoinSubstitution([pkg_share, 'yaml', 'cam_101.yaml']),
        'cam3': PathJoinSubstitution([pkg_share, 'yaml', 'cam_106.yaml']),
        'cam4': PathJoinSubstitution([pkg_share, 'yaml', 'cam_122.yaml']),
    }
    return LaunchDescription([

        # ===== Global parameters (if needed) =====
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),

        # # ===== Camera 1 =====
        # DeclareLaunchArgument(
        #     'cam1_yaml_path',
        #     default_value=cam_configs['cam1'],
        #     description='YAML config for front camera'
        # ),
        # DeclareLaunchArgument('cam1_frame_id', default_value='camera1'),
        # DeclareLaunchArgument('cam1_image_topic', default_value='/cam1/image_raw'),
        # DeclareLaunchArgument('cam1_status_topic', default_value='/cam1/camera_general'),

        # Node(
        #     package='camera_stream',
        #     executable='gst_ipcam_node',
        #     name='gst_ipcam_1',  # Unique node name
        #     # namespace='camera/front', 
        #     output='screen',
        #     parameters=[{
        #         'yaml_path': LaunchConfiguration('cam1_yaml_path'),
        #         'image_topic': LaunchConfiguration('cam1_image_topic'),
        #         'status_topic' : LaunchConfiguration('cam1_status_topic'),
        #         'frame_id': LaunchConfiguration('cam1_frame_id'),
        #         'timeout_ms': timeout,
        #         'expected_fps': fps,
        #         'host': "192.168.0.197",
        #     }]
        # ),

        # # ===== Camera 2 =====
        # DeclareLaunchArgument(
        #     'cam2_yaml_path',
        #     default_value=cam_configs['cam2'],
        #     description='YAML config for front camera'
        # ),
        # DeclareLaunchArgument('cam2_frame_id', default_value='camera2'),
        # DeclareLaunchArgument('cam2_image_topic', default_value='/cam2/image_raw'),
        # DeclareLaunchArgument('cam2_status_topic', default_value='/cam2/camera_general'),

        # Node(
        #     package='camera_stream',
        #     executable='gst_ipcam_node',
        #     name='gst_ipcam_2',  
        #     # namespace='camera/front',  
        #     output='screen',
        #     parameters=[{
        #         'yaml_path': LaunchConfiguration('cam2_yaml_path'),
        #         'frame_id': LaunchConfiguration('cam2_frame_id'),
        #         'image_topic': LaunchConfiguration('cam2_image_topic'),
        #         'status_topic' : LaunchConfiguration('cam2_status_topic'),
        #         'timeout_ms': timeout,
        #         'expected_fps': fps,
        #         'host': "192.168.0.101",
        #     }]
        # ),

        # ===== Camera 3 =====
        DeclareLaunchArgument(
            'cam3_yaml_path',
            default_value=cam_configs['cam3'],
            description='YAML config for front camera'
        ),
        DeclareLaunchArgument('cam3_frame_id', default_value='camera3'),
        DeclareLaunchArgument('cam3_image_topic', default_value='/cam3/image_raw'),
        DeclareLaunchArgument('cam3_status_topic', default_value='/cam3/camera_general'),

        Node(
            package='camera_stream',
            executable='gst_ipcam_node',
            name='gst_ipcam_3',  
            # namespace='camera/front',  
            output='screen',
            parameters=[{
                'yaml_path': LaunchConfiguration('cam3_yaml_path'),
                'frame_id': LaunchConfiguration('cam3_frame_id'),
                'image_topic': LaunchConfiguration('cam3_image_topic'),
                'status_topic' : LaunchConfiguration('cam3_status_topic'),
                'timeout_ms': timeout,
                'expected_fps': fps,
                'host': "192.168.0.106",

            }]
        ),

        # # ===== Camera 4 ===== 
        # DeclareLaunchArgument(
        #     'cam4_yaml_path',
        #     default_value=cam_configs['cam4'],
        #     description='YAML config for front camera'
        # ),
        # DeclareLaunchArgument('cam4_frame_id', default_value='cam4'),
        # DeclareLaunchArgument('cam4_image_topic', default_value='/cam4/image_raw'),
        # DeclareLaunchArgument('cam4_status_topic', default_value='/cam4/camera_general'),

        # Node(
        #     package='camera_stream',
        #     executable='gst_ipcam_node',
        #     name='gst_ipcam_4',  
        #     # namespace='camera/front',  
        #     output='screen',
        #     parameters=[{
        #         'yaml_path': LaunchConfiguration('cam4_yaml_path'),
        #         'frame_id': LaunchConfiguration('cam4_frame_id'),
        #         'image_topic': LaunchConfiguration('cam4_image_topic'),
        #         'status_topic' : LaunchConfiguration('cam4_status_topic'),
        #         'timeout_ms': timeout,
        #         'expected_fps': fps,
        #         'host': "192.168.0.177",

        #     }]
        # ),

        
    ])

