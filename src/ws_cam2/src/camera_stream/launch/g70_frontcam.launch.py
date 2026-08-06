#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_stream')

    # # rtsp_url = "rtsp://192.168.0.52:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream"
    # # rtsp_url = "rtsp://192.168.0.220:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream" # g70 cam
    # rtsp_url = "rtsp://192.168.0.150:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream" # lab test
    # gst_pipe = ("rtspsrc location=" + rtsp_url + " protocols=tcp latency=0 "
    #             "! rtph265depay "
    #             "! h265parse " 
    #             "! nvv4l2decoder "
    #             "! nvvidconv "
    #             "! video/x-raw,format=BGRx,width=1280,height=720 "
    #             "! videoconvert "
    #             "! video/x-raw,format=BGR "
    #             "! appsink name=appsink")

    yaml_path = "/home/wise/fo_perception/src/ws_cam2/src/camera_stream/yaml/cam_parameters_220.yaml"
    # yaml_path = "/home/wise/fo_perception/src/ws_cam2/src/camera_stream/yaml/test_rtsp_25.yaml"
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_path',
            default_value=yaml_path,
            description='YAML file path including camera intrinsic/extrinsic parameters.'
        ),
        
        DeclareLaunchArgument('frame_id', default_value='camera'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('image_rect_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),


        Node(
            package='camera_stream',
            executable='gst_ipcam_node',
            name='gst_ipcam_node',
            output='screen',
            parameters=[{
                # 'pipeline': LaunchConfiguration('pipeline'),
                'yaml_path': LaunchConfiguration('yaml_path'),
                
                'image_topic': LaunchConfiguration('image_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'timeout_ms': 200,
                'expected_fps': 30.0,
            }]
        ),

        Node(
            package='camera_stream',
            executable='rectify_node',
            name='rectify_node',
            output='screen',
            parameters=[{
                'yaml_path': LaunchConfiguration('yaml_path'),

                'input_topic': LaunchConfiguration('image_topic'),
                'output_topic': LaunchConfiguration('image_rect_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                # TODO: 카메라 파라미터도 여기서 설정할 수 있게          
            }]
        ),
    ])

