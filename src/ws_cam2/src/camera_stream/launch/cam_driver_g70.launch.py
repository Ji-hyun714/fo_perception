#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_stream')

    # rtsp_url = "rtsp://192.168.0.52:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream"
    # rtsp_url = "rtsp://192.168.0.220:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream" # g70 cam
    rtsp_url = "rtsp://192.168.0.150:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream" # lab test
    gst_pipe = ("rtspsrc location=" + rtsp_url + " protocols=tcp latency=0 "
                "! rtph265depay "
                "! h265parse " 
                "! nvv4l2decoder "
                "! nvvidconv "
                "! video/x-raw,format=BGRx,width=1280,height=720 "
                "! videoconvert "
                "! video/x-raw,format=BGR "
                "! appsink name=appsink")
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'pipeline',
            default_value=gst_pipe,
            description='The GStreamer pipeline string for the camera'
        ),
        
        DeclareLaunchArgument('frame_id', default_value='camera'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('image_rect_topic', default_value='/image_rect'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        # DeclareLaunchArgument('params_file', default_value=default_params_file),


        Node(
            package='camera_stream',
            executable='gst_ipcam_node',
            name='gst_ipcam_node',
            output='screen',
            parameters=[{
                'pipeline': LaunchConfiguration('pipeline'),
                
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
                'input_topic': LaunchConfiguration('image_topic'),
                'output_topic': LaunchConfiguration('image_rect_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                # TODO: 카메라 파라미터도 여기서 설정할 수 있게          
            }]
        ),
    ])

