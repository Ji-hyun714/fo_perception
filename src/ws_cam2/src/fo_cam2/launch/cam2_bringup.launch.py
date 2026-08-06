#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('fo_cam2')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'cam2_ld_params.yaml'])

    return LaunchDescription([


        DeclareLaunchArgument('frame_id',          default_value='camera'),
        DeclareLaunchArgument('out_period',        default_value='20'),

        DeclareLaunchArgument('image_raw_topic',   default_value='/image_raw'),
        DeclareLaunchArgument('image_rect_topic',  default_value='/image_rect'),
        DeclareLaunchArgument('tracked_results',   default_value='/yolo/tracking'),
        DeclareLaunchArgument('topic_ld',          default_value='/cam2_ld'),
        DeclareLaunchArgument('topic_td',          default_value='/cam2_tdtlod'),
        DeclareLaunchArgument('topic_td_v2',       default_value='/cam2_td'),
        DeclareLaunchArgument('topic_tl_v2',       default_value='/cam2_tl'),
        DeclareLaunchArgument('topic_od_v2',       default_value='/cam2_od'),
        DeclareLaunchArgument('output_topic_cam2', default_value='/cam2_data'),
        DeclareLaunchArgument('use_builder_v2',    default_value='False'),
        
        DeclareLaunchArgument('image_width',       default_value='1280'),
        DeclareLaunchArgument('image_height',      default_value='720'),

        DeclareLaunchArgument('debug_mode',        default_value='False'),
    
        Node(
            package='fo_cam2',
            executable='cam2_builder',
            name='cam2_builder',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_builder_v2')),
            parameters=[{
                'input_topic_ld': LaunchConfiguration('topic_ld'),
                'input_topic_tdtlod': LaunchConfiguration('topic_td'),
                'output_topic': LaunchConfiguration('output_topic_cam2'),
                'timer_period_ms': LaunchConfiguration('out_period'),
            }]
        ),

        Node(
            package='fo_cam2',
            executable='cam2_builder_v2',
            name='cam2_builder_v2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_builder_v2')),
            parameters=[{
                'input_topic_ld': LaunchConfiguration('topic_ld'),
                'input_topic_td': LaunchConfiguration('topic_td_v2'),
                'input_topic_tl': LaunchConfiguration('topic_tl_v2'),
                'input_topic_od': LaunchConfiguration('topic_od_v2'),
                'output_topic': LaunchConfiguration('output_topic_cam2'),
                'publish_period_ms': LaunchConfiguration('out_period'),
            }]
        ),

        Node(
            package='fo_cam2',
            executable='ld_interface',
            name='ld_interface',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('image_rect_topic'),
                'output_topic': LaunchConfiguration('topic_ld'),
                'debug_mode': LaunchConfiguration('debug_mode'),
                # params_file,

            }]
            
        ),
   
        Node(
            package='fo_cam2',
            executable='tdtlod_interface',
            name='tdtlod_interface',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('tracked_results'),
                'input_topic_dbgimg': LaunchConfiguration('image_rect_topic'),
                'output_topic': LaunchConfiguration('topic_td'),
                'debug_mode': LaunchConfiguration('debug_mode'),
            }]
        ),
    ])
