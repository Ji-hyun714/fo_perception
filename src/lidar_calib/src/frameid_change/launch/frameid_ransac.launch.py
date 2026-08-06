from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # -----------------------------
    # ADSP Change nodes (frame_id overwrite)
    # -----------------------------
    # change_left = Node(
    #     package='frameid_change',
    #     executable='pcd_frameid_change_ransac',
    #     name='change_left',
    #     parameters=[
    #         {'input_topic': '/lidar/points_fl'},
    #         {'output_topic': '/iv_points_left_change'},
    #         {'new_frame_id': 'seyond_left'},
    #     ]
    # )

    # change_right = Node(
    #     package='frameid_change',
    #     executable='pcd_frameid_change_ransac',
    #     name='change_right',
    #     parameters=[
    #         {'input_topic': '/lidar/points_fr'},
    #         {'output_topic': '/iv_points_right_change'},
    #         {'new_frame_id': 'seyond_right'},
    #     ]
    # )

    # -----------------------------
    # FO Change nodes (frame_id overwrite)
    # -----------------------------
    change_fl = Node(
        package='frameid_change',
        executable='pcd_frameid_change',
        name='change_fl',
        parameters=[
            {'input_topic': '/TFnonground/lidar_11'},
            # {'input_topic': '/nonground/lidar_11'},
            {'output_topic': '/change_frame/lidar_11'},
            {'new_frame_id': 'velo11'},
        ]
    )

    change_fr = Node(
        package='frameid_change',
        executable='pcd_frameid_change',
        name='change_fr',
        parameters=[
            {'input_topic': '/TFnonground/lidar_12'},
            # {'input_topic': '/nonground/lidar_12'},
            {'output_topic': '/change_frame/lidar_12'},
            {'new_frame_id': 'velo12'},
        ]
    )

    change_r = Node(
        package='frameid_change',
        executable='pcd_frameid_change',
        name='change_r',
        parameters=[
            {'input_topic': '/TFnonground/lidar_13'},
            # {'input_topic': '/nonground/lidar_13'},
            {'output_topic': '/change_frame/lidar_13'},
            {'new_frame_id': 'velo13'},
        ]
    )

    return LaunchDescription([
        change_fl,
        change_fr,
        change_r
    ])

