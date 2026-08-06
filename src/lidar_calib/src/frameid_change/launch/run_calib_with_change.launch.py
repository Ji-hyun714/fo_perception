from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # -----------------------------
    # 1. Change nodes (frame_id overwrite)
    # -----------------------------
    change_left = Node(
        package='frameid_change',
        executable='pcd_frameid_change',
        name='change_left',
        parameters=[
            {'input_topic': '/iv_points_left'},
            {'output_topic': '/iv_points_left_change'},
            {'new_frame_id': 'seyond_left'},
        ]
    )

    change_right = Node(
        package='frameid_change',
        executable='pcd_frameid_change',
        name='change_right',
        parameters=[
            {'input_topic': '/iv_points_right'},
            {'output_topic': '/iv_points_right_change'},
            {'new_frame_id': 'seyond_right'},
        ]
    )

    # -----------------------------
    # 2. Include Multi_LiCa launch
    # -----------------------------
    multilica_pkg = get_package_share_directory('multi_lidar_calibrator')
    multilica_launch = os.path.join(multilica_pkg, 'launch', 'calibration.launch.py')

    multilica = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multilica_launch),
        launch_arguments={
            'parameter_file': '/home/wise/adsp_perception_lidar/src/lidar_calib/src/Multi_LiCa/config/params.yaml'
        }.items()
    )

    return LaunchDescription([
        change_left,
        change_right,
        multilica
    ])

