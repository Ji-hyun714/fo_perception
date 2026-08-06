import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('patchworkpp'),
    'config',
    'params_ros2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value="/merge_201/points", description="a pointcloud topic to process",),
        DeclareLaunchArgument('cloud_frame', default_value="vehicle", description="a pointcloud topic to process",),
        
        Node(
            package='patchworkpp',
            executable='demo',
            name='ground_segmentation',
            output='screen',
            parameters=[
                {'cloud_topic': LaunchConfiguration("cloud_topic")}, # Input pointcloud
                {'frame_id': LaunchConfiguration("cloud_frame")},
                {'sensor_height': 0.6},
                {'num_iter': 3},             # Number of iterations for ground plane estimation using PCA.
                {'num_lpr': 30},             # Maximum number of points to be selected as lowest points representative. (Lowest Point Representative 수)
                {'num_min_pts': 10},         # Minimum number of points to be estimated as ground plane in each patch. (0개 이하 point → ground estimation skip)
                {'th_seeds': 0.2},           # threshold for lowest point representatives using in initial seeds selection of ground points. (ground seed 선택 threshold: lowest point~lowest point+th_seeds 사이에서 seed 선택)
                {'th_dist': 0.05},           # ground의 두께 (범위 안의 point는 모두 ground로 판단 / 값이 작으면 ground 엄격, 값이 크면 ground가 두꺼워짐)
                {'th_seeds_v': 0.6},         # ground와 수직구조물을 구분하기 위한 seed (z가 0.25m 이상 차이나면 vertical structure seed로 사용)
                {'th_dist_v': 0.6},          # vertical plane 두께 (plane ± 0.9m를 vertical structure로 인정)
                {'max_r': 70.0},             # max_range of ground estimation area
                {'min_r': 3.0},              # min_range of ground estimation area
                # {'min_r': 1.0},              # min_range of ground estimation area
                {'uprightness_thr': 0.101},  # threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE. (ground plane normal vector 검사)
                {'verbose': False},          # display verbose info
                {'display_time': False},     # display running_time and pointcloud sizes
            ],
            arguments=[LaunchConfiguration('cloud_topic')],
        ),
    ])
 


