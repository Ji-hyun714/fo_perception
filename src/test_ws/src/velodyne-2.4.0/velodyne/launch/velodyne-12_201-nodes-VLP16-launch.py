
import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    
    # Lidar2 (192.168.12.201)
    driver_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file_2 = os.path.join(driver_share_dir_2, 'config', '12_201.yaml')
    velodyne_driver_node_2 = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file_2],
                                                   remappings=[('/velodyne_packets', '/lidar_12_201/packets')]
                                                   )

    convert_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file_2 = os.path.join(convert_share_dir_2, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file_2, 'r') as f:
        convert_params_2 = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params_2['calibration'] = os.path.join(convert_share_dir_2, 'params', 'VLP16db.yaml')
    velodyne_transform_node_2 = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params_2],
                                                      remappings=[('/velodyne_packets', '/lidar_12_201/packets'),
                                                                  ('/velodyne_points', '/lidar_12_201/points')
                                                                  ]
                                                      )

    laserscan_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_2 = os.path.join(laserscan_share_dir_2, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_2 = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file_2],
                                                      )
    
    # Static Transform Publisher for Lidar 1 (velo1 -> vehicle)
    static_tf_2 = launch_ros.actions.Node(package='tf2_ros',
                                          executable='static_transform_publisher',
                                          arguments=['2.2', '-1', '0', '-1.20', '0', '0', 'vehicle', 'velo2'])  ## x y z yaw pitch roll parent_frame child_frame

    return launch.LaunchDescription([velodyne_driver_node_2,
                                     velodyne_transform_node_2,
                                     velodyne_laserscan_node_2,
                                     static_tf_2,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_2,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
