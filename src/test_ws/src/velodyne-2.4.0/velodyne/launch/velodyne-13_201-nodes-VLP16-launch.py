
import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    
    # Lidar3 (192.168.13.201)
    driver_share_dir_3 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file_3 = os.path.join(driver_share_dir_3, 'config', '13_201.yaml')
    velodyne_driver_node_3 = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file_3],
                                                   remappings=[('/velodyne_packets', '/lidar_13_201/packets')]
                                                   )

    convert_share_dir_3 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file_3 = os.path.join(convert_share_dir_3, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file_3, 'r') as f:
        convert_params_3 = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params_3['calibration'] = os.path.join(convert_share_dir_3, 'params', 'VLP16db.yaml')
    velodyne_transform_node_3 = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params_3],
                                                      remappings=[('/velodyne_packets', '/lidar_13_201/packets'),
                                                                  ('/velodyne_points', '/lidar_13_201/points')
                                                                  ]
                                                      )

    laserscan_share_dir_3 = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_3 = os.path.join(laserscan_share_dir_3, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_3 = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file_3],
                                                      )
    
    # Static Transform Publisher for Lidar 1 (velo1 -> vehicle)
    static_tf_3 = launch_ros.actions.Node(package='tf2_ros',
                                          executable='static_transform_publisher',
                                          arguments=['-2', '0', '0', '-2.60', '0', '0', 'vehicle', 'velo3'])  ## x y z yaw pitch roll parent_frame child_frame


    return launch.LaunchDescription([velodyne_driver_node_3,
                                     velodyne_transform_node_3,
                                     velodyne_laserscan_node_3,
                                     static_tf_3,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_3,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
