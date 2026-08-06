
import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    # Lidar1 (192.168.11.201)
    driver_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file_1 = os.path.join(driver_share_dir_1, 'config', '11_201.yaml')
    velodyne_driver_node_1 = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file_1],
                                                   remappings=[('/velodyne_packets', '/lidar_11_201/packets')]
                                                   )

    convert_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file_1 = os.path.join(convert_share_dir_1, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file_1, 'r') as f:
        convert_params_1 = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params_1['calibration'] = os.path.join(convert_share_dir_1, 'params', 'VLP16db.yaml')
    velodyne_transform_node_1 = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params_1],
                                                      remappings=[('/velodyne_packets', '/lidar_11_201/packets'),
                                                                  ('/velodyne_points', '/lidar_11_201/points')
                                                                  ]
                                                      )

    laserscan_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_1 = os.path.join(laserscan_share_dir_1, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_1 = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file_1],
                                                      )
    
    # Static Transform Publisher for Lidar 1 (velo1 -> vehicle)
    static_tf_1 = launch_ros.actions.Node(package='tf2_ros',
                                          executable='static_transform_publisher',
                                          arguments=['2.2', '1', '0', '1.15', '0', '0', 'vehicle', 'velo1'])  ## x y z yaw pitch roll parent_frame child_frame

    


    return launch.LaunchDescription([velodyne_driver_node_1,
                                     velodyne_transform_node_1,
                                     velodyne_laserscan_node_1,
                                     static_tf_1,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_1,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
