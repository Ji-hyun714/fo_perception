# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# """Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    # Lidar (192.168.11.201)
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


    return launch.LaunchDescription([velodyne_driver_node_1,
                                     velodyne_transform_node_1,
                                     velodyne_laserscan_node_1,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_1,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])

# import os
# import yaml

# import ament_index_python.packages
# import launch
# import launch_ros.actions


# def generate_launch_description():
#     # LiDAR 1
#     driver_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
#     driver_params_file_1 = os.path.join(driver_share_dir_1, 'config', '11_201.yaml')
#     velodyne_driver_node_1 = launch_ros.actions.Node(package='velodyne_driver',
#                                                      executable='velodyne_driver_node',
#                                                     #  namespace='lidar_11_201',  # 네임스페이스 설정
#                                                      output='both',
#                                                      parameters=[driver_params_file_1],
#                                                      remappings=[('/velodyne_points', '/lidar_11_201/points')]
#                                                      )
    
    

#     convert_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
#     convert_params_file_1 = os.path.join(convert_share_dir_1, 'config', 'VLP16-velodyne_transform_node-params.yaml')
#     with open(convert_params_file_1, 'r') as f_1:
#         convert_params_1 = yaml.safe_load(f_1)['velodyne_transform_node']['ros__parameters']
#     convert_params_1['calibration'] = os.path.join(convert_share_dir_1, 'params', 'VLP16db.yaml')
#     velodyne_transform_node_1 = launch_ros.actions.Node(package='velodyne_pointcloud',
#                                                         executable='velodyne_transform_node',
#                                                         output='both',
#                                                         parameters=[convert_params_1])

#     # LiDAR 2
#     driver_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
#     driver_params_file_2 = os.path.join(driver_share_dir_2, 'config', '12_201.yaml')
#     velodyne_driver_node_2 = launch_ros.actions.Node(package='velodyne_driver',
#                                                      executable='velodyne_driver_node',
#                                                     #  namespace='lidar_12_201',  # 네임스페이스 설정
#                                                      output='both',
#                                                      parameters=[driver_params_file_2],
#                                                      remappings=[('/velodyne_points', '/lidar_12_201/points')]
#                                                      )

#     convert_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
#     convert_params_file_2 = os.path.join(convert_share_dir_2, 'config', 'VLP16-velodyne_transform_node-params.yaml')
#     with open(convert_params_file_2, 'r') as f_2:
#         convert_params_2 = yaml.safe_load(f_2)['velodyne_transform_node']['ros__parameters']
#     convert_params_2['calibration'] = os.path.join(convert_share_dir_2, 'params', 'VLP16db.yaml')
#     velodyne_transform_node_2 = launch_ros.actions.Node(package='velodyne_pointcloud',
#                                                         executable='velodyne_transform_node',
#                                                         output='both',
#                                                         parameters=[convert_params_2])

#     # LiDAR 3
#     driver_share_dir_3 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
#     driver_params_file_3 = os.path.join(driver_share_dir_3, 'config', '13_201.yaml')
#     velodyne_driver_node_3 = launch_ros.actions.Node(package='velodyne_driver',
#                                                      executable='velodyne_driver_node',
#                                                     #  namespace='lidar_13_201',  # 네임스페이스 설정
#                                                      output='both',
#                                                      parameters=[driver_params_file_3],
#                                                      remappings=[('/velodyne_points', '/lidar_13_201/points')]
#                                                      )

#     convert_share_dir_3 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
#     convert_params_file_3 = os.path.join(convert_share_dir_3, 'config', 'VLP16-velodyne_transform_node-params.yaml')
#     with open(convert_params_file_3, 'r') as f_3:
#         convert_params_3 = yaml.safe_load(f_3)['velodyne_transform_node']['ros__parameters']
#     convert_params_3['calibration'] = os.path.join(convert_share_dir_3, 'params', 'VLP16db.yaml')
#     velodyne_transform_node_3 = launch_ros.actions.Node(package='velodyne_pointcloud',
#                                                         executable='velodyne_transform_node',
#                                                         output='both',
#                                                         parameters=[convert_params_3])

#     return launch.LaunchDescription([
#         velodyne_driver_node_1,
#         velodyne_transform_node_1,

#         velodyne_driver_node_2,
#         velodyne_transform_node_2,

#         velodyne_driver_node_3,
#         velodyne_transform_node_3,

#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=velodyne_driver_node_1,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )),
#     ])
