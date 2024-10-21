#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import AppendEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value='false'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

        
    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/velodyne")
    )

    sensor_nodes = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
          Node(
            package='openzen_driver',
            executable='openzen_node',
            name='openzen_node'),
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(os.path.join(
                  get_package_share_directory('velodyne_driver'), 'launch', 'velodyne_driver_node-VLP16-launch.py'
              ))
          ),
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(os.path.join(
                  get_package_share_directory('velodyne_pointcloud'), 'launch', 'velodyne_transform_node-VLP16-launch.py'
          ))
          ),
        ]
    )

    urdf_path = '/home/manh/tvc_nav/src/tvc_description/urdf/amr_1lidar.urdf'

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    urdf_node = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_desc
                }],
            ),
        ]
    )

    glim_ros_node = Node(
        package='glim_ros',
        executable='glim_localization',
        name='glim_ros',
        parameters=[{
            "config_path": LaunchConfiguration('config'),
            "use_sim_time": use_sim_time
        }],
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_launch_arg)
    ld.add_action(glim_ros_node)
    ld.add_action(sensor_nodes)
    # ld.add_action(octomap_server)
    ld.add_action(urdf_node)


    # Add the commands to the launch description

    return ld