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
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # args that can be set from the command line or a default will be used

    sensor_nodes = GroupAction(
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

    glim_ros_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        parameters=[{
            "config_path": LaunchConfiguration('config'),
                "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(glim_ros_node)
    ld.add_action(sensor_nodes)
    # ld.add_action(octomap_server)
    # ld.add_action(static_tf_node)


    # Add the commands to the launch description

    return ld