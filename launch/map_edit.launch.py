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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
        # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value='true'
    )
    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/velodyne")
    )
    resolution_launch_arg = DeclareLaunchArgument(
        "resolution", default_value=TextSubstitution(text="0.2")
    )
    frame_id_launch_arg = DeclareLaunchArgument(
        "frame_id", default_value=TextSubstitution(text="map")
    )
    max_range_launch_arg = DeclareLaunchArgument(
        "max_range", default_value=TextSubstitution(text="200.0")
    )

    glim_ros_node = Node(
        package='glim_ros',
        executable='offline_viewer',
        name='glim_ros',
        parameters=[{
            "config_path": LaunchConfiguration('config'),
                "use_sim_time": LaunchConfiguration("use_sim_time")
        }],

        arguments=[LaunchConfiguration('config')],
        output='screen'
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
                "resolution": LaunchConfiguration('resolution'),
                "frame_id": LaunchConfiguration('frame_id'),
                "base_frame_id": "base_footprint",
                "sensor_model.max_range": LaunchConfiguration('max_range'),
                # "point_cloud_min_z": -0.5,
                # "point_cloud_max_z": 1.0,
                "ground_filter.distance": 0.2,
                "ground_filter.plane_distance": 0.2,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # "incremental_2D_projection": True,
            }],
        remappings=[
            ('/cloud_in', '/glim_ros/map'),
        ],
        output='screen'
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "1.5", "0", "0", "0", "base_footprint", "imu"],
        parameters=[{
            # "config_path": LaunchConfiguration('config'),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(resolution_launch_arg)
    ld.add_action(config_launch_arg)

    ld.add_action(frame_id_launch_arg)
    ld.add_action(max_range_launch_arg)
    ld.add_action(glim_ros_node)
    ld.add_action(octomap_server)
    ld.add_action(static_tf_node)


    # Add the commands to the launch description

    return ld
