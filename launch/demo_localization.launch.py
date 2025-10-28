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
from launch.actions import     OpaqueFunction


def urdf_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = LaunchConfiguration('urdf_file_name', default='liosam.urdf')

    # urdf_path = urdf_file_name.perform(context=context)
    urdf_path = os.path.join(
        get_package_share_directory('glim_ros'),
        'urdf',urdf_file_name.perform(context=context))
    print("urdf path: {}".format(urdf_path))

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return [
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

def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value='false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    publish_urdf_arg = DeclareLaunchArgument(
        "publish_urdf", default_value='false'
    )

    publish_urdf = LaunchConfiguration('publish_urdf')

    config_launch_arg = DeclareLaunchArgument(
        "config", default_value=TextSubstitution(text="config/liosam/walking")
    )

    localization_launch_arg = DeclareLaunchArgument(
        "localization", default_value='false'
    )

    map_path_launch_arg = DeclareLaunchArgument(
        "map_path", default_value="/home/manh/Documents/park_gps"
    )

    urdf_arg = DeclareLaunchArgument('urdf_file_name', default_value='liosam.urdf')
    urdf_node = GroupAction(
        condition=IfCondition(publish_urdf),
        actions=[
            OpaqueFunction(function=urdf_setup),
        ]
    )

    glim_ros_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        parameters=[{
            "config_path": LaunchConfiguration('config'),
            "localization": LaunchConfiguration('localization'),
            "map_path": LaunchConfiguration('map_path'),
            "use_sim_time": use_sim_time,
        }],
        output='screen',
        # prefix=["gnome-terminal -- gdb -ex run --args"]

    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(publish_urdf_arg)
    ld.add_action(config_launch_arg)
    ld.add_action(localization_launch_arg)
    ld.add_action(map_path_launch_arg)
    ld.add_action(glim_ros_node)
    ld.add_action(urdf_arg)
    ld.add_action(urdf_node)

    # Add the commands to the launch description

    return ld
