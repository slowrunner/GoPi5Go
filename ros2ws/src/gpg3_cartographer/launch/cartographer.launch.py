#!/usr/bin/env python3

#  FILE:  cartographer.launch.py for GoPiGo3
#  AUTHOR:  mod for GoPiGo3: slowrunner

# Copyright 2022 ROBOTIS CO., LTD.
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
# Author: Darby Lim

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare('gpg3_cartographer'),
            'config',
        ]
    )
    configuration_basename = LaunchConfiguration('configuration_basename')

    resolution = LaunchConfiguration('resolution')


    return LaunchDescription([

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full direction of config file'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='gpg3_2d.lua',
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell of occupancy grid'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[],
            arguments=['-resolution', resolution]),

    ])
