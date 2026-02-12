#!/usr/bin/env python3
#
# Copyright 2024 TerraNox
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
# Author: 이성민 (roboticsmaster@naver.com)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('sendbooster_agv_simulation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart', default='true')

    # Default paths
    default_map_path = os.path.join(
        pkg_share, 'models', 'sendbooster_world', 'meshes', 'map', 'map.yaml'
    )
    default_params_path = os.path.join(pkg_share, 'param', 'nav2_params.yaml')
    ekf_config_path = os.path.join(pkg_share, 'param', 'ekf_config.yaml')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to Nav2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack'
    )

    # Include Nav2 bringup launch
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False'
        }.items()
    )

    # Laser Scan Merger - combines front and back LiDAR for 360° coverage
    laser_merger_cmd = Node(
        package='sendbooster_agv_simulation',
        executable='laser_scan_merger',
        name='laser_scan_merger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'destination_frame': 'base_link',
            'scan_destination_topic': '/scan_merged'
        }],
        output='screen'
    )

    # EKF Localization - fuses odom, IMU, and AMCL pose for accurate localization
    ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Pose Reinitializer - periodically updates initial pose to correct odometry drift
    pose_reinit_cmd = Node(
        package='sendbooster_agv_simulation',
        executable='pose_reinitializer',
        name='pose_reinitializer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'reinit_period': 30.0,  # seconds
            'covariance_threshold': 0.1  # only reinit if covariance is low
        }],
        output='screen'
    )

    # RViz with Nav2 config
    rviz_config_file = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add actions - EKF and laser merger must start before Nav2
    ld.add_action(laser_merger_cmd)
    ld.add_action(ekf_node_cmd)  # EKF provides odom->base_footprint TF
    ld.add_action(nav2_bringup_cmd)
    # ld.add_action(pose_reinit_cmd)  # Disabled - AMCL handles drift naturally
    ld.add_action(rviz_cmd)

    return ld
