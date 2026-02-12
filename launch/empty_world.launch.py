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
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('sendbooster_agv_simulation')
    launch_file_dir = os.path.join(pkg_share, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Set GAZEBO_MODEL_PATH
    gazebo_model_path = os.path.join(pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-7.5')
    y_pose = LaunchConfiguration('y_pose', default='-9')

    world = os.path.join(pkg_share, 'worlds', 'empty_world.world')

    # Debug print
    print(f"[DEBUG] GAZEBO_MODEL_PATH: {gazebo_model_path}")
    print(f"[DEBUG] World file: {world}")
    print(f"[DEBUG] World exists: {os.path.exists(world)}")

    # Set environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    # Launch gzclient without EOL plugin to avoid crash
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_agv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_sendbooster_agv.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_agv_cmd)

    return ld
