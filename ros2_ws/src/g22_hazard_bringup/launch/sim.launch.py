# Copyright (c) 2018 Intel Corporation
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
# Changed by RSDIS Group 22 (2026) for our world, robots and UAV.

"""Starts Gazebo with our world, the UAV and the two ground robots."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
)
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('g22_hazard_bringup')
    ugv_xacro = os.path.join(pkg_dir, 'models', 'ugv', 'ugv.sdf.xacro')

    # Spawn poses
    robots = [
        {'name': 'ugv1', 'x_pose': -11.0, 'y_pose': -7.8, 'yaw': 0.0},
        {'name': 'ugv2', 'x_pose': 12.0, 'y_pose': -9.0, 'yaw': 3.14159},
    ]

    # Simulation settings
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'hazard_zone.sdf'),
        description='Full path to world file to load',
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo without its GUI',
    )

    # Gazebo server, and the GUI unless headless
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
    )

    start_gazebo_gui_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    # Clock and UAV topics
    gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[
            {
                'config_file': os.path.join(pkg_dir, 'config', 'gz_bridge.yaml'),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Define commands for launching the ground robots
    nav_instances_cmds = []
    for robot in robots:
        name = robot['name']
        tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

        group = GroupAction(
            [
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-name', name,
                        '-string', Command([
                            FindExecutable(name='xacro'), ' ', ugv_xacro,
                            ' namespace:=', name]),
                        '-x', str(robot['x_pose']), '-y', str(robot['y_pose']),
                        '-z', '0.01', '-Y', str(robot['yaw'])],
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    namespace=name,
                    parameters=[
                        {
                            'config_file': os.path.join(pkg_dir, 'config', 'ugv_bridge.yaml'),
                            'expand_gz_topic_names': True,
                            'use_sim_time': True,
                        }
                    ],
                    output='screen',
                ),
                # static TFs of the robot body
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    namespace=name,
                    arguments=['--z', '0.09',
                               '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
                    remappings=tf_remaps,
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    namespace=name,
                    arguments=['--x', '0.1', '--z', '0.08',
                               '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
                    remappings=tf_remaps,
                ),
                LogInfo(msg=['Launching ', name]),
            ]
        )

        nav_instances_cmds.append(group)

    # so Gazebo finds model://x3_uav
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_dir, 'models'))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_headless_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_gui_cmd)
    ld.add_action(gz_bridge_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
