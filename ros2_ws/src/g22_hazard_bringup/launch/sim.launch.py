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

"""Starts Gazebo with our world, the UAV and the two ground robots, each with its own Nav2."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def launch_arguments(pkg_dir, robots):
    arguments = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_dir, 'worlds', 'hazard_zone.sdf'),
                              description='Full path to world file to load'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='Run Gazebo without its GUI'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_dir, 'maps', 'hazard_zone.yaml'),
                              description='Full path to map file to load'),
    ]
    for robot in robots:
        name = robot['name']
        arguments.append(DeclareLaunchArgument(
            f'{name}_params_file', default_value=os.path.join(pkg_dir, 'config', f'nav2_{name}.yaml'),
            description=f'Full path to the ROS2 parameters file for {name}'))
    arguments.append(DeclareLaunchArgument('autostart', default_value='true',
                                          description='Automatically startup the stacks'))
    return arguments


def gazebo_processes():
    return [
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', LaunchConfiguration('world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen',
                       condition=UnlessCondition(LaunchConfiguration('headless'))),
    ]


def simulation_bridges(pkg_dir):
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': os.path.join(pkg_dir, 'config', 'gz_bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        # Separate image transport keeps camera traffic away from the clock bridge.
        Node(package='ros_gz_image', executable='image_bridge', arguments=['/uav1/camera/image'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]


def spawn_robot(robot, pkg_dir):
    name = robot['name']
    xacro = os.path.join(pkg_dir, 'models', 'ugv', 'ugv.sdf.xacro')
    return Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', name,
                   '-string', Command([FindExecutable(name='xacro'), ' ', xacro, ' namespace:=', name]),
                   '-x', str(robot['x_pose']), '-y', str(robot['y_pose']),
                   '-z', '0.01', '-Y', str(robot['yaw'])])


def robot_bridge(name, pkg_dir):
    return Node(
        package='ros_gz_bridge', executable='parameter_bridge', namespace=name,
        parameters=[{'config_file': os.path.join(pkg_dir, 'config', 'ugv_bridge.yaml'),
                     'expand_gz_topic_names': True, 'use_sim_time': True}], output='screen')


def static_transforms(name):
    transforms = [
        ['--z', '0.09', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        ['--x', '0.1', '--z', '0.08', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
    ]
    return [Node(package='tf2_ros', executable='static_transform_publisher', namespace=name,
                 arguments=arguments, remappings=TF_REMAPS) for arguments in transforms]


def navigation_launch(name, params_file, launch_dir):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': name, 'use_namespace': 'True', 'slam': 'False',
                          'map': LaunchConfiguration('map'), 'use_sim_time': 'True',
                          'params_file': params_file, 'autostart': LaunchConfiguration('autostart'),
                          'use_composition': 'True', 'use_respawn': 'False'}.items())


def robot_bringup(robot, pkg_dir, nav2_launch_dir):
    name = robot['name']
    params_file = LaunchConfiguration(f'{name}_params_file')
    return GroupAction([
        spawn_robot(robot, pkg_dir), robot_bridge(name, pkg_dir), *static_transforms(name),
        navigation_launch(name, params_file, nav2_launch_dir),
        LogInfo(msg=['Launching ', name, ' with ', params_file]),
    ])


def generate_launch_description():
    pkg_dir = get_package_share_directory('g22_hazard_bringup')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    # AMCL's initial poses in nav2_ugvN.yaml must match these spawn poses.
    robots = [
        {'name': 'ugv1', 'x_pose': -11.0, 'y_pose': -7.8, 'yaw': 0.0},
        {'name': 'ugv2', 'x_pose': 12.0, 'y_pose': -9.0, 'yaw': 3.14159},
    ]
    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_dir, 'models')),
        *launch_arguments(pkg_dir, robots), *gazebo_processes(), *simulation_bridges(pkg_dir),
        *(robot_bringup(robot, pkg_dir, nav2_launch_dir) for robot in robots),
    ])
