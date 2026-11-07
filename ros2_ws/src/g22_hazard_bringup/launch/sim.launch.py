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
)
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml


TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def launch_arguments(pkg_dir):
    arguments = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_dir, 'worlds', 'hazard_zone.sdf'),
                              description='Full path to world file to load'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='Run Gazebo without its GUI'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_dir, 'maps', 'hazard_zone.yaml'),
                              description='Full path to map file to load'),
    ]
    arguments.append(DeclareLaunchArgument(
        'nav_params_file', default_value=os.path.join(pkg_dir, 'config', 'nav2.yaml'),
        description='Shared Nav2 parameters for the ground robots'))
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


def navigation_nodes(robot):
    """Only localisation, planning, driving and obstacle avoidance are needed."""
    name = robot['name']
    source = ReplaceString(source_file=LaunchConfiguration('nav_params_file'),
                           replacements={'<robot_namespace>': f'/{name}'})
    parameters = ParameterFile(RewrittenYaml(source_file=source, root_key=name,
                                             param_rewrites={}, convert_types=True))
    common = [parameters, {'use_sim_time': True}]
    navigation = [
        ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server'),
        ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
        ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
        ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor', 'collision_monitor'),
        ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
    ]
    components = [
        ComposableNode(package='nav2_map_server', plugin='nav2_map_server::MapServer',
                       name='map_server', namespace=name,
                       parameters=common + [{'yaml_filename': LaunchConfiguration('map')}],
                       remappings=TF_REMAPS),
        ComposableNode(package='nav2_amcl', plugin='nav2_amcl::AmclNode', name='amcl', namespace=name,
                       parameters=common + [{'initial_pose.x': robot['x_pose'],
                                             'initial_pose.y': robot['y_pose'],
                                             'initial_pose.yaw': robot['yaw']}], remappings=TF_REMAPS),
    ]
    for package, plugin, node_name in navigation:
        remaps = TF_REMAPS
        if node_name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remaps = remaps + [('cmd_vel', 'cmd_vel_nav')]
        components.append(ComposableNode(package=package, plugin=plugin, name=node_name,
                                         namespace=name, parameters=common, remappings=remaps))
    for group, nodes in [('localization', ['map_server', 'amcl']),
                         ('navigation', [node[2] for node in navigation])]:
        components.append(ComposableNode(
            package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
            name=f'lifecycle_manager_{group}', namespace=name,
            parameters=[{'use_sim_time': True,
                         'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                         'node_names': nodes}]))
    return ComposableNodeContainer(
        name='nav2_container', namespace=name, package='rclcpp_components',
        executable='component_container_isolated', composable_node_descriptions=components,
        parameters=common, remappings=TF_REMAPS,
        output='screen')


def robot_bringup(robot, pkg_dir):
    return GroupAction([
        spawn_robot(robot, pkg_dir), robot_bridge(robot['name'], pkg_dir),
        *static_transforms(robot['name']), navigation_nodes(robot),
    ])


def generate_launch_description():
    pkg_dir = get_package_share_directory('g22_hazard_bringup')
    robots = [
        {'name': 'ugv1', 'x_pose': -11.0, 'y_pose': -7.8, 'yaw': 0.0},
        {'name': 'ugv2', 'x_pose': 12.0, 'y_pose': -9.0, 'yaw': 3.14159},
    ]
    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_dir, 'models')),
        *launch_arguments(pkg_dir), *gazebo_processes(), *simulation_bridges(pkg_dir),
        *(robot_bringup(robot, pkg_dir) for robot in robots),
    ])
