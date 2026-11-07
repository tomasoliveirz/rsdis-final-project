"""Sensors on the three robots, the UAV mission, the coordinator, the metrics logger,
sign detection, the hazard map and RViz. Run after sim.launch.py (or use all.launch.py)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mission_node(executable, namespace='', parameters=None, remappings=None):
    return Node(package='g22_hazard_mission', executable=executable, namespace=namespace,
                parameters=[{'use_sim_time': True, **(parameters or {})}],
                remappings=remappings, output='screen')


def sensor_node(robot, hazards_file):
    name, pose_source, rate = robot
    return mission_node('hazard_sensor', name,
                        {'robot': name, 'pose_source': pose_source, 'rate': rate,
                         'hazards_file': hazards_file},
                        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])


def rviz_node(pkg_dir):
    return Node(package='rviz2', executable='rviz2',
                arguments=['-d', os.path.join(pkg_dir, 'rviz', 'hazards.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')))


def launch_arguments(pkg_dir):
    return [
        DeclareLaunchArgument('hazards_file', default_value=os.path.join(pkg_dir, 'config', 'hazards.yaml'),
                              description='Hidden hazard sources'),
        DeclareLaunchArgument('show_truth', default_value='false',
                              description='Also draw the real sources in RViz'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('g22_hazard_mission')
    hazards_file = LaunchConfiguration('hazards_file')
    robots = [('uav1', 'odom', 10.0), ('ugv1', 'tf', 2.0), ('ugv2', 'tf', 2.0)]
    return LaunchDescription([
        *launch_arguments(pkg_dir),
        *(sensor_node(robot, hazards_file) for robot in robots),
        mission_node('uav_survey', 'uav1'),
        mission_node('sign_detector', 'uav1'),
        mission_node('coordinator', parameters={'robots': ['ugv1', 'ugv2']}),
        mission_node('mission_logger', parameters={'hazards_file': hazards_file}),
        mission_node('hazard_viz', parameters={'robots': [robot[0] for robot in robots],
                                             'hazards_file': hazards_file,
                                             'show_truth': LaunchConfiguration('show_truth')}),
        rviz_node(pkg_dir),
    ])
