"""Sensors on the three robots. Run after sim.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('g22_hazard_mission')
    hazards_file = LaunchConfiguration('hazards_file')

    # name, where the position comes from, readings per second
    robots = [('uav1', 'odom', 5.0), ('ugv1', 'tf', 2.0), ('ugv2', 'tf', 2.0)]

    sensors = [
        Node(
            package='g22_hazard_mission',
            executable='hazard_sensor',
            namespace=name,
            parameters=[{
                'robot': name,
                'pose_source': source,
                'rate': rate,
                'hazards_file': hazards_file,
                'use_sim_time': True,
            }],
            # each robot has its own TF tree in its namespace
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        )
        for name, source, rate in robots
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'hazards_file',
            default_value=os.path.join(pkg_dir, 'config', 'hazards.yaml'),
            description='Hidden hazard sources'),
        *sensors,
    ])
