"""Whole mission from one launch file: the simulation, then (after Nav2 has had
time to start) the sensors, UAV, coordinator, logger, hazard map and RViz.

    ros2 launch g22_hazard_mission all.launch.py headless:=True
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('g22_hazard_bringup')
    mission = get_package_share_directory('g22_hazard_mission')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless'),
                          'video_camera': LaunchConfiguration('video_camera')}.items(),
    )
    nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mission, 'launch', 'mission.launch.py')),
        launch_arguments={
            'show_truth': LaunchConfiguration('show_truth'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('video_camera', default_value='false',
                              description='Front camera on each UGV, for the demo video only'),
        DeclareLaunchArgument('show_truth', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mission_delay', default_value='25.0',
                              description='Seconds to wait for Gazebo and Nav2 before the mission'),
        sim,
        TimerAction(period=LaunchConfiguration('mission_delay'), actions=[nodes]),
    ])
