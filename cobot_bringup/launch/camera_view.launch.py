#!/usr/bin/env python3
"""Open RViz image panels for the live conveyor RGB-D topics."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = os.path.join(
        FindPackageShare('cobot_bringup').find('cobot_bringup'),
        'config', 'camera_view.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='cobot_camera_view',
            arguments=['-d', config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
