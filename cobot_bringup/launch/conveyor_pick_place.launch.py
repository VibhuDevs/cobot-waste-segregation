#!/usr/bin/env python3
"""Integrated Gazebo, MoveIt, RGB-D perception, and pick/place launch."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_autonomy = LaunchConfiguration('enable_autonomy')
    world_file = LaunchConfiguration('world_file')
    conveyor_length = LaunchConfiguration('conveyor_length')
    conveyor_width = LaunchConfiguration('conveyor_width')
    conveyor_height = LaunchConfiguration('conveyor_height')
    conveyor_center_x = LaunchConfiguration('conveyor_center_x')
    conveyor_center_y = LaunchConfiguration('conveyor_center_y')
    collection_box_x = LaunchConfiguration('collection_box_x')
    collection_box_y = LaunchConfiguration('collection_box_y')
    collection_box_length = LaunchConfiguration('collection_box_length')
    collection_box_width = LaunchConfiguration('collection_box_width')
    collection_box_height = LaunchConfiguration('collection_box_height')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('cobot_gazebo').find('cobot_gazebo'),
                         'launch', 'cobot.gazebo.launch.py')]),
        launch_arguments={
            'world_file': world_file,
            'conveyor_length': conveyor_length,
            'conveyor_width': conveyor_width,
            'conveyor_height': conveyor_height,
            'conveyor_center_x': conveyor_center_x,
            'conveyor_center_y': conveyor_center_y,
            'collection_box_x': collection_box_x,
            'collection_box_y': collection_box_y,
            'collection_box_length': collection_box_length,
            'collection_box_width': collection_box_width,
            'collection_box_height': collection_box_height,
            'load_controllers': 'true',
            'use_robot_state_pub': 'true',
            'use_rviz': 'false',
            'jsp_gui': 'false',
            'use_camera': 'true',
            'use_conveyor': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('cobot_moveit_config').find('cobot_moveit_config'),
                         'launch', 'move_group.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('cobot_perception').find('cobot_perception'),
                         'launch', 'perception.launch.py')]),
        launch_arguments={
            'conveyor_length': conveyor_length,
            'conveyor_width': conveyor_width,
            'conveyor_height': conveyor_height,
            'conveyor_center_x': conveyor_center_x,
            'conveyor_center_y': conveyor_center_y,
        }.items(),
    )
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('cobot_manipulation').find('cobot_manipulation'),
                         'launch', 'manipulation.launch.py')]),
        launch_arguments={
            'conveyor_length': conveyor_length,
            'conveyor_width': conveyor_width,
            'conveyor_height': conveyor_height,
            'conveyor_center_x': conveyor_center_x,
            'conveyor_center_y': conveyor_center_y,
            'collection_box_x': collection_box_x,
            'collection_box_y': collection_box_y,
            'collection_box_length': collection_box_length,
            'collection_box_width': collection_box_width,
            'collection_box_height': collection_box_height,
        }.items(),
    )

    rviz_config = os.path.join(
        FindPackageShare('cobot_moveit_config').find('cobot_moveit_config'),
        'config', 'cobot_280_moveit.rviz')
    moveit_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='conveyor_pick_place.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_autonomy', default_value='true'),
        DeclareLaunchArgument('conveyor_length', default_value='0.5'),
        DeclareLaunchArgument('conveyor_width', default_value='0.16'),
        DeclareLaunchArgument('conveyor_height', default_value='0.08'),
        DeclareLaunchArgument('conveyor_center_x', default_value='0.32'),
        DeclareLaunchArgument('conveyor_center_y', default_value='0.14'),
        DeclareLaunchArgument('collection_box_x', default_value='0.15'),
        DeclareLaunchArgument('collection_box_y', default_value='-0.16'),
        DeclareLaunchArgument('collection_box_length', default_value='0.16'),
        DeclareLaunchArgument('collection_box_width', default_value='0.16'),
        DeclareLaunchArgument('collection_box_height', default_value='0.10'),
        gazebo_launch,
        TimerAction(period=12.0, actions=[moveit_launch]),
        TimerAction(period=14.0, actions=[perception_launch]),
        TimerAction(period=15.0, actions=[moveit_rviz]),
        TimerAction(
            period=20.0,
            actions=[manipulation_launch],
            condition=IfCondition(enable_autonomy),
        ),
    ])
