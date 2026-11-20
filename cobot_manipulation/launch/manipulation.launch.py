from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
        Node(
            package='cobot_manipulation',
            executable='pick_place_coordinator',
            name='pick_place_coordinator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'conveyor_length': LaunchConfiguration('conveyor_length'),
                'conveyor_width': LaunchConfiguration('conveyor_width'),
                'conveyor_height': LaunchConfiguration('conveyor_height'),
                'conveyor_center_x': LaunchConfiguration('conveyor_center_x'),
                'conveyor_center_y': LaunchConfiguration('conveyor_center_y'),
                'collection_box_x': LaunchConfiguration('collection_box_x'),
                'collection_box_y': LaunchConfiguration('collection_box_y'),
                'collection_box_length': LaunchConfiguration('collection_box_length'),
                'collection_box_width': LaunchConfiguration('collection_box_width'),
                'collection_box_height': LaunchConfiguration('collection_box_height'),
            }],
        ),
    ])
