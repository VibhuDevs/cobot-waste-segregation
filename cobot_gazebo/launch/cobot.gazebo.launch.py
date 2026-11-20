#!/usr/bin/env python3
"""
Launch Gazebo simulation with a robot.

This launch file sets up a complete ROS 2 simulation environment with Gazebo for
a cobot robot.

:author: @vibhudevs
:date: April 13, 2026
"""

import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the Gazebo simulation.

    This function sets up all necessary parameters, paths, and nodes required to launch
    the Gazebo simulation with a robot. It handles:
    1. Setting up package paths and constants
    2. Declaring launch arguments for robot configuration
    3. Setting up the Gazebo environment
    4. Spawning the robot in simulation

    Returns:
        LaunchDescription: A complete launch description for the simulation
    """
    # Constants for paths to different files and folders
    package_name_gazebo = 'cobot_gazebo'
    package_name_description = 'cobot_description'
    package_name_moveit = 'cobot_moveit_config'

    default_robot_name = 'cobot_280'
    gazebo_models_path = 'models'
    default_world_file = 'conveyor_pick_place.sdf'
    gazebo_worlds_path = 'worlds'

    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    # Set the path to different files and folders
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_description = FindPackageShare(
        package=package_name_description).find(package_name_description)
    pkg_share_moveit = FindPackageShare(package=package_name_moveit).find(package_name_moveit)

    gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
    default_ros_gz_bridge_config_file_path = os.path.join(
        pkg_share_gazebo, ros_gz_bridge_config_file_path)

    # Launch configuration variables
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_conveyor = LaunchConfiguration('use_conveyor')
    use_sim_time = LaunchConfiguration('use_sim_time')
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

    world_path = PathJoinSubstitution([
        pkg_share_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    declare_use_conveyor_cmd = DeclareLaunchArgument(
        name='use_conveyor',
        default_value='true',
        description='Enable the conveyor speed and object spawning node')

    # GUI and visualization arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='true',
        description='Flag to enable the RGBD camera for Gazebo point cloud simulation')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)')

    conveyor_dimension_args = [
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
    ]

    # Pose arguments
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # Include Robot State Publisher launch file if enabled
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': jsp_gui,
            'use_jsp': 'false',
            'use_camera': use_camera,
            # Use the fixed overhead RGB-D sensor for conveyor perception.
            'use_camera_sensor': 'false',
            'use_gazebo': use_gazebo,
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )

    # Include ROS 2 Controllers launch file if enabled
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit, 'launch', 'load_ros2_controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    # Set Gazebo model path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)

    def start_gazebo(context):
        selected_world = world_file.perform(context)
        resolved_world_path = os.path.join(pkg_share_gazebo, gazebo_worlds_path, selected_world)
        if selected_world == default_world_file:
            length = float(conveyor_length.perform(context))
            width = float(conveyor_width.perform(context))
            height = float(conveyor_height.perform(context))
            center_x = float(conveyor_center_x.perform(context))
            center_y = float(conveyor_center_y.perform(context))
            if length <= 0.3 or width <= 0.10 or height <= 0.05:
                raise RuntimeError(
                    'conveyor dimensions must exceed 0.3 m length, 0.10 m width, '
                    'and 0.05 m height')
            with open(resolved_world_path + '.in', 'r', encoding='utf-8') as template:
                sdf = template.read()
            replacements = {
                '@@CONVEYOR_CENTER_X@@': center_x,
                '@@CONVEYOR_CENTER_Y@@': center_y,
                '@@CONVEYOR_HEIGHT@@': height,
                '@@CAMERA_HEIGHT@@': height + 0.8,
                '@@YOLO_CAMERA_Y@@': center_y - width / 2.0 - 0.30,
                '@@YOLO_STAND_HEIGHT@@': height + 0.25,
                '@@YOLO_STAND_CENTER_Z@@': (height + 0.25) / 2.0,
                '@@YOLO_CAMERA_LOCAL_Z@@': (height + 0.25) / 2.0,
                '@@BOX_X@@': float(collection_box_x.perform(context)),
                '@@BOX_Y@@': float(collection_box_y.perform(context)),
                '@@BOX_HEIGHT_HALF@@': float(collection_box_height.perform(context)) / 2.0,
                '@@BOX_FLOOR_Z@@': -float(collection_box_height.perform(context)) / 2.0 + 0.01,
                '@@BOX_LENGTH@@': float(collection_box_length.perform(context)),
                '@@BOX_WIDTH@@': float(collection_box_width.perform(context)),
                '@@BOX_HEIGHT@@': float(collection_box_height.perform(context)),
                '@@BOX_WALL_X@@': float(collection_box_length.perform(context)) / 2.0 - 0.01,
                '@@BOX_WALL_Y@@': float(collection_box_width.perform(context)) / 2.0 - 0.01,
                '@@CONVEYOR_LENGTH@@': length,
                '@@CONVEYOR_WIDTH@@': width,
                '@@CONVEYOR_RAIL_Y@@': width / 2.0 + 0.02,
                '@@CONVEYOR_RAIL_LENGTH@@': length + 0.04,
                '@@CONVEYOR_LEG_X@@': length * 0.325,
                '@@CONVEYOR_LEG_Y@@': width * 0.35,
                '@@CONVEYOR_LEG_HEIGHT@@': max(0.08, height - 0.04),
                '@@CONVEYOR_LEG_Z@@': -(height + 0.04) / 2.0,
                '@@CONVEYOR_ROLLER_X@@': length / 2.0 - 0.045,
            }
            for placeholder, value in replacements.items():
                sdf = sdf.replace(placeholder, f'{value:.4f}')
            if '@@' in sdf:
                raise RuntimeError('Unexpanded conveyor dimension token in world SDF')
            resolved_world_path = os.path.join(
                tempfile.gettempdir(), f'cobot_conveyor_{os.getpid()}.sdf')
            with open(resolved_world_path, 'w', encoding='utf-8') as output:
                output.write(sdf)

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments=[('gz_args', [' -r -v 4 ', resolved_world_path])],
        )]

    def start_overhead_camera_tf(context):
        if use_camera.perform(context).lower() != 'true':
            return []
        center_x = conveyor_center_x.perform(context)
        center_y = float(conveyor_center_y.perform(context))
        width = float(conveyor_width.perform(context))
        conveyor_height_value = float(conveyor_height.perform(context))
        side_y = center_y - width / 2.0 - 0.30
        camera_nodes = [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', conveyor_center_x.perform(context),
                '--y', str(center_y),
                '--z', str(conveyor_height_value + 0.8),
                '--qx', '0.70710678', '--qy', '-0.70710678',
                '--qz', '0.0', '--qw', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'camera_head_color_optical_frame',
            ],
            output='screen',
        ), Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', center_x, '--y', str(center_y),
                '--z', str(conveyor_height_value + 0.8),
                '--qx', '0.70710678', '--qy', '-0.70710678',
                '--qz', '0.0', '--qw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'camera_ir_optical_frame',
            ],
            output='screen',
        ), Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', center_x, '--y', str(side_y),
                '--z', str(conveyor_height_value + 0.25),
                '--qx', '-0.70710678', '--qy', '0.0',
                '--qz', '0.0', '--qw', '0.70710678',
                '--frame-id', 'world', '--child-frame-id', 'camera_yolo_color_optical_frame',
            ],
            output='screen',
        )]
        return camera_nodes

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   'config_file:=' + default_ros_gz_bridge_config_file_path],
        output='screen'
    )

    # Includes optimizations to minimize latency and bandwidth when streaming image data
    start_gazebo_ros_image_bridge_cmd = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=[
        '/camera_head/depth_image',
        '/camera_head/image',
        '/camera_ir/image',
        '/camera_yolo/depth_image',
        '/camera_yolo/image',
    ],
    remappings=[
        ('/camera_head/depth_image',
         '/camera_head/depth/image_rect_raw'),
        ('/camera_head/image',
         '/camera_head/color/image_raw'),
        ('/camera_ir/image', '/camera_ir/image'),
        ('/camera_yolo/depth_image', '/camera_yolo/depth/image_rect_raw'),
        ('/camera_yolo/image', '/camera_yolo/color/image_raw'),
    ],
    condition=IfCondition(use_camera),
    output='screen',
)

    # ROS service bridge used by conveyor_manager to spawn physical objects.
    start_spawn_service_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/default/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen',
        condition=IfCondition(use_conveyor),
    )

    def start_conveyor_manager(context):
        length = float(conveyor_length.perform(context))
        height = float(conveyor_height.perform(context))
        center_x = float(conveyor_center_x.perform(context))
        center_y = float(conveyor_center_y.perform(context))
        with open(os.path.join(pkg_share_gazebo, 'config', 'conveyor.yaml'),
                  'r', encoding='utf-8') as stream:
            parameters = yaml.safe_load(stream)['conveyor_manager']['ros__parameters']
        parameters['spawn_x'] = center_x - length / 2.0 + 0.035
        parameters['spawn_y'] = center_y
        parameters['spawn_z'] = height + 0.04 + parameters['object_dimensions'][2] / 2.0
        return [Node(
            package='cobot_perception',
            executable='conveyor_manager',
            name='conveyor_manager',
            output='screen',
            parameters=[parameters, {'use_sim_time': use_sim_time}],
        )]
    
    

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])

    delayed_bridge_cmd = TimerAction(period=1.0, actions=[start_gazebo_ros_bridge_cmd])
    delayed_spawn_cmd = TimerAction(period=3.0, actions=[start_gazebo_ros_spawner_cmd])
    delayed_conveyor_manager_cmd = TimerAction(
        period=5.0,
        actions=[start_spawn_service_bridge_cmd,
                 OpaqueFunction(function=start_conveyor_manager)],
        condition=IfCondition(use_conveyor),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_conveyor_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    for argument in conveyor_dimension_args:
        ld.add_action(argument)

    # Add pose arguments
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add the actions to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    ld.add_action(OpaqueFunction(function=start_gazebo))
    ld.add_action(OpaqueFunction(function=start_overhead_camera_tf))
    ld.add_action(delayed_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(delayed_spawn_cmd)
    ld.add_action(delayed_conveyor_manager_cmd)

    return ld
