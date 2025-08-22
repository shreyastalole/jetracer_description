#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package directories
    pkg_jetracer_description = FindPackageShare('jetracer_description')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Specify x position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Specify y position of the robot')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Specify z position of the robot')

    # Robot State Publisher
    urdf_file = PathJoinSubstitution([pkg_jetracer_description, 'urdf', 'jetracer.xacro'])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Gazebo Launch (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r']}.items()
    )

    # Robot spawner (delayed to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'jetracer',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose
                ],
                output='screen'
            )
        ]
    )

    # Bridge between ROS 2 and Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    # Simple Controller Node
    controller_node = Node(
        package='jetracer_description',
        executable='simple_controller',
        name='simple_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Teleop Twist Keyboard (delayed to start after everything else)
    teleop_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add the nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(bridge)
    ld.add_action(controller_node)
    ld.add_action(teleop_node)

    return ld
