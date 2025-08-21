#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package directories
    pkg_jetracer_description = FindPackageShare('jetracer_description')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

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

    # Simple WASD Controller Node
    wasd_controller_node = Node(
        package='jetracer_description',
        executable='simple_wasd_controller',
        name='simple_wasd_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': 1.0,     # m/s
            'angular_speed': 2.0,    # rad/s
            'update_rate': 30.0      # Hz
        }]
    )

    # RViz with working config
    rviz_config_file = PathJoinSubstitution([
        pkg_jetracer_description, 'config', 'jetracer_working.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add the nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(wasd_controller_node)
    ld.add_action(rviz_node)

    return ld
