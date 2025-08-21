#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="jetracer_world.world",
            description="World file to load in Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    # Initialize Arguments
    world_file = LaunchConfiguration("world_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Robot description
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("jetracer_description"), "urdf", "jetracer.xacro"])
    ])
    
    robot_description = {"robot_description": robot_description_content}

    # Start Gazebo server
    start_gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "4",
             PathJoinSubstitution([FindPackageShare("jetracer_description"), "worlds", world_file])],
        output="screen",
    )

    # Start Gazebo client
    start_gazebo_client = ExecuteProcess(
        cmd=["gz", "sim", "-g", "-v", "4"],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        arguments=[
            "-topic", "/robot_description",
            "-name", "jetracer",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # ROS-Gazebo bridge for topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    nodes_to_start = [
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        bridge,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
