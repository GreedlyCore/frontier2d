#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import random

random_x_around = round(random.uniform(-1, 1), 2)
random_y_around = round(random.uniform(-1, 1), 2)

def generate_launch_description():
    # Declare launch arguments
    beetlebot_name_arg = DeclareLaunchArgument(
        'beetlebot_name',
        default_value='beetlebot',
        description='Name of the robot'
    )
    
    robot_file_arg = DeclareLaunchArgument(
        'robot_file',
        default_value='beetlebot.urdf',
        description='Robot URDF file name'
    )
    
    x_spawn_arg = DeclareLaunchArgument(
        'x_spawn',
        default_value=str(random_x_around),
        description='X coordinate for robot spawn'
    )
    
    y_spawn_arg = DeclareLaunchArgument(
        'y_spawn',
        default_value=str(random_y_around),
        description='Y coordinate for robot spawn'
    )
    
    z_spawn_arg = DeclareLaunchArgument(
        'z_spawn',
        default_value='0.0',
        description='Z coordinate for robot spawn'
    )
    
    roll_spawn_arg = DeclareLaunchArgument(
        'roll_spawn',
        default_value='0.0',
        description='Roll angle for robot spawn'
    )
    
    pitch_spawn_arg = DeclareLaunchArgument(
        'pitch_spawn',
        default_value='0.0',
        description='Pitch angle for robot spawn'
    )
    
    yaw_spawn_arg = DeclareLaunchArgument(
        'yaw_spawn',
        default_value='0.0',
        description='Yaw angle for robot spawn'
    )
    
    # Get launch configurations
    beetlebot_name = LaunchConfiguration('beetlebot_name')
    robot_file = LaunchConfiguration('robot_file')
    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    z_spawn = LaunchConfiguration('z_spawn')
    roll_spawn = LaunchConfiguration('roll_spawn')
    pitch_spawn = LaunchConfiguration('pitch_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')
    
    # Robot state publisher launch
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'beetlebot_name': beetlebot_name,
            'robot_file': robot_file,
        }.items()
    )
    
    # Gazebo spawn launch
    spawn_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_gazebo'),
                'launch',
                'spawn_in_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'beetlebot_name': beetlebot_name,
            'x_spawn': x_spawn,
            'y_spawn': y_spawn,
            'z_spawn': z_spawn,
            'roll_spawn': roll_spawn,
            'pitch_spawn': pitch_spawn,
            'yaw_spawn': yaw_spawn,
        }.items()
    )
    
    # Static odom to world launch
    static_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_gazebo'),
                'launch',
                'static_odom_to_world.launch.py'
            ])
        ]),
        launch_arguments={
            'beetlebot_name': beetlebot_name,
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        beetlebot_name_arg,
        robot_file_arg,
        x_spawn_arg,
        y_spawn_arg,
        z_spawn_arg,
        roll_spawn_arg,
        pitch_spawn_arg,
        yaw_spawn_arg,
        
        # Launch includes
        robot_state_publisher_launch,
        spawn_gazebo_launch,
        static_odom_launch,
    ])