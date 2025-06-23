import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    run_headless = LaunchConfiguration("run_headless")

    # Correctly use PathJoinSubstitution for launching SLAM toolbox
    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ]),
            "use_sim_time:=True",
            ["slam_params_file:=", LaunchConfiguration('params_file3')]
        ],
        shell=False,
        output="screen",
    )

    # Navigation launch
    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ]),
            "use_sim_time:=True",
            ["params_file:=", LaunchConfiguration('params_file')]
        ],
        shell=False,
        output="screen",
    )

    # Use PathJoinSubstitution here instead of os.path.join
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("beetlebot_algo"),
                "launch",
                "bringup.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "True",
            "params_file": LaunchConfiguration('params_file3'),
        }.items(),
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("beetlebot_gazebo"),
                "config",
                "nav2_params.yaml"
            ]),
            description="Full path to the ROS2 parameters file for nav2"
        ),

        DeclareLaunchArgument(
            "params_file2",
            default_value=PathJoinSubstitution([
                FindPackageShare("beetlebot_gazebo"),
                "config",
                "slam_params.yaml"
            ]),
            description="Full path to the SLAM parameters file"
        ),

        DeclareLaunchArgument(
            "params_file3",
            default_value="/home/sonieth2/ros/ros2/projects/pose-graph-slam/beetlebot/src/beetlebot_bringup/config/slam_nav2.yaml",
            # default_value=PathJoinSubstitution([
            #     FindPackageShare("beetlebot_bringup"),
            #     "config",
            #     "slam_nav2.yaml"
            # ]),
            description="Full path to the combined SLAM/Nav2 parameters"
        ),

        DeclareLaunchArgument(
            "run_headless",
            default_value="False",
            description="Start GZ in headless mode and don't start RViz"
        ),

        # Include launch files
        bringup,
        # toolbox,
        # navigation
    ])