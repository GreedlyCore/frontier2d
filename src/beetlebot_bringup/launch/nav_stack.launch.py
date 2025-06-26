import os
from launch import LaunchDescription
# from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

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
            "params_file3",
            default_value="/home/sonieth2/ros/ros2/projects/practice_3d_year/frontier2d/src/beetlebot_bringup/config/slam_nav2.yaml",
            # default_value="src/beetlebot_bringup/config/slam_nav2.yaml",
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
    ])