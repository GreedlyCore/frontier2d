import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():    
    
    explore_launch = Node(
        package='pyexplore',
        executable='explore_node.py',
        name='explore_node',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        explore_launch
    ])