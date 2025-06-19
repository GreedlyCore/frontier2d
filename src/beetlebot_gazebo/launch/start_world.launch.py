import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)

def generate_launch_description():
    # Configure ROS nodes for launch
    package_description = "beetlebot_description"
    package_gazebo = "beetlebot_gazebo"

    # Setup project paths
    pkg_project_gazebo = get_package_share_directory('beetlebot_gazebo')
    package_directory_description = get_package_share_directory(package_description)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim
    install_dir_path = get_package_prefix(package_description) + "/share"
    install_gazebo_dir_path = get_package_prefix(package_gazebo) + "/share"
    gazebo_resource_paths = [install_dir_path, install_gazebo_dir_path]

    # Update environment variables for Gazebo
    for env_var in ["GZ_SIM_RESOURCE_PATH", "GZ_SIM_MODEL_PATH", "SDF_PATH"]:
        if env_var in os.environ:
            for resource_path in gazebo_resource_paths:
                if resource_path not in os.environ[env_var]:
                    os.environ[env_var] += (':' + resource_path)
        else:
            os.environ[env_var] = (':'.join(gazebo_resource_paths))


    # Setup to launch the simulator and Gazebo world
    # '-r ' -- disable auto-pause when gz started
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r ',
            PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                'warehouse_world.sdf'
            ])
        ]}.items(),
    )

    return LaunchDescription([
        gz_sim,    
    ])