import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configure ROS nodes for launch
    package_gazebo = "beetlebot_gazebo"
    # Setup project paths
    pkg_project_gazebo = get_package_share_directory(package_gazebo)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # example for argument providing --- 
    #     gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('beetlebot_gazebo'),
    #             'launch',
    #             'world.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'world': 'big_warehouse.sdf'}.items()
    # )
    # SELECT FROM -- 'big_warehouse.sdf' 'warehouse_world.sdf' 'empty_world.sdf', 'house.sdf'
    select_world = DeclareLaunchArgument(
        'world',
        default_value='warehouse_world.sdf',
        description='SDF world file to load from the beetlebot_gazebo/worlds folder'
    )

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
                LaunchConfiguration('world')
            ])
        ]}.items(),
    )
    
    return LaunchDescription([
        select_world,
        gz_sim,
    ])