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
    
    pkg_project_gazebo = get_package_share_directory('beetlebot_gazebo')
    
    explore_launch = Node(
        package='pyexplore',
        executable='explore_node.py',
        name='explore_node',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_gazebo'),
                'launch',
                'beetlebot_warehouse.launch.py'
            ])
        ]),
        launch_arguments={'world': 'big_warehouse.sdf'}.items()
    )
    odom_noisy_node = Node(
        package='beetlebot_algo',
        executable='odom_noisy.py',
        name='noisy_odometry_node',
        output='screen'
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_bringup'),
                'launch',
                'nav_stack.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("beetlebot_algo"), 'config', 'ekf.yaml')],
           )

    # RViz
    # essential for use FindPackageShare instead of `rviz_config_file = os.path.join(pkg_project_gazebo, 'rviz', 'frontier_rviz')`
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', 'src/beetlebot_gazebo/rviz/pyfrontier_rviz.rviz'], #  'beetlebot.rviz' 
    #    arguments=['-d', LaunchConfiguration("rvizconfig")], #  'beetlebot.rviz' 
       parameters=[{'use_sim_time': True}],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=PathJoinSubstitution([
                FindPackageShare("beetlebot_gazebo"),
                "rviz/navigation_config.rviz",
            ]),
            description="Absolute path to rviz config file" #TODO: make it real absolute, not from install
        ),
        gazebo_launch,
        explore_launch,
        rviz,
        TimerAction(
            period=3.0,
            actions=[
                nav_launch,
            ])
    ])