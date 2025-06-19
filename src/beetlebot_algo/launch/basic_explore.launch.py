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
    
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_algo'),
                'launch',
                'explore.launch.py'
            ])
        ])
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
                FindPackageShare('beetlebot_gazebo'),
                'launch',
                'nav_stack.launch.py'
            ])
        ])
    )
    

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("beetlebot_algo"), 'config', 'ekf.yaml')],
           )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_gazebo, 'rviz', 'beetlebot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gazebo_launch,
        TimerAction(
            period=12.0,
            actions=[
                nav_launch,
                explore_launch,
                rviz
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                odom_noisy_node,
                # ekf_node
            ]
        ),
    ])

