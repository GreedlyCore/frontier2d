import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    frontend_node = Node(
        package='beetlebot_algo',
        executable='frontend.py',
        name='graph_slam_frontend',
        output='screen'
    )
    mapping_node = Node(
        package='beetlebot_algo',
        executable='LidarMapping.py',
        name='graph_slam_mapping',
        output='screen'
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
        ])
    )
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("beetlebot_algo"), 'config', 'ekf.yaml')],
           )

    # Node(
    #     package='scan_matching',
    #     executable='scan_matching',
    #     name='scan_matching_node',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # ),

    return LaunchDescription([
        TimerAction(
            period=12.0,
            actions=[
                # frontend_node,
                # mapping_node
                nav_launch
            ]
        ),
        gazebo_launch,
        TimerAction(
            period=10.0,
            actions=[
                odom_noisy_node,
                ekf_node
            ]
        ),
    ])

