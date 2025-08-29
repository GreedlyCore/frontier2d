import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    declare_world_name = DeclareLaunchArgument(
        'world', default_value='big_warehouse.sdf',
        description='World file name'
    )
    declare_robot_x = DeclareLaunchArgument(
        'x', default_value='1.0',
        description='Initial robot X position'
    )
    declare_robot_y = DeclareLaunchArgument(
        'y', default_value='1.0',
        description='Initial robot Y position'
    )
    declare_robot_z = DeclareLaunchArgument(
        'z', default_value='0.1',
        description='Initial robot Z position'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation/Gazebo clock'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', 
        default_value=PathJoinSubstitution([
            FindPackageShare('beetlebot_gazebo'),
            'rviz',
            'frontier_lite_rviz.rviz'
        ]),
        # default_value=PathJoinSubstitution([
        #     FindPackageShare('beetlebot_gazebo'),
        #     'rviz',
        #     'frontier_lite_rviz.rviz'
        # ]),
        description='RViz config file path'
    )
    
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_gazebo'),
                'launch',
                'world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_description'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_name,
        declare_robot_x,
        declare_robot_y,
        declare_robot_z,
        declare_use_sim_time,
        declare_rviz,
        declare_rviz_config,
        
        world_launch,
        robot_launch,
        rviz,
    ])