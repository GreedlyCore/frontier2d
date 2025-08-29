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
    robotworld_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_bringup'),
                'launch',
                'robot_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': 'big_warehouse.sdf',
            'x': '1.0', 
            'y': '1.0', 
            'z': '0.1',
            'use_sim_time': 'true',
            'rviz': 'true',
            'rviz_config':'src/beetlebot_gazebo/rviz/frontier_lite_rviz.rviz'
        }.items()
    )
    
    nav_launch = IncludeLaunchDescription(
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
            "use_namespace": "False",
            "params_file": LaunchConfiguration('params_file_nav'),
        }.items(),
    )
    
    config = os.path.join(
        get_package_share_directory("beetlebot_bringup"), "config", "explore_params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )


    
    explore_launch = Node(
        package='pyexplore',
        executable='explore_node.py',
        name='explore_node',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )
    
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_namespace_argument,                # Declare launch arguments
        DeclareLaunchArgument(
            "params_file_nav",
            default_value="src/beetlebot_bringup/config/slam_nav2.yaml",
            # default_value=PathJoinSubstitution([
            #     FindPackageShare("beetlebot_bringup"),
            #     "config",
            #     "slam_nav2.yaml"
            # ]),
            description="Full path to the combined SLAM/Nav2 parameters"
        ),
        robotworld_launch, 
        TimerAction(
            period=2.0,
            actions=[
                nav_launch,
                explore_launch,
               
            ]
        ),        
    ])