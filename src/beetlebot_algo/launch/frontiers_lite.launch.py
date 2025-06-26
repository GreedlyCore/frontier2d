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
    
    # pkg_project_gazebo = get_package_share_directory('beetlebot_gazebo')
    
 
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

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('beetlebot_bringup'),
                'launch',
                'nav_stack.launch.py'
            ])
        ])
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

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    explore_launch = Node(
        package="explore_lite",
        name="explore_node",
        # namespace=namespace,
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
        # arguments=['--ros-args', '--log-level', 'DEBUG' ]
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', 'src/beetlebot_gazebo/rviz/frontier_lite_rviz.rviz'],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_namespace_argument,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gazebo_launch,
        TimerAction(
            period=2.0,
            actions=[
                # nav_launch,
                explore_launch,
                rviz
            ]
        ),        
    ])

