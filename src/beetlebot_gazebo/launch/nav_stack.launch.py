import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output



def generate_launch_description():
    run_headless = LaunchConfiguration("run_headless")
  
    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["slam_params_file:=", LaunchConfiguration('params_file2')]
        ],
        shell=False,
        output="screen",
    )
 
    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["params_file:=", LaunchConfiguration('params_file')]
        ],
        shell=False,
        output="screen",
    )
    rviz_node = Node(
        condition=IfCondition(NotSubstitution(run_headless)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )


    # waiting_success = RegisterEventHandler(
    #     OnProcessIO(
    #         target_action=navigation,
    #         on_stdout=on_matching_output(
    #             navigation_ready_message,
    #             [
    #                 LogInfo(msg="Ready for navigation!"),
    #             ],
    #         ),
    #     )
    # )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=[FindPackageShare("beetlebot_gazebo"), "/config/nav2_params.yaml"],
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                "params_file2",
                default_value=[FindPackageShare("beetlebot_gazebo"), "/config/slam_params.yaml"],
                description="Full path tsdfsdfsdfsd",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("beetlebot_gazebo"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            toolbox,
            navigation,
            rviz_node
        ])
