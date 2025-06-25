#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro
import yaml

def replace_namespace(d, old_str, new_str):
    if isinstance(d, dict):
        for key, value in d.items():
            d[key] = replace_namespace(value, old_str, new_str)
    elif isinstance(d, list):
        for i in range(len(d)):
            d[i] = replace_namespace(d[i], old_str, new_str)
    elif isinstance(d, str):
        return d.replace(old_str, new_str)
    return d

def replace_namespace_in_params(yaml_file_path, namespace):
    # if not namespace.startswith('/'):
    #     namespace = '/' + namespace

    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Replace namespace with 'namespace'
    data = replace_namespace(data, 'namespace', namespace)
    # Convert the modified data back to YAML format
    modified_yaml = yaml.dump(data, default_flow_style=False)
    # Write the modified YAML to the output file
    path = f"/tmp/{namespace}.yaml"
    with open(path, 'w') as file:
        file.write(modified_yaml)

    print(f"Modified YAML has been saved to {path}")
    return path

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'rb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': '0.3'},
        {'name': 'rb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': '0.3'},
    ]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('beetlebot_gazebo'), "worlds")])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    xacro_desc = os.path.join(
        get_package_share_directory('beetlebot_description'),
        'urdf', 'beetlebot.xacro')

    world_file = os.path.join(get_package_share_directory('beetlebot_gazebo'), "worlds", "warehouse_world.sdf")

    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', f'-r -v1 {world_file}')
        ]
    )
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gz)
 
    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    ld.add_action(clock_bridge)

    last_action = None
    # Spawn robot instances in gazebo
    for robot in robots:

        namespace = '/' + robot['name'] 
        namespace2 = robot['name'] + '/'

        doc = xacro.parse(open(xacro_desc))
        xacro.process_doc(doc, mappings={"namespace": namespace2})
        robot_description = {"robot_description": doc.toxml()}

        print(f'\n\n {doc.toxml()} \n\n')
        
        # Create state publisher node for that instance
        robot_state_publisher = Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='both',
            parameters=[   robot_description   ]
            # parameters=[
            #     {'use_sim_time': True},
            #     {'robot_description': robot_description},
            # ]
        )
        
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            name="my_robot_spawn",
            arguments=[
                "-name", robot['name'],
                '-topic', f"{namespace}/robot_description",
                '-robot_namespace', namespace,
                    '-x', robot['x_pose'],
                   '-y', robot['y_pose'],
                   '-z', robot['z_pose']],
            output="screen",
        )
        
        bridge_file = os.path.join(get_package_share_directory('beetlebot_gazebo'), 'config', 'beetlebot_ros_bridge_namespace.yaml')
        namespaced_bridge_file = replace_namespace_in_params(bridge_file, robot['name'])
        
        # import yaml
        # with open(namespaced_bridge_file) as stream:
        #     try:
        #         print(); print(yaml.safe_load(stream));   print()
        #     except yaml.YAMLError as exc:
        #         print(exc)
        
        # ROS-Gazebo Bridge
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            parameters=[{
                'config_file': namespaced_bridge_file,
                # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'use_sim_time': True
            }],
            output='screen'
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(robot_state_publisher)
            ld.add_action(spawn_robot)
            ld.add_action(bridge)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        spawn_robot,
                        robot_state_publisher,
                        bridge
                    ],
                )
            )

            ld.add_action(spawn_robot_event)
        last_action = spawn_robot
    return ld
