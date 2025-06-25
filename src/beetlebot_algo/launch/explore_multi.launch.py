#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, TimerAction
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

def replace_namespace_in_params(yaml_file_path, namespace, name='default'):
    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Replace namespace with 'namespace'
    data = replace_namespace(data, 'namespace', namespace)
    # Convert the modified data back to YAML format
    modified_yaml = yaml.dump(data, default_flow_style=False)
    # Write the modified YAML to the output file
    path = f"/tmp/{namespace}_{name}.yaml"
    with open(path, 'w') as file:
        file.write(modified_yaml)

    print(f"Modified YAML has been saved to {path}")
    return path

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots (reduced to 2 for clear setup)
    robots = [
        {'name': 'rb1', 'x_pose': '0', 'y_pose': '0', 'z_pose': '0.3'},
        {'name': 'rb2', 'x_pose': '0', 'y_pose': '10', 'z_pose': '0.3'},
        # ...
        # {'name': 'rb2', 'x_pose': '0', 'y_pose': '10', 'z_pose': '0.3'},
    ]

    # Environment setup
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('beetlebot_gazebo'), "worlds")])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true', description='Use simulator time'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz for visualization'
    )

    # File paths
    xacro_desc = os.path.join(
        get_package_share_directory('beetlebot_description'),
        'urdf', 'beetlebot.xacro')
    
    # "warehouse_world.sdf"
    world_file = os.path.join(get_package_share_directory('beetlebot_gazebo'), "worlds", "big_warehouse.sdf")
    nav_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    params_file = os.path.join(get_package_share_directory('beetlebot_bringup'), 'config', 'slam_nav2_namespace.yaml')

    # Gazebo launch
    # ign_gz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ]),
    #     launch_arguments=[
    #         ('gz_args', f'-r -v1 {world_file}')
    #     ]
    # ) 
    ign_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v1 ' +
                              world_file
                             ])])
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gz)
 
    # Clock bridge for Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )
    ld.add_action(clock_bridge)

    # Remapping for tf
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    
    # Spawn robots with nav2-slam
    for robot in robots:
        namespace = '/' + robot['name'] 
        namespace2 = robot['name'] + '/'

        # Remapping for tf
        # remappings = [('/tf', 'tf' + namespace), ('/tf_static', 'tf_static' + namespace)]

        
        # Process XACRO
        doc = xacro.parse(open(xacro_desc))
        xacro.process_doc(doc, mappings={"namespace": namespace2})
        robot_description = {"robot_description": doc.toxml()}

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='both',
            parameters=[robot_description],
        )

        # Spawn robot in Gazebo
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

        # Create namespaced parameter file
        namespaced_param_file = replace_namespace_in_params(params_file, robot['name'], 'bringup')

        print(f'\n\n {namespaced_param_file} \n\n')
        
        import yaml
        with open(namespaced_param_file) as stream:
            try:
                print(yaml.safe_load(stream))
            except yaml.YAMLError as exc:
                print(exc)


        # Nav2 bringup with SLAM
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')),
                # os.path.join(get_package_share_directory('beetlebot_algo'), 'launch' , 'bringup.launch.py')),
            launch_arguments={  
                'slam': 'True',
                # 'namespace': namespace,
                'namespace': robot['name'],
                'use_namespace': 'True',
                'map': '',
                'params_file': namespaced_param_file,
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'log_level': 'info', #'debug'
                'use_composition': 'False'
            }.items()
        )
        
        # bringup_cmd = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare("beetlebot_algo"),
        #             "launch",
        #             "bringup.launch.py"
        #         ])
        #     ),
        #     launch_arguments={
        #         "use_sim_time": "True",
        #         "slam": "True",
        #         'use_sim_time': use_sim_time,
        #         "params_file": namespaced_param_file,
        #     }.items(),
        # )

        # ROS-Gazebo Bridge
        bridge_file = os.path.join(get_package_share_directory('beetlebot_gazebo'), 'config', 'beetlebot_ros_bridge_namespace.yaml')
        namespaced_bridge_file = replace_namespace_in_params(bridge_file, robot['name'], 'bridge')
        
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

        # Exploration node for each robot
        explore_config = os.path.join(
            get_package_share_directory("beetlebot_algo"), "config", "explore_params.yaml"
        )
        
        explore_node = Node(
            package="explore_lite",
            name="explore_node",
            namespace=namespace,
            executable="explore",
            parameters=[explore_config, {"use_sim_time": use_sim_time}],
            output="screen",
            remappings=remappings,
        )

        if last_action is None:
            # Add actions directly for the first robot
            ld.add_action(robot_state_publisher)
            ld.add_action(spawn_robot)
            ld.add_action(bringup_cmd)
            ld.add_action(bridge)
            ld.add_action(explore_node)
        else:
            # Use RegisterEventHandler for subsequent robots
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        robot_state_publisher,
                        spawn_robot,
                        bringup_cmd,
                        bridge,
                        explore_node
                    ],
                )
            )
            ld.add_action(spawn_robot_event)
        
        last_action = spawn_robot

    
        ######################
    # # Start rviz nodes and drive nodes after the last robot is spawned
    # for robot in robots:

    #     namespace = '/' + robot['name']

    #     # # Create a initial pose topic publish call
    #     # message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
    #     #     robot['x_pose'] + ', y: ' + robot['y_pose'] + \
    #     #     ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

    #     # initial_pose_cmd = ExecuteProcess(
    #     #     cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
    #     #         'geometry_msgs/PoseWithCovarianceStamped', message],
    #     #     output='screen'
    #     # )

    #     # RViz for visualization
    #     rviz_config_file = "src/beetlebot_bringup/rviz/multi.rviz"
    #     rviz_cmd = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(nav_launch_dir, 'rviz_launch.py')),
    #             launch_arguments={'use_sim_time': use_sim_time, 
    #                               'namespace': namespace,
    #                               'use_namespace': 'True',
    #                               'rviz_config': rviz_config_file, 'log_level': 'warn'}.items()
    #                                 )

    #     # Use RegisterEventHandler to ensure next robot rviz launch happens 
    #     # only after all robots are spawned
    #     post_spawn_event = RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=last_action,
    #             on_exit=[rviz_cmd],
    #             # on_exit=[initial_pose_cmd, rviz_cmd],
    #         )
    #     )

    #     # Perform next rviz and other node instantiation after the previous intialpose request done
    #     # last_action = initial_pose_cmd

    #     ld.add_action(post_spawn_event)
    # ######################    
    
    map_merge_config = os.path.join(
        get_package_share_directory("beetlebot_bringup"), "config", "merge.yaml"
    )
    
    map_merge_node = Node(
        package="multirobot_map_merge",
        name="map_merge",
        executable="map_merge",
        parameters=[
            map_merge_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=remappings,
    )
    
    ld.add_action(map_merge_node)

    return ld