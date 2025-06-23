# Beetlebot 


```
├── beetlebot_algo
│   ├── config
│   ├── include
│   │   └── beetlebot_algo
│   ├── launch
│   ├── scripts
│   │   └── __pycache__
│   └── src
├── beetlebot_description
│   ├── beetlebot_description
│   ├── config
│   ├── launch
│   ├── meshes
│   ├── resource
│   ├── test
│   └── urdf
├── beetlebot_gazebo
│   ├── beetlebot_gazebo
│   ├── config
│   ├── launch
│   ├── models
│   │   ├── house
│   │   ├── track1
│   │   └── track2
│   ├── resource
│   ├── rviz
│   ├── test
│   └── worlds
│       └── sub_models
├── imgs
└── robot_localization
    ├── doc
    │   └── images
    ├── include
    │   └── robot_localization
    ├── launch
    ├── params
    ├── src
    ├── srv
    └── test
```

frontiers_lite.launch.py -- contains a basic exploration with classic algo

metrics

time

quality

check papers and look for using them

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb2/cmd_vel

ros2 run tf2_tools view_frames --ros-args --remap tf:=/rb1/tf --remap tf_static:=/rb1/tf_static


ros2 run nav2_map_server map_saver_cli -f ~/map

## Related papers

## Ack