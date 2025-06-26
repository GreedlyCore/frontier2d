# Explore via SLAM-Toolbox


## Structure of summary project (sim-real)
```
├── maps
├── src
│   ├── beetlebot_bringup
│   │   ├── config
│   │   ├── include
│   │   ├── launch
│   │   ├── rviz
│   │   └── src
│   ├── beetlebot_description
│   │   ├── beetlebot_description
│   │   ├── launch
│   │   ├── meshes
│   │   ├── resource
│   │   ├── test
│   │   └── urdf
│   ├── beetlebot_gazebo
│   │   ├── beetlebot_gazebo
│   │   ├── config
│   │   ├── launch
│   │   ├── models
│   │   ├── resource
│   │   ├── rviz
│   │   └── worlds
│   ├── explore
│   │   ├── config
│   │   ├── doc
│   │   ├── include
│   │   ├── launch
│   │   └── src
│   ├── map_merge
│   │   ├── config
│   │   ├── doc
│   │   ├── include
│   │   ├── launch
│   │   ├── src
│   │   └── test
│   ├── pyexplore
│   │   ├── include
│   │   ├── launch
│   │   ├── scripts
│   │   └── src
│   └── robot_localization
│       ├── doc
│       ├── include
│       ├── launch
│       ├── params
│       ├── src
│       ├── srv
│       └── test
├── tf_logs
└── worlds
```

So you need a SLAM-Toolbox && Nav2 full stack preinstalled before using a package.

frontiers_lite.launch.py -- contains a basic exploration with classic algo

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb2/cmd_vel

Listen the TF of robot1:
```
ros2 run tf2_tools view_frames --ros-args --remap tf:=/rb1/tf --remap tf_static:=/rb1/tf_static
```

Save current map:
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```

If building the workspace freezes linux (usually: you're run out of RAM, only reboot will save you)
```
colcon build --symlink-install --executor sequential
```

Written report paper in russian: