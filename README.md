## Python version of expore_lite 
Adapted for ROS2 (slam-toolbox + nav2) with some heuristics for faster exploration

### Structure of summary project 
That workspace need to be created on your own, contains a few external packages
```
```

External packages:
- SLAM-Toolbox
- Nav2
- CycloneDDS


## How to launch different versions
```
ros2 launch beetlebot_bringup pyfrontiers.launch.py
```



<!-- frontiers_lite.launch.py -- contains a basic exploration with classic algo

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb2/cmd_vel

Listen the TF of robot1:
```
ros2 run tf2_tools view_frames --ros-args --remap tf:=/rb1/tf --remap tf_static:=/rb1/tf_static
```

If building the workspace freezes linux (usually: you're run out of RAM, only reboot will save you)
```
colcon build --symlink-install --executor sequential
``` -->




### Metrics evaluation
There's a timer that saves current robot map every 20 seconds. Otherwise you can do it manually:
```
ros2 run nav2_map_server map_saver_cli -f /maps
```
With automatically map saving it provides two more variables - explored area (pix^2), time passed and total travelled distance by robot.
We can use this data to compare approaches.
Final data will be saved to folders with timestamp names.

### Experiments

### Report paper
later
<!-- for now it's availiable in russian only -->

### TODO-list

## TODO

- [ ] Multi-robot exploration support
- [ ] Recovery behaviour for exploration using nav2 BT or my own
- [ ] State machine using BT instead of many functions
- [ ] Autonomous actors instead of pre-defined with path (enable-disable with launch file too)
- [ ] Odometry noisy model with align
- [x] return robot_localization pkg 
<!-- ```
sudo apt install ros-humble-ament-lint-auto
``` -->