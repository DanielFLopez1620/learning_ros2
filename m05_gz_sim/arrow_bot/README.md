# Build and run container
```
cd docker/iron # it can be other ROS version
bash run_basic.bash # build time about 1 hour. Have 20 GB of free space
```

After container is builded connect to it (for example with the Portainer program).


# Commands in container
## Run gazebo with model
```
. ~/.profile
ros2 launch arrow_bot gazebo.launch.py
```

## Run gazebo with model and ros2_controllers
```
. ~/.profile
ros2 launch arrow_bot ros2_control.launch.py
```

## Choose of map
```
. ~/.profile
GZ_SIM_WORLD=empty ros2 launch arrow_bot ros2_control.launch.py
GZ_SIM_WORLD=cars_and_trees ros2 launch arrow_bot ros2_control.launch.py
GZ_SIM_WORLD=empty ros2 launch arrow_bot gazebo.launch.py
GZ_SIM_WORLD=cars_and_trees ros2 launch arrow_bot gazebo.launch.py
```
cars_and_trees map will download included objects from internet firt run (it can take time)

# Rebuild ros package (in case of reexport from RobotCAD)
```
bash run_basic.bash -r
```

# Troubleshooting
Segmentation fault - posible reasons:
- using fixed joint as not fixed. Check all joints types.

rqt crash - posible reasons:
- not set velocity limits of controlled joints by joint_trajectory_controller (divide by zero error)