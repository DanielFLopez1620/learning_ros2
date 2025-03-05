# Usage of controllers

## diff_drive
```
. ~/.profile
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```
Click on the open terminal and use the keys provided in the terminal to drive the car.

## diff_drive unstamped
```
. ~/.profile
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```
ROS2 Jazzy diff_drive_controller does not support cmd_vel_unstamped anymore. Use stamped instead (cmd_vel)
Click on the open terminal and use the keys provided in the terminal to drive the car.

## gripper

#### open gripper
```
. ~/.profile
ros2 action send_goal /gripper_action_controller_position/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.01, max_effort: 100}}"
```

#### close gripper
```
. ~/.profile
ros2 action send_goal /gripper_action_controller_position/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 100}}"
```

## joint_trajectory
Open rqt -> plugins (tab) -> Robot tools -> Joint trajectory controller
Select controller manager and controller, press - enable button.