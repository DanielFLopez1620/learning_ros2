#!/bin/bash

# Obtain and display the value of the secret
export ROS2_SECRET=$(cat /run/secrets/ros2_pw)
echo "ROS 2 Secret: $ROS2_SECRET"

# Make a ROS 2 execution
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker