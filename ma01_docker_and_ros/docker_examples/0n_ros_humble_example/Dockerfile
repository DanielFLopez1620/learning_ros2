# Our footprint the Open Source Robotics Foundation image for ROS 2 humble
FROM docker.io/osrf/ros:humble-desktop-full

# Add maintainer info
LABEL maintainer="DanielFLopez1620 <dfelipe.lopez@gmail.com"

# Argument for user
ARG USERNAME=ros2_user

# User Identifier (1000 - 10000 for application accounts)
ARG USER_UID=1000 

# Group identifier (100 + for the user's group)
ARG USER_GID=$USER_UID

# Create the user and give proper permissions
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update system package
RUN apt-get update && apt-get upgrade -y

# Install python package manager
RUN apt-get install -y python3-pip

# Set enviromental variable for calling shell
ENV SHELL=/bin/bash

# Configure Colcon Mixin
RUN colcon mixin remove default
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Install Gazebo
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-gz

# Setup CycloneDDS
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install TurtleBot3
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-turtlebot3*

# Configure User
USER $USERNAME
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros2_user/.bashrc
RUN echo "source /home/ros2_user/ws/install/setup.bash || echo 'Workspace not Ready. Run colcon build at /home/ros2_user/ws'" >> /home/ros2_user/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros2_user/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/ros2_user/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> /home/ros2_user/.bashrc
