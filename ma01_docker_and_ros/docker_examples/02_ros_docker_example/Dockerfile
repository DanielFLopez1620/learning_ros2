# Use ROS 2 Humble base image
FROM ros:humble

# Set up the working directory in the container
WORKDIR /workspace

# Install dependencies for ROS 2
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS 2 workspace into the container
COPY ./ros_ex_ws /workspace

# Install the ROS 2 workspace dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Source the ROS 2 setup script and the workspace setup script
# Set the entrypoint to source the workspace and run the ROS 2 node
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run brief_rclcpp float_pub"]

# Expose necessary ports for ROS 2
EXPOSE 11311 

# Bash call
CMD ["/bin/bash"]
