# Use ROS 2 Humble base image
FROM ros:humble

# Set up the working directory in the container
WORKDIR /workspace

# Install dependencies for ROS 2 and your package
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS 2 workspace into the container
COPY ./ros_ex_ws /workspace

# Install the ROS 2 workspace dependencies
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the ROS 2 setup script and the workspace setup script
CMD ["/bin/bash"]
