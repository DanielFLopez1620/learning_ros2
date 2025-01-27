FROM osrf/ros:humble-desktop-full

# Create a workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Set up workspace
RUN ["/bin/bash", "-c",  "source /opt/ros/humble/setup.bash && colcon build"]

# Run a talker node
CMD ["ros2 run demo_nodes_cpp talker"]