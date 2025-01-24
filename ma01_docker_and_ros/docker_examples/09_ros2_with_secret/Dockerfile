FROM osrf/ros:jazzy-desktop-full

# Add workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Copy bash file to test the secret
COPY entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

# Execute
ENTRYPOINT ["/ros2_ws/entrypoint.sh"]