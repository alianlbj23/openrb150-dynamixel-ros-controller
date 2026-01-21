FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
