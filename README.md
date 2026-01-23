# ROS Control Panel (PyQt5)

A lightweight GUI to connect to a ROSBridge server, publish wheel commands from keyboard input, and control a robot arm via sliders or preset buttons defined in keyboard.yaml.

Note: Virtual environment setup is automatic. Scripts bootstrap a venv and install requirements as needed; no manual venv steps required.

## Quick Start

Follow these steps in order:

1) Build the Docker image for ROSBridge (ROS 2 Humble):

    ```bash
    python Docker/build.py
    ```

This creates or uses a Dockerfile at docker/Dockerfile.rosbridge-humble and builds an image tagged rosbridge:humble by default. Useful flags:

```bash
python Docker/build.py 
```

2) Start ROSBridge in Docker:

    ```bash
    python start_rosbridge_docker.py
    ```

Defaults:
- Image: rosbridge:humble
- Container name: rosbridge_humble
- Host port: 9090 (mapped to container 9090)

3) Launch the GUI and connect to ROSBridge:

    ```bash
    python main.py
    ```

Enter the ROSBridge IP (Docker host IP) and port (default from YAML is 9090), click Connect, then use the keyboard and sliders/presets to control the robot.

## YAML Configuration

Edit keyboard.yaml to change topics, keyboard mappings, joint limits, and presets.

- ros_port: Default ROSBridge port shown in the GUI.
- ros_topics.wheel: Topic for wheel speeds (std_msgs/Float32MultiArray).
- ros_topics.arm: Topic for arm joint commands (trajectory_msgs/JointTrajectoryPoint).
- ros_topics.arm_angles_float32multiarray: Topic publishing the same arm angles as an array (std_msgs/Float32MultiArray).
- arm_publish_in_radians: true to convert degrees to radians before publishing arm topics; false to publish degrees directly.
- key_mappings: Key → [v1, v2, v3, v4] list for wheel speeds. Positive/negative values set direction; list length should match expected wheels.
- arm_joint_limits: Per-joint min/max and default angle (degrees). Used to build GUI sliders and enforce ranges.
- arm_joint_angle_presets: Named poses → 8-angle lists (degrees). GUI creates buttons for each preset name and publishes on click.

## Notes

- The GUI auto-creates a venv and installs requirements from requirements.txt on first run.
- To stop ROSBridge, press Ctrl+C in the terminal where start_rosbridge_docker.py is running.
