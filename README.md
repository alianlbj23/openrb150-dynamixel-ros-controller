# ROS Control Panel (PyQt5)

A lightweight GUI to connect to a ROSBridge server, publish wheel commands from keyboard input, and control a robot arm via sliders or preset buttons defined in `keyboard.yaml`.

## Features

- Connect/disconnect to ROSBridge by IP and port
- Publish wheel speeds from keyboard shortcuts
- Publish arm joint angles in radians from sliders
- One-click arm presets from YAML
- Joint reset to default positions

## Requirements

- Python 3.x
- Packages in `requirements.txt`

## Setup

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Configuration

Edit `keyboard.yaml` to change topics, keyboard mappings, joint limits, and presets.

Example sections:

- `ros_port`: default ROSBridge port
- `ros_topics.wheel`: wheel command topic (Float32MultiArray)
- `ros_topics.arm`: arm command topic (JointTrajectoryPoint)
- `key_mappings`: keyboard key -> list of wheel speeds (degrees are not used here)
- `arm_joint_limits`: slider ranges and default values (degrees)
- `arm_joint_angle_presets`: button name -> list of joint angles (degrees)

## Usage

1. Launch the app and enter ROSBridge IP + port.
2. Click **Connect** to start publishing.
3. Use keyboard keys from `key_mappings` to publish wheel speeds.
4. Adjust sliders to publish arm joint angles (degrees converted to radians).
5. Click a preset button to publish the preset joint angles (degrees converted to radians).
6. Click **Reset Joints** to move to default angles.

## Build (optional)

`build.py` creates a single-file executable using PyInstaller:

```bash
python build.py
```

The output is placed in `dist/`.
