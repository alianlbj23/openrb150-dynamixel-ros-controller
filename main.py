#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import threading
import subprocess
import hashlib
import math
import time
from pathlib import Path

# ----------------------------
# VENV BOOTSTRAP (auto venv + requirements + re-exec)
# ----------------------------

def _in_venv() -> bool:
    return getattr(sys, "base_prefix", sys.prefix) != sys.prefix


def _venv_python_path(venv_dir: str) -> str:
    if os.name == "nt":
        return os.path.join(venv_dir, "Scripts", "python.exe")
    return os.path.join(venv_dir, "bin", "python")


def _run_checked(cmd: list[str]) -> None:
    print("[BOOTSTRAP]>", " ".join(cmd))
    subprocess.check_call(cmd)


def _ensure_pip(python_exe: str) -> None:
    try:
        subprocess.check_call(
            [python_exe, "-m", "pip", "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        _run_checked([python_exe, "-m", "ensurepip", "--upgrade"])
        _run_checked([python_exe, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel"])


def _hash_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _ensure_venv_and_reexec():
    if _in_venv():
        return

    base_dir = os.path.dirname(
        sys.executable if getattr(sys, "frozen", False) else os.path.abspath(__file__)
    )
    venv_dir = os.path.join(base_dir, ".venv")
    req_path = os.path.join(base_dir, "requirements.txt")
    hash_path = os.path.join(venv_dir, ".requirements_hash")

    if not os.path.isdir(venv_dir):
        _run_checked([sys.executable, "-m", "venv", venv_dir])

    venv_python = _venv_python_path(venv_dir)
    _ensure_pip(venv_python)

    if os.path.exists(req_path):
        req_hash = _hash_file(req_path)
        old_hash = ""
        if os.path.exists(hash_path):
            try:
                with open(hash_path, "r", encoding="utf-8") as f:
                    old_hash = f.read().strip()
            except Exception:
                old_hash = ""

        if req_hash != old_hash:
            _run_checked([venv_python, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel"])
            _run_checked([venv_python, "-m", "pip", "install", "-r", req_path])
            with open(hash_path, "w", encoding="utf-8") as f:
                f.write(req_hash)
    else:
        print("[BOOTSTRAP][WARN] requirements.txt not found; running with venv anyway.")

    os.execv(venv_python, [venv_python, os.path.abspath(__file__), *sys.argv[1:]])


_ensure_venv_and_reexec()

# ----------------------------
# Regular imports (after venv ready)
# ----------------------------
import yaml
import roslibpy
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QMessageBox,
    QSlider,
    QFormLayout,
    QScrollArea,
)
from PyQt5.QtCore import Qt


class IPInputWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.connected = False
        self.current_ip = ""
        self.wheel_pub = None  # roslibpy.Topic for wheel
        self.arm_pub = None  # roslibpy.Topic for arm joints
        self.arm_angles_float32multiarray_pub = None  # roslibpy.Topic for arm angle array
        self.ros = None  # roslibpy.Ros object

        base_dir = os.path.dirname(sys.executable if getattr(sys, "frozen", False) else __file__)
        yaml_path = os.path.join(base_dir, "keyboard.yaml")

        with open(yaml_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
            self.key_map = config.get("key_mappings", {})
            self.joint_limits = config.get("arm_joint_limits", {})
            self.arm_presets = config.get("arm_joint_angle_presets", {})
            self.rosbridge_port = int(config.get("ros_port", 9090))
            self.arm_publish_in_radians = bool(config.get("arm_publish_in_radians", True))

            ros_topics = config.get("ros_topics", {})
            self.wheel_topic = ros_topics.get("wheel", "/car_C_rear_wheel")
            self.arm_topic = ros_topics.get("arm", "/robot_arm")
            self.arm_angles_float32multiarray_topic = ros_topics.get("arm_angles_float32multiarray", "/robot_arm_angles_float32multiarray")

        self.joint_sliders: dict[str, QSlider] = {}
        self.joint_labels: dict[str, QLabel] = {}
        self._joint_timer: threading.Timer | None = None

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS Control Panel")
        self.setFixedSize(600, 600)

        # IP input
        ip_label = QLabel("ROS Master IP:", self)
        self.ip_edit = QLineEdit(self)
        self.ip_edit.setPlaceholderText("e.g. 192.168.0.10")
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(ip_label)
        ip_layout.addWidget(self.ip_edit)

        # Port input
        port_label = QLabel("ROSBridge Port:", self)
        self.port_edit = QLineEdit(self)
        self.port_edit.setText(str(self.rosbridge_port))
        port_layout = QHBoxLayout()
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_edit)

        # Connect button
        self.btn_connect = QPushButton("Connect", self)
        self.btn_connect.clicked.connect(self.on_connect_click)

        # Current connection label
        self.current_ip_label = QLabel("", self)
        self.current_ip_label.setVisible(False)

        # Key display
        self.key_label = QLabel("Press a key to control the robot", self)
        self.key_label.setAlignment(Qt.AlignCenter)
        self.key_label.setStyleSheet("font-size: 18px;")

        # Main layout
        layout = QVBoxLayout()
        layout.addLayout(ip_layout)
        layout.addLayout(port_layout)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.current_ip_label)
        layout.addWidget(self.key_label)

        # Reset joints button
        self.btn_reset_joints = QPushButton("Reset Joints", self)
        self.btn_reset_joints.clicked.connect(self.reset_all_joint_sliders)
        self.btn_reset_joints.setVisible(False)
        layout.addWidget(self.btn_reset_joints)

        # Preset buttons from yaml
        self.preset_buttons_widget = QWidget()
        self.preset_buttons_layout = QHBoxLayout()
        self.preset_buttons_widget.setLayout(self.preset_buttons_layout)
        self.preset_buttons_widget.setVisible(False)
        for preset_name in self.arm_presets.keys():
            btn = QPushButton(preset_name, self)
            btn.clicked.connect(lambda _, name=preset_name: self.on_preset_clicked(name))
            self.preset_buttons_layout.addWidget(btn)
        layout.addWidget(self.preset_buttons_widget)

        # Joint sliders with scroll area
        self.form_layout_widget = QWidget()
        form_layout = QFormLayout()
        self.form_layout_widget.setLayout(form_layout)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.form_layout_widget)

        for joint_name, limits in self.joint_limits.items():
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(limits["min"]))
            slider.setMaximum(int(limits["max"]))
            slider.setValue(int(limits.get("default", (limits["min"] + limits["max"]) // 2)))

            label = QLabel(str(slider.value()))
            self.joint_labels[joint_name] = label

            h_layout = QHBoxLayout()
            h_layout.addWidget(slider)
            h_layout.addWidget(label)

            slider.valueChanged.connect(lambda value, name=joint_name: self.on_joint_slider_changed(name))

            self.joint_sliders[joint_name] = slider
            form_layout.addRow(f"{joint_name} ({limits['min']}~{limits['max']})", h_layout)

        self.form_layout_widget.setVisible(False)
        layout.addWidget(self.scroll_area)

        self.setLayout(layout)

    def _connect_rosbridge(self, ip: str, port: int, timeout: int = 5, max_retries: int = 3):
        for attempt in range(1, max_retries + 1):
            ros = roslibpy.Ros(host=ip, port=port)
            try:
                print(f"[INFO] Attempting to connect to ROSBridge (try {attempt}/{max_retries})...")
                ros.run(timeout=timeout)
                if ros.is_connected:
                    self.ros = ros

                    self.wheel_pub = roslibpy.Topic(self.ros, self.wheel_topic, "std_msgs/Float32MultiArray")
                    self.wheel_pub.advertise()

                    self.arm_pub = roslibpy.Topic(self.ros, self.arm_topic, "trajectory_msgs/JointTrajectoryPoint")
                    self.arm_pub.advertise()
                    self.arm_angles_float32multiarray_pub = roslibpy.Topic(
                        self.ros, self.arm_angles_float32multiarray_topic, "std_msgs/Float32MultiArray"
                    )
                    self.arm_angles_float32multiarray_pub.advertise()

                    print(f"[INFO] Connected to ROSBridge on attempt {attempt}")
                    return True, ""
                else:
                    print("[WARN] roslibpy connected=False")
            except Exception as e:
                print(f"[ERROR] ROSBridge connection failed on attempt {attempt}: {e}")
            time.sleep(1)
        return False, f"Failed to connect after {max_retries} attempts"

    def _disconnect_rosbridge(self):
        if self.wheel_pub:
            try:
                self.wheel_pub.unadvertise()
            except Exception as e:
                print(f"Error unadvertising wheel_pub: {e}")
            self.wheel_pub = None

        if self.arm_pub:
            try:
                self.arm_pub.unadvertise()
            except Exception as e:
                print(f"Error unadvertising arm_pub: {e}")
            self.arm_pub = None
        if self.arm_angles_float32multiarray_pub:
            try:
                self.arm_angles_float32multiarray_pub.unadvertise()
            except Exception as e:
                print(f"Error unadvertising arm_angles_float32multiarray_pub: {e}")
            self.arm_angles_float32multiarray_pub = None

        if self.ros and self.ros.is_connected:
            try:
                self.ros.terminate()
            except Exception as e:
                print(f"Error terminating ROS connection: {e}")
        self.ros = None
        print("[INFO] Disconnected from ROSBridge.")

    def on_connect_click(self):
        if not self.connected:
            ip = self.ip_edit.text().strip()
            port_text = self.port_edit.text().strip()

            if not self.validate_ip(ip):
                QMessageBox.warning(self, "Warning", "Invalid IP format.")
                return

            try:
                port = int(port_text)
            except ValueError:
                QMessageBox.warning(self, "Warning", "Invalid Port format. Must be a number.")
                return

            ok, err = self._connect_rosbridge(ip, port)
            if ok:
                self._set_connected(ip, port)
                QMessageBox.information(self, "Success", f"Connected to ROSBridge at ws://{ip}:{port}")
            else:
                QMessageBox.critical(self, "錯誤", f"rosbridge連線失敗: {err}")
        else:
            self._disconnect_rosbridge()
            self._set_disconnected()
            QMessageBox.information(self, "Info", "Disconnected from ROSBridge.")

    def _set_connected(self, ip: str, port: int):
        self.connected = True
        self.current_ip = ip
        self.ip_edit.setEnabled(False)
        self.port_edit.setEnabled(False)
        self.btn_connect.setText("Disconnect")
        self.current_ip_label.setText(f"Connected to: {ip}:{port}")
        self.current_ip_label.setVisible(True)
        self.form_layout_widget.setVisible(True)
        self.btn_reset_joints.setVisible(True)
        self.preset_buttons_widget.setVisible(True)
        self.key_label.setText("Use keyboard to control the robot")

    def _set_disconnected(self):
        self.connected = False
        self.current_ip = ""
        self.ip_edit.setEnabled(True)
        self.port_edit.setEnabled(True)
        self.btn_connect.setText("Connect")
        self.current_ip_label.setVisible(False)
        self.form_layout_widget.setVisible(False)
        self.btn_reset_joints.setVisible(False)
        self.preset_buttons_widget.setVisible(False)
        self.key_label.setText("Press a key to control the robot")

    def keyPressEvent(self, event):
        if self.connected:
            key = event.text()
            if key and key in self.key_map:
                self.key_label.setText(f"Key Pressed: {key}")
                wheel_speed = self.key_map[key]
                self.publish_wheel_speed(wheel_speed)
            elif key:
                self.key_label.setText(f"Key '{key}' not mapped.")

    def publish_wheel_speed(self, speeds):
        if not (self.ros and self.ros.is_connected and self.wheel_pub):
            print("[WARN] ROS not connected, skip wheel speed publish.")
            return
        msg = roslibpy.Message({"layout": {"dim": [], "data_offset": 0}, "data": list(map(float, speeds))})
        self.wheel_pub.publish(msg)

    def publish_robot_arm(self, joint_values):
        if not (self.ros and self.ros.is_connected and self.arm_pub):
            print("[WARN] ROS not connected, skip robot_arm publish.")
            return
        msg = roslibpy.Message(
            {
                "positions": list(map(float, joint_values)),
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {"secs": 0, "nsecs": 0},
            }
        )
        self.arm_pub.publish(msg)
        self.publish_arm_angles_float32multiarray_array(joint_values)

    def publish_arm_angles_float32multiarray_array(self, joint_values):
        if not (self.ros and self.ros.is_connected and self.arm_angles_float32multiarray_pub):
            print("[WARN] ROS not connected, skip arm_angles_float32multiarray publish.")
            return
        msg = roslibpy.Message({"layout": {"dim": [], "data_offset": 0}, "data": list(map(float, joint_values))})
        self.arm_angles_float32multiarray_pub.publish(msg)

    def send_joint_command(self):
        if not self.connected:
            return
        joint_values_deg = [self.joint_sliders[j].value() for j in sorted(self.joint_sliders.keys())]
        joint_values = self._convert_joint_values(joint_values_deg)
        self.publish_robot_arm(joint_values)

    def on_joint_slider_changed(self, joint_name):
        value = self.joint_sliders[joint_name].value()
        self.joint_labels[joint_name].setText(str(value))
        if self._joint_timer is not None:
            self._joint_timer.cancel()
        self._joint_timer = threading.Timer(0.1, self.send_joint_command)
        self._joint_timer.start()

    def reset_all_joint_sliders(self):
        for joint_name, slider in self.joint_sliders.items():
            default_val = int(self.joint_limits[joint_name].get("default", 0))
            slider.setValue(default_val)
            self.joint_labels[joint_name].setText(str(default_val))
        self.send_joint_command()

    def on_preset_clicked(self, preset_name):
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to ROSBridge.")
            return
        angles_deg = self.arm_presets.get(preset_name, [])
        if not angles_deg:
            QMessageBox.warning(self, "Warning", f"Preset '{preset_name}' is empty.")
            return
        angles = self._convert_joint_values(angles_deg)
        self.publish_robot_arm(angles)

        joint_names = sorted(self.joint_sliders.keys())
        if len(joint_names) == len(angles_deg):
            for joint_name, angle in zip(joint_names, angles_deg):
                self.joint_sliders[joint_name].setValue(int(angle))
                self.joint_labels[joint_name].setText(str(int(angle)))

    def closeEvent(self, event):
        self._disconnect_rosbridge()
        event.accept()

    def _convert_joint_values(self, values_deg):
        if self.arm_publish_in_radians:
            return [math.radians(v) for v in values_deg]
        return list(map(float, values_deg))

    @staticmethod
    def validate_ip(ip: str) -> bool:
        parts = ip.split(".")
        if len(parts) != 4:
            return False
        for p in parts:
            if not p.isdigit() or not 0 <= int(p) <= 255:
                return False
        return True


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IPInputWindow()
    window.show()
    sys.exit(app.exec_())
