# Odrive hardware interface

This package is a non-official ROS package for the Odrive S1 hardware interface using can communication and ROS2 Control. Currently, the system is able to communicate through the can bus with the Odrive and send commands to the motors using only effort (torque) commands.

## Installation:

```bash
rosdep install --from-paths . -y --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## Usage:

```bash
ros2 launch odrive_hw_interface odrive.launch.py
```

Then, in another terminal:

```bash
ros2 launch odrive_hw_interface odrive_forward_position_controller.launch.py
```
