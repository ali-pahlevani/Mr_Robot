# lift_slide_description

English | [中文](./README-CN.md)

---

![Cover](./image/cover.gif)


URDF/Xacro description package for the lift-slide mechanism. Provides the robot model, mesh files, and a standalone visualization launch file.

## Features

- Parameterized Xacro model with configurable joint limits
- Optional ros2_control hardware interface macro
- STL mesh files for all links
- Standalone display launch for visualization without hardware

## Package Contents

| Path | Description |
|------|-------------|
| `urdf/lift_slide_module.urdf.xacro` | Main robot model |
| `urdf/lift_slide_ros2_control.urdf.xacro` | ros2_control hardware configuration macro |
| `meshes/` | STL mesh files (lift_link, plate_link, chest_link, left/right_link0_base) |
| `rviz/display.rviz` | RViz configuration for visualization |
| `launch/display.launch.py` | Standalone visualization launch |

## Links and Joints

| Link | Description |
|------|-------------|
| `lift_base_link` | Fixed base column |
| `lift_carriage_link` | Moving carriage/plate |
| `chest_link` | Chest enclosure (fixed to carriage) |
| `left_link0_base` | Left arm mount point (fixed to carriage) |
| `right_link0_base` | Right arm mount point (fixed to carriage) |

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| `lift_joint` | prismatic | Z | software range -0.750 to 0.400 m |
| `chest_joint` | fixed | - | - |
| `left_joint0_base` | fixed | - | - |
| `right_joint0_base` | fixed | - | - |

## Launch (Visualization Only)

```bash
ros2 launch lift_slide_description display.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_joint_state_gui` | `true` | Launch joint_state_publisher_gui |
| `start_rviz` | `true` | Launch RViz |
| `min_height` | `-0.750` | Software minimum position (m) |
| `max_height` | `0.400` | Software maximum position (m) |

## TF Frames Published

`lift_base_link` -> `lift_carriage_link` -> `chest_link`, `left_link0_base`, `right_link0_base`

## Build

```bash
colcon build --packages-select lift_slide_description
source install/setup.bash
```

## Prerequisites

- ROS 2 (Humble/Iron)
- xacro
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
