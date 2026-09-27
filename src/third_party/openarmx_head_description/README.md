# openarmx_head_description

English | [中文](./README-CN.md)

---

![Cover](./image/cover.gif)


URDF/xacro model description for the OpenArmX 2-DOF head (yaw + pitch).

## Overview

This package provides the kinematic model (URDF via xacro) and visual meshes for the OpenArmX head. The head has two revolute joints driven by Robstride RS00 motors:

- **Pitch joint** (`openarmx_head_pitch_joint`): Nod up/down, bottom motor, rotates around X-axis
- **Yaw joint** (`openarmx_head_yaw_joint`): Turn left/right, upper motor, rotates around Z-axis

## Kinematic Structure

```
world (fixed)
  └── head_base_link (neck mount, neck bracket, neck limit, bottom motor RS00-2 body)
        └── [pitch joint] openarmx_head_pitch_joint (revolute, axis X)
              └── head_pitch_link (neck cover, upper motor RS00-1, head connector, guard plate)
                    └── [yaw joint] openarmx_head_yaw_joint (revolute, axis Z)
                          └── head_yaw_link (head bracket, head limit, front/rear shells, camera)
```

## Joint Limits

| Joint | Min | Max | Max Velocity | Max Effort |
|-------|-----|-----|--------------|------------|
| `openarmx_head_yaw_joint` | -90 deg (-1.5708 rad) | +90 deg (+1.5708 rad) | 33.0 rad/s | 14.0 Nm |
| `openarmx_head_pitch_joint` | -62.2 deg (-1.086 rad) | +23.1 deg (+0.403 rad) | 33.0 rad/s | 14.0 Nm |

## Xacro Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `can_interface` | `can2` | CAN bus interface |
| `use_fake_hardware` | `false` | Use simulated hardware plugin |
| `can_fd` | `false` | Enable CAN-FD |
| `control_mode` | `mit` | Motor control mode: `mit` or `csp` |
| `home_on_activate` | `true` | Slowly move to zero position on activation |
| `home_duration_sec` | `3.0` | Duration for homing motion (seconds) |

## ros2_control Integration

The xacro includes a ros2_control system interface definition (`head.ros2_control.xacro`) that configures:

- Plugin: `openarmx_head_hardware/OpenArmX_HeadHW` (or `fake_components/GenericSystem` when simulating)
- Command interfaces: position, velocity, effort (per joint)
- State interfaces: position, velocity, effort (per joint)

## File Structure

```
openarmx_head_description/
├── urdf/
│   ├── head.urdf.xacro              # Main URDF model
│   └── ros2_control/
│       └── head.ros2_control.xacro   # ros2_control hardware definition
├── config/
│   └── joint_limits.yaml             # Joint limit parameters
└── meshes/
    └── visual/                       # STL mesh files for visualization
```

## Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_description
source install/setup.bash
```

## Usage

This package is typically used as a dependency by `openarmx_head_bringup`. To generate the URDF manually:

```bash
xacro $(ros2 pkg prefix openarmx_head_description)/share/openarmx_head_description/urdf/head.urdf.xacro
```

## Dependencies

- `robot_state_publisher`
- `xacro`

## Notes

- Meshes are in STL format, originally in millimeters. The xacro applies a scale factor of 0.001 to convert to meters.
- The mesh coordinate system uses a +pi/2 rotation around X to convert from the CAD convention (Y=up) to the URDF convention (Z=up).

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
