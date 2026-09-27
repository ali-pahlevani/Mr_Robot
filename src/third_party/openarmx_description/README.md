# OpenArmX Description

English | [中文](./README_CN.md)

---

English | [简体中文](README_CN.md)

[![ROS 2](https://img.shields.io/badge/ROS-2-blue.svg)](https://docs.ros.org/en/humble/index.html)

A comprehensive URDF description package for the OpenArmX robot platform, providing detailed kinematic, dynamic, and visual models for ROS 2 simulation and control.

## Overview

The `openarmx_description` package contains URDF (Unified Robot Description Format) files that define the complete mechanical structure, kinematics, dynamics, and visual representation of OpenArmX robots. This package serves as the foundation for robot visualization, motion planning, simulation, and hardware control in ROS 2 environments.

## Features

- **Complete Robot Models**: Full URDF descriptions for OpenArmX robots with accurate kinematic and dynamic parameters
- **Modular Architecture**: Separate descriptions for arm, body, and end-effector components enabling flexible robot configurations
- **Collision Geometry**: Detailed collision meshes for safe motion planning and obstacle avoidance
- **Visual Models**: High-quality STL/DAE meshes for realistic 3D visualization
- **ROS 2 Control Integration**: Pre-configured ros2_control hardware interfaces for both simulation and real hardware
- **Dual-Arm Support**: Native support for bimanual robot configurations
- **Multiple Robot Variants**: Support for different arm types (v10) and end-effectors (OpenArmX Hand)
- **Parametric Configuration**: YAML-based configuration files for kinematics, dynamics, and joint limits

## Package Structure

```
openarmx_description/
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS 2 package manifest
├── LICENSE                     # Apache 2.0 license
├── config/                     # Robot parameter configurations
│   ├── arm/                    # Arm-specific parameters
│   │   └── v10/                # v10 arm configuration files
│   │       ├── inertials.yaml          # Link mass and inertia properties
│   │       ├── joint_limits.yaml       # Joint position/velocity/effort limits
│   │       ├── kinematics.yaml         # DH parameters and transforms
│   │       ├── kinematics_link.yaml    # Link frame definitions
│   │       └── kinematics_offset.yaml  # Joint offset calibration
│   ├── body/                   # Body/torso parameters
│   │   └── v10/                # v10 body configuration
│   └── hand/                   # End-effector parameters
│       └── openarmx_hand/      # OpenArmX Hand gripper configuration
├── launch/                     # Launch files for visualization
│   └── display_openarmx.launch.py  # Launch robot visualization in RViz
├── meshes/                     # 3D mesh files (STL/DAE)
│   ├── arm/                    # Arm link meshes
│   │   └── v10/
│   │       ├── collision/      # Simplified collision geometry
│   │       └── visual/         # Detailed visual meshes
│   ├── body/                   # Body/torso meshes
│   │   └── v10/
│   │       ├── collision/
│   │       └── visual/
│   └── ee/                     # End-effector meshes
│       └── openarmx_hand/
│           ├── collision/
│           └── visual/
├── rviz/                       # RViz configuration files
│   ├── arm_only.rviz          # Single arm visualization config
│   └── bimanual.rviz          # Dual-arm visualization config
└── urdf/                       # URDF/Xacro description files
    ├── arm/                    # Arm URDF components
    │   ├── openarmx_arm.xacro        # Main arm description
    │   └── openarmx_macro.xacro      # Arm xacro macros
    ├── body/                   # Body/torso URDF components
    │   ├── openarmx_body.xacro       # Main body description
    │   └── openarmx_body_macro.xacro # Body xacro macros
    ├── ee/                     # End-effector URDF components
    │   ├── openarmx_hand.xacro       # OpenArmX Hand description
    │   ├── openarmx_hand_macro.xacro # Hand xacro macros
    │   ├── openarmx_hand_arguments.xacro  # Hand parameters
    │   └── ee_with_one_link.xacro    # Generic EE attachment
    ├── robot/                  # Complete robot assemblies
    │   ├── openarmx_robot.xacro      # Generic robot macro
    │   ├── v10.urdf.xacro            # v10 robot variant
    │   └── openarmx_bimanual_sim.urdf # Pre-generated bimanual URDF
    └── ros2_control/           # ROS 2 Control configurations
        ├── openarmx.ros2_control.xacro         # Single arm control config
        └── openarmx.bimanual.ros2_control.xacro # Dual-arm control config
```

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- `xacro` package
- `joint_state_publisher_gui` package
- `rviz2` package

### Building from Source

```bash
# Navigate to your ROS 2 workspace

---
cd ~/openarmx_ws/src

# Clone the repository

---
git clone https://github.com/openarmx-arm/openarmx_description.git

# Build the package

---
colcon build --packages-select openarmx_description

# Source the workspace

---
source install/setup.bash
```

## Usage

### Visualizing the Robot in RViz

Launch the robot visualization with default configuration:

```bash
ros2 launch openarmx_description display_openarmx.launch.py arm_type:=v10 bimanual:=true
```

#### Launch Arguments

- `arm_type` (required): Type of arm to visualize
  - `v10`: 7-DOF OpenArmX v10 arm

- `ee_type` (default: `openarmx_hand`): End-effector type
  - `openarmx_hand`: OpenArmX Hand gripper
  - `none`: No end-effector

- `bimanual` (default: `false`): Enable dual-arm configuration
  - `true`: Load bimanual robot with two arms
  - `false`: Load single arm only


### Generating URDF from Xacro

To convert xacro files to URDF for inspection:

```bash
xacro $(ros2 pkg prefix openarmx_description)/share/openarmx_description/urdf/robot/v10.urdf.xacro \
    arm_type:=v10 ee_type:=openarmx_hand bimanual:=true > robot_bimanual.urdf
```

## Supported Robot Configurations

### Arm Types

| Type | DOF | Description |
|------|-----|-------------|
| v10  | 7   | OpenArmX v10 - 7-DOF collaborative arm with 5kg payload |

### End-Effectors

| Type | Description |
|------|-------------|
| openarmx_hand | Parallel jaw gripper with position control |
| none | No end-effector (flange only) |

### Configuration Modes

- **Single Arm**: One manipulator with optional end-effector
- **Bimanual**: Dual-arm system with synchronized control

## Configuration Files

The package uses YAML configuration files to define robot parameters:

- **`inertials.yaml`**: Mass, center of mass, and inertia tensor for each link
- **`joint_limits.yaml`**: Position, velocity, and effort limits for each joint
- **`kinematics.yaml`**: Forward kinematics parameters (DH convention)
- **`kinematics_link.yaml`**: Link-to-link transformation definitions
- **`kinematics_offset.yaml`**: Joint zero-position calibration offsets

## ROS 2 Control Integration

The package includes pre-configured ros2_control hardware interfaces:

- **Position controllers**: Joint trajectory control
- **Velocity controllers**: Direct velocity commands
- **Effort controllers**: Torque-based control
- **Gripper controllers**: End-effector control

Use `use_fake_hardware:=true` for simulation or `use_fake_hardware:=false` for real hardware control.

## Development

### Adding a New Robot Variant

1. Create configuration directory: `config/arm/your_variant/`
2. Add required YAML files: `inertials.yaml`, `joint_limits.yaml`, etc.
3. Create meshes: `meshes/arm/your_variant/{collision,visual}/`
4. Create xacro file: `urdf/robot/your_variant.urdf.xacro`
5. Update launch files to support the new variant

### Modifying Robot Parameters

Robot parameters can be tuned by editing YAML files in the `config/` directory. After modifications, rebuild the workspace:

```bash
colcon build --packages-select openarmx_description
```

## Troubleshooting

### RViz shows no robot model
- Ensure the package is properly sourced: `source install/setup.bash`
- Check that `arm_type` argument matches available configurations
- Verify URDF generation: `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro path/to/file.xacro)"`

### Mesh files not loading
- Verify mesh paths in xacro files use `package://` URI scheme
- Ensure meshes are installed: check `install/openarmx_description/share/openarmx_description/meshes/`

### Joint limits violated in simulation
- Check and adjust limits in `config/arm/*/joint_limits.yaml`
- Ensure controllers respect joint limits in their configuration

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Make your changes with clear commit messages
4. Test thoroughly with different configurations
5. Submit a pull request

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Zhang Li** (张力)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 6.0.0

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
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
