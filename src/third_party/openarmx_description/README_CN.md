# OpenArmX Description

[English](./README.md) | 中文

---

[English](README.md) | 简体中文

[![ROS 2](https://img.shields.io/badge/ROS-2-blue.svg)](https://docs.ros.org/en/humble/index.html)

OpenArmX 机器人平台的完整 URDF 描述包，为 ROS 2 仿真和控制提供详细的运动学、动力学和可视化模型。

## 概述

`openarmx_description` 包包含定义 OpenArmX 机器人完整机械结构、运动学、动力学和可视化表示的 URDF（统一机器人描述格式）文件。该包是 ROS 2 环境中机器人可视化、运动规划、仿真和硬件控制的基础。

## 功能特性

- **完整的机器人模型**：具有精确运动学和动力学参数的完整 OpenArmX 机器人 URDF 描述
- **模块化架构**：机械臂、本体和末端执行器组件的独立描述，支持灵活的机器人配置
- **碰撞几何体**：详细的碰撞网格，用于安全的运动规划和避障
- **可视化模型**：高质量的 STL/DAE 网格，实现逼真的 3D 可视化
- **ROS 2 控制集成**：为仿真和真实硬件预配置的 ros2_control 硬件接口
- **双臂支持**：原生支持双臂机器人配置
- **多种机器人变体**：支持不同的机械臂类型（v10）和末端执行器（OpenArmX Hand）
- **参数化配置**：基于 YAML 的运动学、动力学和关节限制配置文件

## 包结构

```
openarmx_description/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS 2 包清单
├── LICENSE                     # Apache 2.0 许可证
├── config/                     # 机器人参数配置
│   ├── arm/                    # 机械臂特定参数
│   │   └── v10/                # v10 机械臂配置文件
│   │       ├── inertials.yaml          # 连杆质量和惯性属性
│   │       ├── joint_limits.yaml       # 关节位置/速度/力矩限制
│   │       ├── kinematics.yaml         # DH 参数和变换
│   │       ├── kinematics_link.yaml    # 连杆坐标系定义
│   │       └── kinematics_offset.yaml  # 关节零点偏移校准
│   ├── body/                   # 本体/躯干参数
│   │   └── v10/                # v10 本体配置
│   └── hand/                   # 末端执行器参数
│       └── openarmx_hand/      # OpenArmX Hand 夹爪配置
├── launch/                     # 可视化启动文件
│   └── display_openarmx.launch.py  # 在 RViz 中启动机器人可视化
├── meshes/                     # 3D 网格文件（STL/DAE）
│   ├── arm/                    # 机械臂连杆网格
│   │   └── v10/
│   │       ├── collision/      # 简化碰撞几何体
│   │       └── visual/         # 详细可视化网格
│   ├── body/                   # 本体/躯干网格
│   │   └── v10/
│   │       ├── collision/
│   │       └── visual/
│   └── ee/                     # 末端执行器网格
│       └── openarmx_hand/
│           ├── collision/
│           └── visual/
├── rviz/                       # RViz 配置文件
│   ├── arm_only.rviz          # 单臂可视化配置
│   └── bimanual.rviz          # 双臂可视化配置
└── urdf/                       # URDF/Xacro 描述文件
    ├── arm/                    # 机械臂 URDF 组件
    │   ├── openarmx_arm.xacro        # 主机械臂描述
    │   └── openarmx_macro.xacro      # 机械臂 xacro 宏
    ├── body/                   # 本体/躯干 URDF 组件
    │   ├── openarmx_body.xacro       # 主本体描述
    │   └── openarmx_body_macro.xacro # 本体 xacro 宏
    ├── ee/                     # 末端执行器 URDF 组件
    │   ├── openarmx_hand.xacro       # OpenArmX Hand 描述
    │   ├── openarmx_hand_macro.xacro # Hand xacro 宏
    │   ├── openarmx_hand_arguments.xacro  # Hand 参数
    │   └── ee_with_one_link.xacro    # 通用末端执行器连接
    ├── robot/                  # 完整机器人组装
    │   ├── openarmx_robot.xacro      # 通用机器人宏
    │   ├── v10.urdf.xacro            # v10 机器人变体
    │   └── openarmx_bimanual_sim.urdf # 预生成的双臂 URDF
    └── ros2_control/           # ROS 2 控制配置
        ├── openarmx.ros2_control.xacro         # 单臂控制配置
        └── openarmx.bimanual.ros2_control.xacro # 双臂控制配置
```

## 安装

### 前置要求

- ROS 2（Humble 或更高版本）
- Python 3.10+
- `xacro` 包
- `joint_state_publisher_gui` 包
- `rviz2` 包

### 从源码构建

```bash
# 导航到 ROS 2 工作空间

---
cd ~/openarmx_ws/src

# 克隆仓库

---
git clone https://github.com/openarmx-arm/openarmx_description.git

# 构建包

---
colcon build --packages-select openarmx_description

# 设置环境

---
source install/setup.bash
```

## 使用方法

### 在 RViz 中可视化机器人

使用默认配置启动机器人可视化：

```bash
ros2 launch openarmx_description display_openarmx.launch.py arm_type:=v10 bimanual:=true
```

#### 启动参数

- `arm_type`（必需）：要可视化的机械臂类型
  - `v10`：7 自由度 OpenArmX v10 机械臂

- `ee_type`（默认值：`openarmx_hand`）：末端执行器类型
  - `openarmx_hand`：OpenArmX Hand 夹爪
  - `none`：无末端执行器

- `bimanual`（必需：`false`）：启用双臂配置
  - `true`：加载带有两个机械臂的双臂机器人
  - `false`：仅加载单个机械臂


### 从 Xacro 生成 URDF

将 xacro 文件转换为 URDF 以进行检查：

```bash
xacro $(ros2 pkg prefix openarmx_description)/share/openarmx_description/urdf/robot/v10.urdf.xacro \
    arm_type:=v10 ee_type:=openarmx_hand bimanual:=true > /home/openarmx/openarmx/src/openarmx_description/urdf/robot/openarmx_robot.urdf
```

## 支持的机器人配置

### 机械臂类型

| 类型 | 自由度 | 描述 |
|------|--------|------|
| v10  | 7      | OpenArmX v10 - 7 自由度协作机械臂，载荷 5kg |

### 末端执行器

| 类型 | 描述 |
|------|------|
| openarmx_hand | 带位置控制的平行夹爪 |
| none | 无末端执行器（仅法兰） |

### 配置模式

- **单臂**：一个机械臂，可选末端执行器
- **双臂**：具有同步控制的双臂系统

## 配置文件

该包使用 YAML 配置文件定义机器人参数：

- **`inertials.yaml`**：每个连杆的质量、质心和惯性张量
- **`joint_limits.yaml`**：每个关节的位置、速度和力矩限制
- **`kinematics.yaml`**：正运动学参数（DH 约定）
- **`kinematics_link.yaml`**：连杆到连杆的变换定义
- **`kinematics_offset.yaml`**：关节零位校准偏移

## ROS 2 控制集成

该包包含预配置的 ros2_control 硬件接口：

- **位置控制器**：关节轨迹控制
- **速度控制器**：直接速度命令
- **力矩控制器**：基于扭矩的控制
- **夹爪控制器**：末端执行器控制

使用 `use_fake_hardware:=true` 进行仿真，或使用 `use_fake_hardware:=false` 进行真实硬件控制。

## 开发

### 添加新的机器人变体

1. 创建配置目录：`config/arm/your_variant/`
2. 添加所需的 YAML 文件：`inertials.yaml`、`joint_limits.yaml` 等
3. 创建网格：`meshes/arm/your_variant/{collision,visual}/`
4. 创建 xacro 文件：`urdf/robot/your_variant.urdf.xacro`
5. 更新启动文件以支持新变体

### 修改机器人参数

可以通过编辑 `config/` 目录中的 YAML 文件来调整机器人参数。修改后，重新构建工作空间：

```bash
colcon build --packages-select openarmx_description
```

## 故障排除

### RViz 不显示机器人模型
- 确保包已正确设置环境：`source install/setup.bash`
- 检查 `arm_type` 参数是否与可用配置匹配
- 验证 URDF 生成：`ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro path/to/file.xacro)"`

### 网格文件未加载
- 验证 xacro 文件中的网格路径使用 `package://` URI 方案
- 确保网格已安装：检查 `install/openarmx_description/share/openarmx_description/meshes/`

### 仿真中违反关节限制
- 检查并调整 `config/arm/*/joint_limits.yaml` 中的限制
- 确保控制器在其配置中遵守关节限制

## 贡献

欢迎贡献！请遵循以下指南：

1. Fork 仓库
2. 创建功能分支
3. 使用清晰的提交消息进行更改
4. 使用不同配置进行全面测试
5. 提交 Pull Request

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **Zhang Li** (张力)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：6.0.0

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
