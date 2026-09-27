# openarmx_head_description

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


OpenArmX 2自由度头部（偏航 + 俯仰）的 URDF/xacro 模型描述包。

## 概述

本包提供 OpenArmX 头部的运动学模型（基于 xacro 的 URDF）和可视化网格文件。头部具有两个旋转关节，由 Robstride RS00 电机驱动：

- **俯仰关节** (`openarmx_head_pitch_joint`)：点头（上/下），底部电机，绕 X 轴旋转
- **偏航关节** (`openarmx_head_yaw_joint`)：摇头（左/右），上部电机，绕 Z 轴旋转

## 运动学结构

```
world（固定）
  └── head_base_link（颈部安装座、颈部支架、颈部限位、底部电机 RS00-2 本体）
        └── [俯仰关节] openarmx_head_pitch_joint（旋转，X 轴）
              └── head_pitch_link（颈部护罩、上部电机 RS00-1、连接件、护板）
                    └── [偏航关节] openarmx_head_yaw_joint（旋转，Z 轴）
                          └── head_yaw_link（头部支架、头部限位、前/后外壳、摄像头）
```

## 关节限位

| 关节 | 最小值 | 最大值 | 最大速度 | 最大力矩 |
|------|--------|--------|----------|----------|
| `openarmx_head_yaw_joint` | -90° (-1.5708 rad) | +90° (+1.5708 rad) | 33.0 rad/s | 14.0 Nm |
| `openarmx_head_pitch_joint` | -62.2° (-1.086 rad) | +23.1° (+0.403 rad) | 33.0 rad/s | 14.0 Nm |

## Xacro 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `can_interface` | `can2` | CAN 总线接口 |
| `use_fake_hardware` | `false` | 使用仿真硬件插件 |
| `can_fd` | `false` | 启用 CAN-FD |
| `control_mode` | `mit` | 电机控制模式：`mit` 或 `csp` |
| `home_on_activate` | `true` | 激活时缓慢移动到零位 |
| `home_duration_sec` | `3.0` | 回零运动持续时间（秒） |

## ros2_control 集成

xacro 包含 ros2_control 系统接口定义（`head.ros2_control.xacro`），配置内容：

- 插件：`openarmx_head_hardware/OpenArmX_HeadHW`（仿真时使用 `fake_components/GenericSystem`）
- 指令接口：position、velocity、effort（每个关节）
- 状态接口：position、velocity、effort（每个关节）

## 文件结构

```
openarmx_head_description/
├── urdf/
│   ├── head.urdf.xacro              # 主 URDF 模型
│   └── ros2_control/
│       └── head.ros2_control.xacro   # ros2_control 硬件定义
├── config/
│   └── joint_limits.yaml             # 关节限位参数
└── meshes/
    └── visual/                       # STL 可视化网格文件
```

## 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_description
source install/setup.bash
```

## 使用方式

本包通常作为 `openarmx_head_bringup` 的依赖使用。手动生成 URDF：

```bash
xacro $(ros2 pkg prefix openarmx_head_description)/share/openarmx_head_description/urdf/head.urdf.xacro
```

## 依赖

- `robot_state_publisher`
- `xacro`

## 注意事项

- 网格文件为 STL 格式，原始单位为毫米。xacro 中应用了 0.001 的缩放系数以转换为米。
- 网格坐标系通过绕 X 轴旋转 +pi/2，从 CAD 坐标系（Y 轴朝上）转换为 URDF 坐标系（Z 轴朝上）。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

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
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
