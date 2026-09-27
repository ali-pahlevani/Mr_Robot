# lift_slide_description

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


升降滑台机构的 URDF/Xacro 描述包。提供机器人模型、网格文件和独立的可视化启动文件。

## 功能

- 参数化 Xacro 模型，关节限位可配置
- 可选的 ros2_control 硬件接口宏
- 所有连杆的 STL 网格文件
- 独立可视化启动，无需真实硬件

## 包内容

| 路径 | 说明 |
|------|------|
| `urdf/lift_slide_module.urdf.xacro` | 主机器人模型 |
| `urdf/lift_slide_ros2_control.urdf.xacro` | ros2_control 硬件配置宏 |
| `meshes/` | STL 网格文件（lift_link、plate_link、chest_link、left/right_link0_base） |
| `rviz/display.rviz` | RViz 可视化配置 |
| `launch/display.launch.py` | 独立可视化启动文件 |

## 连杆与关节

| 连杆 | 说明 |
|------|------|
| `lift_base_link` | 固定底座立柱 |
| `lift_carriage_link` | 运动滑台/托板 |
| `chest_link` | 胸腔外壳（固定于滑台） |
| `left_link0_base` | 左臂安装点（固定于滑台） |
| `right_link0_base` | 右臂安装点（固定于滑台） |

| 关节 | 类型 | 轴向 | 限位 |
|------|------|------|------|
| `lift_joint` | 移动副 | Z | 软件范围 -0.750 至 0.400 m |
| `chest_joint` | 固定 | - | - |
| `left_joint0_base` | 固定 | - | - |
| `right_joint0_base` | 固定 | - | - |

## 启动（仅可视化）

```bash
ros2 launch lift_slide_description display.launch.py
```

### 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_joint_state_gui` | `true` | 是否启动 joint_state_publisher_gui |
| `start_rviz` | `true` | 是否启动 RViz |
| `min_height` | `-0.750` | 软件最小位置（米） |
| `max_height` | `0.400` | 软件最大位置（米） |

## 发布的 TF 坐标系

`lift_base_link` -> `lift_carriage_link` -> `chest_link`、`left_link0_base`、`right_link0_base`

## 编译

```bash
colcon build --packages-select lift_slide_description
source install/setup.bash
```

## 前置依赖

- ROS 2（Humble/Iron）
- xacro
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
