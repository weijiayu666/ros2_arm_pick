# 机械臂果实采集系统（抓球系统）技术文档

> 系统面向球形果实（如番茄、柑橘等）的识别与抓取。由于目标果实近似球形（直径约 4 cm），
> 视觉与抓取环节统一简化为**“抓球”**问题，本文档以下简称“抓球系统”。

---

## 1. 系统概述

抓球系统由 **差速轮底盘 + 5-DOF 机械臂 + 单目相机** 组成，运行于树莓派（aarch64），
基于 **ROS2 Jazzy**。系统能够：

1. 通过单目相机识别环境中的球形果实；
2. 估计果实相对于机械臂基座（`base_link`）的三维坐标；
3. 通过逆运动学求解机械臂抓取位姿，控制夹爪抓取果实；
4. 当目标超出机械臂工作空间时，自动驱动底盘靠近目标，实现“底盘—机械臂”协同抓取。

核心设计思路：**先视觉像素居中，再 3D 解算，再 IK，再抓取**；IK 失败时由底盘协作逼近后重试。

---

## 2. 硬件组成

| 部件 | 型号/说明 | 连接方式 |
|---|---|---|
| 主控 | 树莓派（aarch64，ROS2 Jazzy） | — |
| 机械臂 | jibot 5-DOF（5 个舵机 + 夹爪） | CH340 串口，by-path `usb-0:1.1`（`ttyUSB0`） |
| 底盘 | 差速轮（2 电机，BDC 驱动器） | CH340 串口，by-path `usb-0:1.3`（`ttyUSB1`） |
| 相机 | USB 单目相机（640×480，安装于机械臂 `link4`） | USB（`ttyACM0` 视频接口） |

> **串口注意**：机械臂与底盘均为 CH340 芯片，二者 `by-id` 相同、无法区分，必须使用 `by-path` 区分。
> 当前接线：机械臂 `usb-0:1.1`，底盘 `usb-0:1.3`。

---

## 3. 软件架构

### 3.1 包结构

| 包 | 类型 | 职责 |
|---|---|---|
| `arm_msg` | 消息包 | `ArmJointAngles.msg`（6 轴角度 + 运动时间） |
| `chassis_msg` | 消息包 | `ChassisCmd.msg`（速度指令）、`ChassisState.msg`（底盘状态） |
| `target_msg` | 消息/服务包 | `Goal.msg`（目标位姿）、`GetGoal.srv`（获取抓取目标） |
| `arm_interfaces` | 服务包 | `GetJointAngles.srv`（读关节角）、`SetJointAngle.srv`（设单关节角） |
| `arm_pkg` | 功能包 | 机械臂驱动、抓取状态机、IK 求解 |
| `vision_pkg` | 功能包 | 相机驱动、球检测定位、HSV 标定 |
| `chassis_driver` | 功能包 | 底盘驱动、底盘-机械臂桥接 |
| `my_robot_description` | 模型包 | URDF 模型（`base_link` + `link1`~`link5`） |

### 3.2 运行时节点

一键启动共拉起 6 个节点：

| 节点 | 包 | 职责 |
|---|---|---|
| `opencv_camera_node` | vision_pkg | 相机驱动，发布 `/image_raw` |
| `arm_joint_node` | arm_pkg | 机械臂串口驱动，收发关节角与夹爪指令 |
| `chassis_driver_node` | chassis_driver | 底盘串口驱动，解析速度指令并回报状态 |
| `ball_detector_node` | vision_pkg | 球检测 + 3D 定位 + 手眼变换 |
| `arm_catch_node` | arm_pkg | 抓取主状态机（扫描/居中/求解/抓取） |
| `chassis_arm_bridge` | chassis_driver | 底盘-机械臂协作桥接（二分逼近靠近目标） |

### 3.3 话题与服务

| 话题/服务 | 类型 | 发布者 → 订阅者 | 说明 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | camera → ball_detector | 原始图像（bgr8） |
| `/ball_pixel` | `Float32MultiArray` | ball_detector → arm_catch | 球像素 `(cx, cy, t)` |
| `/ball_pos` | `Float32MultiArray` | ball_detector → bridge | 球在 `base_link` 下坐标 |
| `/balls_pixel` | `Float32MultiArray` | ball_detector | 所有球的像素框（显示用） |
| `/joint_msg` | `ArmJointAngles` | arm_joint → 各节点 | 机械臂当前真实角度 |
| `/cmd_angles` | `ArmJointAngles` | arm_catch → arm_joint | 机械臂目标角度（唯一控制者） |
| `/gripper_cmd` | `String` | arm_catch → arm_joint | 夹爪指令（`catch;` / `release;`） |
| `/chassis_cmd` | `ChassisCmd` | bridge → chassis_driver | 底盘速度指令 |
| `/chassis_state` | `ChassisState` | chassis_driver → bridge | 底盘电机/电压/温度状态 |
| `/chassis_busy` | `Bool` | bridge → arm_catch | 底盘是否正在调整 |
| `/arm_status` | `String` | arm_catch → bridge | 机械臂状态（协作信号） |
| `/ik_fail_target` | `Float32MultiArray` | arm_catch → bridge | IK 失败时的目标坐标（触发底盘调整） |
| `/back_off` | `Bool` | arm_catch → bridge | 球太近请求底盘后退 |
| `/get_goal` | `GetGoal` 服务 | arm_catch → ball_detector | 居中完成后请求完整 3D 解算 |
| `/get_joint_angles` | `GetJointAngles` 服务 | — → arm_joint | 读取当前关节角 |
| `/set_joint_angle` | `SetJointAngle` 服务 | — → arm_joint | 设置单关节角 |

---

## 4. 系统工作流程

### 4.1 整体流程

```
相机图像 ──> 球检测(像素) ──> 像素居中 ──> 3D解算(/get_goal)
                                              │
                                              v
                                     IK求解 ──失败──> 底盘协作逼近 ──> 重试
                                              │成功
                                              v
                                          夹爪抓取 ──> 归位 ──> 循环
```

### 4.2 抓取主状态机（`arm_catch_node`）

| 状态 | 说明 |
|---|---|
| `RESETTING` | 启动/抓取完成后归位（`[0,0,-3,-91,0]°`），同步软硬件角度 |
| `IDLE` | 等待球出现；连续 3 s 无球则转入扫描 |
| `SWEEPING` | 0 轴在 `-90°~90°` 来回扫描找球（其余轴保持固定姿态） |
| `CENTERING` | 依据球像素误差实时调整 0/2 轴，将球移到画面中心 |
| `CENTERING_CONFIRM` | 居中后延时 1 s 取新帧二次确认，防止误判 |
| `TRACKING` | 底盘调整期间保持球在画面内的小幅跟踪 |
| `SOLVING` | 调用 `/get_goal` 做完整 3D 解算，并运行 IK |
| `GRASPING` | 运动到求解位姿，闭合夹爪 |
| `GRASPING_WAIT` | 夹紧等待（1 s） |
| `PLACING` | 运动到放置位姿（`[0,20,-30,60,0]°`） |
| `OPENING` | 释放夹爪 |
| `MANUAL_MOVE` | 手动目标模式（仅移动，不抓取） |

### 4.3 底盘协作流程（`chassis_arm_bridge`）

当 IK 失败时，`arm_catch_node` 发布 `/ik_fail_target`，`chassis_arm_bridge` 进入离散二分逼近：

```
ALIGN_X（左右对准 x→0）──> MOVE_Y（朝目标 y 走当前偏差一半）
        ^                           │
        │                           v
      SETTLE（停下稳定，重估 y）<── BACK_OFF（球太近则后退）
```

- 轴映射：`y` = 前进/后退，`x` = 左转/右转；
- 转向方向：`x` 偏大 → 右转（`angular_z` 负），`x` 偏小 → 左转（`angular_z` 正）；
- 到位后交还机械臂重试抓取。

---

## 5. 核心算法

### 5.1 视觉球检测（`ball_detector_node`）

1. **HSV 阈值分割**：阈值从 `hsv_params.yaml` 加载，分离目标颜色；
2. **形态学去噪/补缺**：腐蚀/膨胀去除孤立噪点，闭运算补全球面反光造成的缺口；
3. **球心定位**：`cv2.distanceTransform` 距离变换，取全局最大峰值（离背景最远的点）= 球心；
   重叠球时全局最大峰值即**前方球**球心；
4. **半径估算**：找包含球心的轮廓，取 `boundingRect` 外接矩形的内切圆 `min(w,h)/2` 作为球半径；
5. **迟滞锁定**：两球深度相近时，仅在候选球明显更大（>15%）才切换目标，抑制识别框跳变。

### 5.2 深度估算（小孔成像）

单目深度由“已知球直径 + 像素直径”反推：

```
depth = BALL_DIAMETER × f_eq / pixel_diameter
```

其中 `BALL_DIAMETER = 0.040 m`，`f_eq = (fx + fy) / 2`，`pixel_diameter` 为球在图像中的像素直径。

### 5.3 手眼标定与坐标变换

相机安装在机械臂 `link4` 上（眼在手）。坐标变换链：

```
像素坐标 ──内参──> 相机坐标 ──手眼矩阵──> link4 坐标 ──正运动学──> base_link 坐标
```

- 相机内参 + 畸变系数：`camera_intrinsic.xml`；
- 手眼标定矩阵：`hand_eye_result.xml`；
- `link4` 位姿由 `forward_kinematics_link4()` 用当前关节角实时计算。

### 5.4 逆运动学求解（`ik_solver.py`）

- **结构**：5-DOF（q1~q5），末端定义为“两指中心 + 70 mm 沿末端 X 轴前伸 − 10 mm 沿末端 Z 轴”；
- **算法**：解析法（纯几何法，`solve_ik_analytic`）——固定腕部滚转 q5，用 `atan2(y, x)` 几何解出 q1（臂平面转向目标），再对 q1~q4 做 Jacobian 阻尼最小二乘迭代收敛（≤200 步）；
- **特点**：相比 RRT-Connect 快千倍以上，直接返回 `[q_start, q_solution]` 路径。

### 5.5 工作空间约束

半球形约束（单位：米，`base_link` 坐标系）：

| 参数 | 值 | 含义 |
|---|---|---|
| `r_max` | 0.30 | 水平径向距离上限 |
| `z_center` | 0.056 | joint1 高度（半球球心） |
| `reach` | 0.34 | 臂长半径 |
| `z_min` | -0.11 | 高度下限 |
| `z_max` | 0.36 | 高度上限 |

### 5.6 抓取深度分段补偿

机械臂末端定位存在系统性误差，按识别出的 `base_link` 的 `z` 坐标做分段补偿（仅加在 IK 目标上，不参与工作范围判断）：

| z 范围 | 补偿 |
|---|---|
| `z > 30 cm` | +0.7 cm |
| `20 cm < z ≤ 30 cm` | +0.7 cm |
| `15 cm ≤ z ≤ 20 cm` | +1.5 cm |
| `z < 15 cm` | +2.0 cm |

---

## 6. 关键参数

### 6.1 视觉

| 参数 | 值 | 说明 |
|---|---|---|
| 相机分辨率 | 640×480 | 需与内参标定一致 |
| 球直径 | 0.040 m | 用于深度估算 |
| HSV 阈值 | `hsv_params.yaml` | 含腐蚀/膨胀迭代、最小面积 |

### 6.2 居中伺服

| 参数 | 值 |
|---|---|
| 居中像素阈值 | ±50 px |
| 居中增益 | 0.08 °/px |
| 单步限幅 | 25° |
| 居中周期 | 100 ms |

### 6.3 底盘（差速）

| 参数 | 值 |
|---|---|
| `linear_gain` | 14.0 |
| `angular_gain` | 3.0 |
| `max_duty` | 1.0 |
| `track_width` | 0.27 m |
| `invert` / `swap_motors` | 默认 `true` |

---

## 7. 启动方式

### 7.1 一键启动

```bash
source /home/pi4/dev_ws/install/setup.bash
ros2 launch arm_pkg bringup.launch.py
```

### 7.2 单独调试（示例）

```bash
# 仅机械臂 + 视觉，不启动底盘
ros2 run vision_pkg opencv_camera_node
ros2 run arm_pkg arm_joint_node --ros-args -p serial_port:=/dev/serial/by-path/...-usb-0:1.1:1.0-port0
ros2 run vision_pkg ball_detector_node
ros2 run arm_pkg arm_catch_node
```

---

## 8. 标定流程

| 标定项 | 工具/文件 | 说明 |
|---|---|---|
| HSV 阈值 | `ros2 run vision_pkg hsv_calibrator` | 拖动滑杆调参，`s` 保存到 `hsv_params.yaml`，`q` 退出 |
| 相机内参 | `camera_intrinsic.xml` | 需与分辨率 640×480 匹配 |
| 手眼标定 | `hand_eye_result.xml` | 相机（link4）到末端的手眼矩阵 |

---

## 9. 坐标系定义

以机械臂基座 `base_link` 为参考系：

- **x**：右（横向）
- **y**：前（前进方向）
- **z**：上（竖直）

底盘运动指令 `ChassisCmd` 中：
- `linear_x` 正值 = 前进，负值 = 后退；
- `angular_z` 正值 = 左转（逆时针），负值 = 右转（顺时针）。
