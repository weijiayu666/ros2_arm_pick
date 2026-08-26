"""
自动抓取节点 — 与 ball_locator.py 流程一致:

  检测到球 → 像素居中 → /get_goal → IK → /cmd_angles → 舵机

全程自动, 无需手动触发。
"""
import rclpy
from rclpy.node import Node
from target_msg.srv import GetGoal
from arm_msg.msg import ArmJointAngles
from std_msgs.msg import Float32MultiArray, String, Bool
from arm_pkg.ik_solver import solve_ik_analytic
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import math
import time


# 工作空间 (米) - 半球形约束 (抓高处物体)
# 机械臂实测: 竖直最高 41cm, 水平前伸最远 34cm, joint1 高 5.6cm
WORKSPACE = {
    'r_max': 0.30,      # 水平径向距离上限 (前伸 34cm 留 4cm 余量)
    'z_center': 0.056,  # joint1 高度 (半球球心, base_link 上方 5.6cm)
    'reach': 0.34,      # 臂长半径 (水平最远 34cm)
    'z_min': -0.11,     # 高度下限 (机械臂最低可达约 -11cm)
    'z_max': 0.36,      # 抓高处物体: 高度上限 (竖直最高 41cm 留 5cm 余量)
}

# 关节限位 (度)
ANGLE_LIMITS_DEG = [180.0, 135.0, 135.0, 135.0, 135.0]

# 居中像素阈值 (px)
CENTER_THRESHOLD = 50

# 居中调整间隔 (秒) - 连续追踪, 类似 OpenMV 跟随示例的 50ms 间隔
CENTER_INTERVAL = 0.1

# 居中步进增益 (像素误差 → 角度步长, °/px)
CENTER_GAIN = 0.08

# 居中步进上限 (防止单次过冲)
CENTER_STEP_MAX = 25.0

# 居中步进下限 (防止过小无效移动)
CENTER_STEP_MIN = 4.0

# 两次抓取间冷却 (秒)
CATCH_COOLDOWN = 2.0

# 放置位姿: 关节0-4 (度)
PLACE_ANGLES = [0.0, 20.0, -30.0, 60.0, 0.0]

# 归位位姿: 关节0-4 (度)
RESET_ANGLES = [0.0, 0.0, -3.0, -91.0, 0.0]

# 夹爪动作等待时间 (秒)
GRIPPER_CATCH_WAIT = 1.0   # 夹爪夹紧
GRIPPER_OPEN_WAIT  = 1.0   # 夹爪释放

# 居中确认: 首次判定居中后等待此时间, 再取新帧二次确认
CENTER_CONFIRM_WAIT = 1.0

# 居中确认期间球丢失超时: 超过此时间仍无球则回 IDLE (进而触发扫描)
CONFIRM_LOST_TIMEOUT = 2.0

# 居中追踪期间球丢失超时: 超过此时间仍无球则回 IDLE (进而触发扫描)
CENTERING_LOST_TIMEOUT = 2.0

# 运动时间 (毫秒) - 发送给下位机, 固件保证在此时间内完成运动
MOTION_CENTER = 500    # 居中调整: 快速
MOTION_SOLVE  = 1500   # IK求解后移动: 中速
MOTION_PLACE  = 2000   # 归位: 慢速平滑
MOTION_RESET  = 2000   # 归位: 慢速平滑

# 扫描参数 (0轴粗调找球)
SWEEP_STEP_DEG = 8.0    # 扫描步进 (°)
SWEEP_INTERVAL = 0.15   # 扫描步进间隔 (s)
SWEEP_MIN = -90.0
SWEEP_MAX = 90.0

# 底盘调整期间跟踪 (方案A): 0轴+2轴小幅度微调, 保持球在画面中间
TRACK_GAIN = 0.04       # 跟踪增益 (°/px)

# 球太近判定: 像素面积超过此值视为面积过大(太近), 请求底盘后退 (25000px^2 ≈ 直径178px ≈ 球距离约10cm)
TOO_CLOSE_AREA = 25000.0

# 后退请求冷却 (秒): 防止底盘桥接未就绪时每 0.5s 反复刷请求
BACK_OFF_COOLDOWN = 2.0

# 抓取目标 z 轴分段补偿 (米): 按识别出的 base z 坐标分段, 只加在 IK 目标上, 不影响工作范围判断
Z_COMP_FAR  = 0.007   # z > 30cm 时补偿 0.7cm
Z_COMP_DEEP = 0.007   # 20cm < z <= 30cm 时补偿 0.7cm
Z_COMP_MID  = 0.015   # 15cm <= z <= 20cm 时补偿 1.5cm
Z_COMP_NEAR = 0.020   # z < 15cm 时补偿 2cm


class ArmCatchNode(Node):
    def __init__(self):
        super().__init__('arm_catch_node')

        # 当前关节角 (弧度)
        self.current_angles = [0.0] * 6

        # 状态
        self._ball_data = None        # (cx, cy, arrival_time) 原子更新, 避免读写竞态
        self._ball_area_px = 0.0      # 最新球的像素面积 (用于判定面积过大/太近)
        self._last_ball_seen_time = 0.0  # 最后一次检测到球的时间 (用于球丢失超时)
        self._pixel_received = False  # 球检测节点是否已就绪 (收到过 /ball_pixel, 含无球哨兵)
        self._joint_received = False  # 机械臂节点是否已就绪 (收到过 /joint_msg)
        self._center_attempts = 0
        self._state = 'RESETTING'    # 启动先归位, 确保 q4=-91° 与软件角度同步 (修复手眼变换 q4=0° 坐标错)
        self._last_adjust = 0.0
        self._last_catch = 0.0       # 上次抓取完成时间
        self._motor_deadline = 0.0   # 电机运动完成时间, 此前的帧都是运动中途的
        self._state_start_time = time.time()  # 进入当前状态的时间 (启动即进入归位, 立即计时)
        self._cmd_sent = False       # 当前阶段指令是否已发送
        self._ik_busy = False        # IK 计算中, 防止状态机重入
        self._manual_move = False    # 手动目标模式: 只移动到目标位, 不抓取
        self._last_back_off = 0.0    # 上次请求后退时间 (防抖)

        # 底盘协作
        self._chassis_busy = False
        self._ik_failed = False      # 上次 IK 是否失败, 用于触发扫描
        self._last_target = None       # 上次 IK 失败的目标坐标 (base_link)
        self.status_pub = self.create_publisher(String, '/arm_status', 10)
        self.fail_target_pub = self.create_publisher(
            Float32MultiArray, '/ik_fail_target', 10)
        self.back_off_pub = self.create_publisher(Bool, '/back_off', 10)
        self.chassis_busy_sub = self.create_subscription(
            Bool, '/chassis_busy', self._chassis_busy_cb, 10)

        # /get_goal 服务客户端 (复用, 避免每次求解都重建)
        self.goal_client = self.create_client(GetGoal, '/get_goal')

        # 扫描状态
        self._sweep_angle = 0.0
        self._sweep_dir = 1
        self._sweep_last_step = 0.0
        self._sweep_base_angles = [0.0] * 5  # 扫锚时保持其他关节的基准角度
        self._idle_no_ball_time = 0.0  # IDLE 中无球计时

        # IK 线程池: 并行跑两条策略
        self._ik_executor = ThreadPoolExecutor(max_workers=2)

        # 订阅像素坐标 (供居中伺服)
        self.pixel_sub = self.create_subscription(
            Float32MultiArray, '/ball_pixel', self.pixel_callback, 10)

        # 订阅手动目标 (厘米, base_link 坐标): 直接移动到目标位, 不抓取
        self.manual_target_sub = self.create_subscription(
            Float32MultiArray, '/manual_target', self._manual_target_cb, 10)

        # 订阅关节角
        self.joint_sub = self.create_subscription(
            ArmJointAngles, '/joint_msg', self.joint_callback, 10)

        # 发布命令
        self.cmd_pub = self.create_publisher(ArmJointAngles, '/cmd_angles', 10)

        # 发布夹爪命令 (catch / release)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)

        # 定时器: 驱动居中 → IK 状态机
        self.timer = self.create_timer(0.5, self.state_machine_callback)

        self.get_logger().info('Arm catch node started (auto mode). Waiting for ball...')

    def joint_callback(self, msg: ArmJointAngles):
        self._joint_received = True
        self.current_angles = list(msg.angles[:6])

    def pixel_callback(self, msg: Float32MultiArray):
        self._pixel_received = True
        if len(msg.data) >= 2:
            cx = msg.data[0]
            cy = msg.data[1]
            if cx < 0 or cy < 0:
                self._ball_data = None  # 无球哨兵: 检测节点就绪但当前无球
                self._ball_area_px = 0.0
            else:
                self._ball_data = (cx, cy, time.time())  # 原子写入
                self._last_ball_seen_time = time.time()
                diam = msg.data[2] if len(msg.data) >= 3 else 0.0
                self._ball_area_px = math.pi * (diam ** 2) / 4.0

    def _chassis_busy_cb(self, msg: Bool):
        """底盘忙状态回调"""
        self._chassis_busy = msg.data

    def _publish_status(self):
        """发布当前状态到 /arm_status"""
        msg = String()
        msg.data = self._state
        self.status_pub.publish(msg)

    def _in_workspace(self, x, y, z):
        """半球形工作空间检查: 水平径向约束 + z 随 r 收缩"""
        r = math.sqrt(x * x + y * y)
        if r > WORKSPACE['r_max']:
            return False
        reach = math.sqrt(max(0.0, WORKSPACE['reach'] ** 2 - r * r))
        z_geom_min = WORKSPACE['z_center'] - reach
        z_geom_max = WORKSPACE['z_center'] + reach
        z_lo = max(WORKSPACE['z_min'], z_geom_min)
        z_hi = min(WORKSPACE['z_max'], z_geom_max)
        return z_lo <= z <= z_hi

    def _apply_z_comp(self, z_base: float) -> float:
        """抓取目标 z 分段补偿 (只用于 IK 目标, 不用于工作范围判断)"""
        if z_base > 0.30:
            return z_base + Z_COMP_FAR
        if z_base > 0.20:
            return z_base + Z_COMP_DEEP
        if z_base >= 0.15:
            return z_base + Z_COMP_MID
        return z_base + Z_COMP_NEAR

    def _fail(self, reason: str):
        """统一的失败处理: 回到 IDLE + 通知底盘桥接"""
        self._state = 'IDLE'
        self._last_catch = time.time()
        self._ik_failed = True   # 标记 IK 失败, 触发扫描
        self._publish_status()
        # 发布失败目标坐标, 供底盘桥接使用
        if self._last_target is not None:
            msg = Float32MultiArray()
            msg.data = [float(self._last_target[0]), float(self._last_target[1]), float(self._last_target[2])]
            self.fail_target_pub.publish(msg)
        self.get_logger().warn(f'机械臂失败: {reason}')

    # ── 状态机 ──

    def state_machine_callback(self):
        """定时器驱动状态机"""
        if self._state == 'IDLE':
            self._check_auto_start()
        elif self._state == 'CENTERING':
            self._do_centering()
        elif self._state == 'CENTERING_CONFIRM':
            self._do_centering_confirm()
        elif self._state == 'SOLVING':
            self._do_solving()
        elif self._state == 'GRASPING':
            self._do_grasping()
        elif self._state == 'GRASPING_WAIT':
            self._do_grasping_wait()
        elif self._state == 'PLACING':
            self._do_placing()
        elif self._state == 'OPENING':
            self._do_opening()
        elif self._state == 'RESETTING':
            self._do_resetting()
        elif self._state == 'SWEEPING':
            self._do_sweeping()
        elif self._state == 'TRACKING':
            self._do_tracking()
        elif self._state == 'MANUAL_MOVE':
            self._do_manual_move()

        # 每次状态机触发时发布当前状态, 供底盘桥接监控
        self._publish_status()

    def _check_auto_start(self):
        """IDLE 时检测到球 → 开始抓取; 上次 IK 失败且球丢失 → 扫描找球"""
        # 底盘正在移动 → 双轴跟踪保持球居中 (方案A)
        if self._chassis_busy:
            # 关节角未就绪时 current_angles 仍是 [0]*6, 不能跟踪, 等待 arm_joint_node 反馈
            if not self._joint_received:
                self.get_logger().warn(
                    '底盘移动中, 但机械臂关节角未就绪(未收到 /joint_msg), 暂不跟踪',
                    throttle_duration_sec=2.0)
                return
            self._state = 'TRACKING'
            self._motor_deadline = 0.0  # 允许立即使用最新帧
            return

        # IK 计算中, 不重复触发
        if self._ik_busy:
            return

        # 冷却期
        if time.time() - self._last_catch < CATCH_COOLDOWN:
            return

        # 检测到球 → 先判断是否太近(面积过大), 太近则请求底盘后退
        if self._ball_data is not None:
            # 关节角未就绪时 current_angles 仍是 [0]*6, 不能居中, 等待 arm_joint_node 反馈
            if not self._joint_received:
                self.get_logger().warn(
                    '检测到球, 但机械臂关节角未就绪(未收到 /joint_msg), 暂不居中',
                    throttle_duration_sec=2.0)
                return
            if self._ball_area_px > TOO_CLOSE_AREA:
                self._request_back_off()
                return
            self._ik_failed = False  # 清除失败标志
            self._idle_no_ball_time = 0.0
            self._state = 'CENTERING'
            self._center_attempts = 0
            self._center_ang0 = math.degrees(self.current_angles[0])
            self._center_ang2 = math.degrees(self.current_angles[2])
            self.get_logger().info('=== 检测到球, 自动开始抓取 ===')
            return

        # 上次 IK 失败
        if self._ik_failed:
            if self._chassis_busy:
                return  # 底盘正在调整, 等待
            # 底盘已完成, 球仍不可见 → 0轴扫描找球
            self._enter_sweeping('IK 失败, 底盘调整完成')
            return

        # 无球超时 → 自动扫描 (等机械臂 + 球检测节点都就绪后才开始计时)
        if not (self._pixel_received and self._joint_received):
            return
        if self._idle_no_ball_time == 0.0:
            self._idle_no_ball_time = time.time()
        elif time.time() - self._idle_no_ball_time > 3.0:
            self._enter_sweeping('无球超时')

    def _request_back_off(self):
        """球太近(面积过大): 请求底盘后退, 拉开距离"""
        now = time.time()
        if now - self._last_back_off < BACK_OFF_COOLDOWN:
            return  # 防抖: 等待上一轮后退执行完
        self._last_back_off = now
        self.get_logger().info(
            f'=== 球面积过大 ({self._ball_area_px:.0f}px^2 > {TOO_CLOSE_AREA:.0f}px^2), '
            f'请求底盘后退 5cm ==='
        )
        msg = Bool()
        msg.data = True
        self.back_off_pub.publish(msg)

    def _enter_sweeping(self, reason: str):
        """进入扫描状态, 只动 0 轴, 其他关节固定为居中姿态"""
        self.get_logger().info(f'=== {reason}, 启动 0轴扫描找球 ===')
        self._state = 'SWEEPING'
        self._sweep_angle = SWEEP_MIN
        self._sweep_dir = 1
        self._sweep_last_step = 0.0
        self._idle_no_ball_time = 0.0
        self._sweep_base_angles = [0.0, 0.0, -3.0, -91.0, 0.0]

    def _do_centering(self):
        """居中阶段: 连续追踪, 等电机完成+新帧到达后才做下一次调整"""
        data = self._ball_data  # 原子读取, 坐标和时间一定匹配
        if data is None:
            # 球丢失: 超时后回 IDLE 重新判断 (IDLE 无球 3s 会自动扫描)
            if self._last_ball_seen_time > 0.0 and \
                    time.time() - self._last_ball_seen_time > CENTERING_LOST_TIMEOUT:
                self.get_logger().warn('居中阶段球丢失超时, 回到 IDLE')
                self._state = 'IDLE'
                self._motor_deadline = 0.0
            return

        cx, cy, px_time = data

        # 关键: 只用电机运动完成后到达的新帧, 避免运动中途的帧导致过冲
        if px_time < self._motor_deadline:
            return  # 帧是电机运动中途拍的, 等电机停稳后的新帧

        dx = cx - 313.9   # 光心 CX
        dy = cy - 232.6   # 光心 CY

        # 已居中 → 进入确认阶段, 等电机停稳后二次确认
        if abs(dx) <= CENTER_THRESHOLD and abs(dy) <= CENTER_THRESHOLD:
            self.get_logger().info(f'初步居中 (dx={dx:.0f} dy={dy:.0f}px) → 等待 {CENTER_CONFIRM_WAIT}s 二次确认')
            self._state = 'CENTERING_CONFIRM'
            self._state_start_time = time.time()
            return

        # 连续追踪: 从上次指令角度出发, 根据像素偏差实时微调
        if dx < -CENTER_THRESHOLD:
            self._center_ang0 += abs(dx) * CENTER_GAIN * 0.5
        elif dx > CENTER_THRESHOLD:
            self._center_ang0 -= abs(dx) * CENTER_GAIN * 0.5
        if dy < -CENTER_THRESHOLD:
            self._center_ang2 -= abs(dy) * CENTER_GAIN * 0.5
        elif dy > CENTER_THRESHOLD:
            self._center_ang2 += abs(dy) * CENTER_GAIN * 0.5

        # 限位
        self._center_ang0 = max(-120.0, min(120.0, self._center_ang0))
        self._center_ang2 = max(-135.0, min(135.0, self._center_ang2))

        ang = [math.degrees(a) for a in self.current_angles[:5]]
        ang[0] = self._center_ang0
        ang[2] = self._center_ang2

        self._center_attempts += 1
        self.get_logger().info(
            f'追踪 [{self._center_attempts}]: '
            f'dx={dx:.0f} dy={dy:.0f}px → 电机0={ang[0]:+.1f}° 电机2={ang[2]:+.1f}°'
        )

        cmd = ArmJointAngles()
        cmd.angles = [math.radians(a) for a in ang] + [self.current_angles[5]]
        cmd.motion_time_ms = int(CENTER_INTERVAL * 1000)  # 运动时间 = 更新间隔
        self.cmd_pub.publish(cmd)

        # 设置截止时间: 电机运动(100ms) + 摄像头采集+处理余量(~80ms)
        self._motor_deadline = time.time() + CENTER_INTERVAL + 0.08

    def _do_centering_confirm(self):
        """居中确认: 等待电机停稳后取新帧二次确认, 防止过冲误判"""
        elapsed = time.time() - self._state_start_time
        if elapsed < CENTER_CONFIRM_WAIT:
            return  # 等待电机停稳

        data = self._ball_data
        if data is None:
            # 确认期间球丢失: 超时后回 IDLE 重新判断 (IDLE 无球 3s 会自动扫描)
            if elapsed > CENTER_CONFIRM_WAIT + CONFIRM_LOST_TIMEOUT:
                self.get_logger().warn('居中确认期间球丢失, 回到 IDLE')
                self._state = 'IDLE'
                self._motor_deadline = 0.0
            return

        cx, cy, px_time = data
        # 必须是在确认等待期间到达的新帧
        if px_time < self._state_start_time + CENTER_CONFIRM_WAIT * 0.5:
            return  # 帧太旧, 等更新的

        dx = cx - 313.9
        dy = cy - 232.6

        if abs(dx) <= CENTER_THRESHOLD and abs(dy) <= CENTER_THRESHOLD:
            self.get_logger().info(f'✅ 二次确认居中 (dx={dx:.0f} dy={dy:.0f}px) → 请求目标')
            self._state = 'SOLVING'
        else:
            self.get_logger().info(f'❌ 居中未通过 (dx={dx:.0f} dy={dy:.0f}px) → 继续调整')
            self._state = 'CENTERING'
            self._motor_deadline = 0.0  # 重置, 允许立即调整

    # ── 底盘调整期间跟踪: 0轴+2轴小幅度保持球居中 (方案A) ──

    def _do_tracking(self):
        """底盘移动期间双轴跟踪, 防止球完全移出画面"""
        # 底盘就位 → 退出跟踪, 回到 IDLE 重新判断 (太近则后退, 否则重新居中抓取)
        if not self._chassis_busy:
            self.get_logger().info('底盘就位 → 回到 IDLE 重新判断')
            self._state = 'IDLE'
            self._motor_deadline = 0.0
            return

        data = self._ball_data
        if data is None:
            return  # 球丢失, 保持当前姿态, 等底盘停

        cx, cy, px_time = data
        if px_time < self._motor_deadline:
            return  # 电机运动中途的帧, 忽略

        dx = cx - 313.9   # 光心 CX
        dy = cy - 232.6   # 光心 CY

        # 已居中 → 无需调整
        if abs(dx) <= CENTER_THRESHOLD and abs(dy) <= CENTER_THRESHOLD:
            return

        # 小幅度微调 (0轴控制水平, 2轴控制俯仰)
        step0 = 0.0
        step2 = 0.0
        if dx < -CENTER_THRESHOLD:
            step0 = abs(dx) * TRACK_GAIN
        elif dx > CENTER_THRESHOLD:
            step0 = -abs(dx) * TRACK_GAIN
        if dy < -CENTER_THRESHOLD:
            step2 = -abs(dy) * TRACK_GAIN
        elif dy > CENTER_THRESHOLD:
            step2 = abs(dy) * TRACK_GAIN

        ang = [math.degrees(a) for a in self.current_angles[:5]]
        ang[0] = max(-120.0, min(120.0, ang[0] + step0))
        ang[2] = max(-135.0, min(135.0, ang[2] + step2))

        cmd = ArmJointAngles()
        cmd.angles = [math.radians(a) for a in ang] + [self.current_angles[5]]
        cmd.motion_time_ms = int(CENTER_INTERVAL * 1000)
        self.cmd_pub.publish(cmd)

        self._motor_deadline = time.time() + CENTER_INTERVAL + 0.08
        self.get_logger().info(
            f'跟踪: dx={dx:.0f} dy={dy:.0f}px → 0轴{step0:+.1f}° 2轴{step2:+.1f}°'
        )

    # ── 扫描: 0轴 -90°→90° 找球 ──

    def _do_sweeping(self):
        """0轴快速扫描, 直到识别出小球"""
        now = time.time()

        # 检测到球 → 停止扫描, 进入居中
        if self._ball_data is not None and time.time() - self._ball_data[2] < 0.3:
            self.get_logger().info(f'扫描中检测到球! 当前角度 {self._sweep_angle:.0f}°')
            self._ik_failed = False
            self._state = 'CENTERING'
            self._center_attempts = 0
            self._center_ang0 = math.degrees(self.current_angles[0])
            self._center_ang2 = math.degrees(self.current_angles[2])
            return

        # 扫描到头, 反向继续
        if self._sweep_angle >= SWEEP_MAX and self._sweep_dir == 1:
            self._sweep_dir = -1
            self.get_logger().info('扫描到头, 反向继续')
        elif self._sweep_angle <= SWEEP_MIN and self._sweep_dir == -1:
            self._sweep_dir = 1
            self.get_logger().info('扫描到头, 反向继续')

        # 步进
        if now - self._sweep_last_step < SWEEP_INTERVAL:
            return

        self._sweep_angle += SWEEP_STEP_DEG * self._sweep_dir
        if self._sweep_angle > SWEEP_MAX:
            self._sweep_angle = SWEEP_MAX
        elif self._sweep_angle < SWEEP_MIN:
            self._sweep_angle = SWEEP_MIN

        self._sweep_last_step = now

        # 发送关节指令: 只动 0 轴, 其他关节保持进入扫描时的基准角度
        ang = list(self._sweep_base_angles)
        ang[0] = self._sweep_angle
        cmd = ArmJointAngles()
        cmd.angles = [math.radians(a) for a in ang] + [self.current_angles[5]]
        cmd.motion_time_ms = int(SWEEP_INTERVAL * 1000)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f'扫描: 0轴={self._sweep_angle:.0f}°')

    def _do_solving(self):
        """求解阶段: 请求 /get_goal → IK"""
        if self._ik_busy:
            return  # IK 正在计算中, 防止重复提交
        self._ik_busy = True

        # 调用 /get_goal 服务获取完整3D目标 (复用客户端)
        client = self.goal_client
        if not client.wait_for_service(timeout_sec=3.0):
            self._ik_busy = False
            self._fail('/get_goal 服务不可用')
            return

        req = GetGoal.Request()
        future = client.call_async(req)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """收到 /get_goal 响应后提交 IK 到线程池并行计算"""
        try:
            resp = future.result()
        except Exception as e:
            self._ik_busy = False
            self._fail(f'/get_goal 调用异常: {e}')
            return

        if not resp.success:
            self._ik_busy = False
            self._fail('/get_goal: 无有效目标')
            return

        self._last_target = np.array([resp.goal.x, resp.goal.y, resp.goal.z])
        target = self._last_target.copy()

        # 检查工作范围 (半球形: 水平径向 + z 随 r 收缩), 用原始 z (不含补偿)
        if not self._in_workspace(target[0], target[1], target[2]):
            self._ik_busy = False
            r = math.sqrt(target[0] ** 2 + target[1] ** 2)
            self._fail(
                f'目标超出工作范围: [{target[0]*100:.0f}, {target[1]*100:.0f}, {target[2]*100:.0f}] cm (r={r*100:.0f}cm)'
            )
            return

        # 抓取目标 z 加分段补偿 (只影响 IK 解算目标, 不影响上面的范围判断)
        target[2] = self._apply_z_comp(target[2])

        q_start = [float(a) for a in self.current_angles[:5]]
        self.get_logger().info(
            f'目标: ({target[0]*100:.1f}, {target[1]*100:.1f}, {target[2]*100:.1f}) cm  |  '
            f'当前: [{", ".join(f"{math.degrees(a):+.1f}" for a in q_start)}] °'
        )
        self.get_logger().info('提交 IK 到线程池并行计算...')

        # 提交到线程池, 并行跑两条策略
        ik_future = self._ik_executor.submit(self._run_ik_parallel, target, q_start)
        ik_future.add_done_callback(self._ik_done_callback)

    def _run_ik_parallel(self, target, q_start):
        """IK 求解 (解析法: 固定 q4/q5, Jacobian 3×3)"""
        path, err = solve_ik_analytic(target, q_start)
        if err > 0.01:
            return None, err
        return path, err

    def _manual_target_cb(self, msg: Float32MultiArray):
        """手动目标订阅: 收到 [x, y, z] (厘米, base_link 坐标) 后直接移动到该位置, 不抓取"""
        if len(msg.data) < 3:
            self.get_logger().warn('手动目标数据不足, 需要 [x_cm, y_cm, z_cm]')
            return

        x = float(msg.data[0]) / 100.0
        y = float(msg.data[1]) / 100.0
        z = float(msg.data[2]) / 100.0
        target = np.array([x, y, z])

        if not self._in_workspace(x, y, z):
            r = math.sqrt(x * x + y * y)
            self.get_logger().warn(
                f'手动目标超出工作范围: [{x*100:.0f}, {y*100:.0f}, {z*100:.0f}] cm (r={r*100:.0f}cm)')
            return

        if self._ik_busy:
            self.get_logger().warn('IK 计算中, 忽略本次手动目标')
            return

        self._ik_busy = True
        self._manual_move = True
        self._last_target = target
        q_start = [float(a) for a in self.current_angles[:5]]
        self.get_logger().info(
            f'手动目标 ({x*100:.1f}, {y*100:.1f}, {z*100:.1f}) cm → 提交 IK 求解...')
        ik_future = self._ik_executor.submit(self._run_ik_parallel, target, q_start)
        ik_future.add_done_callback(self._ik_done_callback)

    def _ik_done_callback(self, future):
        """IK 计算完成回调 (在线程池线程中执行)"""
        self._ik_busy = False
        manual = self._manual_move
        self._manual_move = False

        try:
            path, err = future.result()
        except Exception as e:
            self._ik_busy = False
            if manual:
                self.get_logger().warn(f'手动目标 IK 线程异常: {e}')
                self._state = 'IDLE'
                return
            self._fail(f'IK 线程异常: {e}')
            return

        if path is None or len(path) < 2:
            self._ik_busy = False
            if manual:
                self.get_logger().warn('手动目标 IK 求解失败')
                self._state = 'IDLE'
                return
            self._fail('IK 求解失败')
            return

        final_q = path[-1]
        final_deg = [math.degrees(q) for q in final_q]

        for i in range(5):
            limit = ANGLE_LIMITS_DEG[i]
            if final_deg[i] < -limit or final_deg[i] > limit:
                self._ik_busy = False
                if manual:
                    self.get_logger().warn(f'手动目标 轴{i+1} 超限位: {final_deg[i]:.0f}°')
                    self._state = 'IDLE'
                    return
                self._fail(f'轴{i+1} 超限位: {final_deg[i]:.0f}°')
                return

        self.get_logger().info(
            f'IK 成功 [{", ".join(f"{d:+.1f}" for d in final_deg)}] °  |  误差={err*1000:.1f}mm'
        )

        cmd = ArmJointAngles()
        cmd.angles = [float(q) for q in final_q] + [self.current_angles[5]]
        cmd.motion_time_ms = MOTION_SOLVE
        self.cmd_pub.publish(cmd)

        if manual:
            self.get_logger().info(f'手动移动到目标位, 等待 {MOTION_SOLVE}ms 到位')
            self._state = 'MANUAL_MOVE'
        else:
            self.get_logger().info(f'已发送 {len(cmd.angles)} 轴指令 → 等待 {MOTION_SOLVE}ms 到位')
            self._state = 'GRASPING'
        self._state_start_time = time.time()

    def _do_manual_move(self):
        """手动移动到位后回到 IDLE (不抓取、不归位)"""
        if time.time() - self._state_start_time < MOTION_SOLVE / 1000.0:
            return
        self.get_logger().info('✅ 手动移动到位')
        self._state = 'IDLE'
        self._last_catch = time.time()  # 触发冷却, 避免立即被自动抓取流程打断

    # ── 放置-开爪-归位 循环 ──

    def _do_grasping(self):
        """等待固件 MOTION_SOLVE 时间完成后发送 catch 命令"""
        if time.time() - self._state_start_time < MOTION_SOLVE / 1000.0:
            return

        self.get_logger().info('✅ GRASPING 完成 → 发送 catch 命令')
        msg = String()
        msg.data = 'catch'
        self.gripper_pub.publish(msg)
        self._state = 'GRASPING_WAIT'
        self._state_start_time = time.time()

    def _do_grasping_wait(self):
        """等待夹爪夹紧"""
        if time.time() - self._state_start_time < GRIPPER_CATCH_WAIT:
            return
        self.get_logger().info('✅ GRASPING_WAIT 完成 → 移动到放置位置')
        self._state = 'PLACING'
        self._state_start_time = time.time()
        self._cmd_sent = False

    def _do_placing(self):
        """发送放置指令，等待固件 MOTION_PLACE 时间完成后打开夹爪"""
        if not self._cmd_sent:
            cmd = ArmJointAngles()
            cmd.angles = [math.radians(a) for a in PLACE_ANGLES] + [self.current_angles[5]]
            cmd.motion_time_ms = MOTION_PLACE
            self.cmd_pub.publish(cmd)
            self._cmd_sent = True
            self.get_logger().info(
                f'放置位姿: [{", ".join(f"{a:+.0f}" for a in PLACE_ANGLES)}] ° | 等待 {MOTION_PLACE}ms'
            )
            return

        if time.time() - self._state_start_time < MOTION_PLACE / 1000.0:
            return

        self.get_logger().info('✅ PLACING 完成 → 打开夹爪')
        self._state = 'OPENING'
        self._state_start_time = time.time()

    def _do_opening(self):
        """释放夹爪, 放置小球"""
        elapsed = time.time() - self._state_start_time

        if elapsed < 0.5:
            self.get_logger().info('释放小球 → 发送 release 命令')
            msg = String()
            msg.data = 'release'
            self.gripper_pub.publish(msg)
            return

        if elapsed < GRIPPER_OPEN_WAIT:
            return

        self.get_logger().info('✅ OPENING 完成 → 归位')
        self._state = 'RESETTING'
        self._state_start_time = time.time()
        self._cmd_sent = False

    def _do_resetting(self):
        """发送归位指令，等待固件 MOTION_RESET 时间完成后回到 IDLE"""
        if not self._cmd_sent:
            cmd = ArmJointAngles()
            cmd.angles = [math.radians(a) for a in RESET_ANGLES] + [self.current_angles[5]]
            cmd.motion_time_ms = MOTION_RESET
            self.cmd_pub.publish(cmd)
            self._cmd_sent = True
            self._state_start_time = time.time()  # 从发命令时刻开始计时归位
            self.get_logger().info(
                f'归位: [{", ".join(f"{a:+.0f}" for a in RESET_ANGLES)}] ° | 夹爪保持打开 | 等待 {MOTION_RESET}ms'
            )
            return

        if time.time() - self._state_start_time < MOTION_RESET / 1000.0:
            return

        self.get_logger().info('✅ RESETTING 完成 → 准备下一次抓取')
        self._last_catch = time.time()
        self._state = 'IDLE'


def main():
    rclpy.init()
    node = ArmCatchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()