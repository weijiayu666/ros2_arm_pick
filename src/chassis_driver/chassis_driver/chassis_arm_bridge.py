"""
底盘-机械臂协作桥接节点 — 只控制底盘, 离散二分逼近靠近目标

流程 (IK 失败后):
  1. 先左右对准球 (x→0)
  2. 朝动态目标 y (由球高度 z 决定) 走当前偏差的一半
  3. 停下等稳定, 用实时 /ball_pos 重新估算 y
  4. 重复 1~3, 直到 y 接近目标
  5. 到位后交还 arm 重试
"""
import rclpy
from rclpy.node import Node
from chassis_msg.msg import ChassisCmd, ChassisState
from std_msgs.msg import Float32MultiArray, String, Bool, Float32
import time
import math


# ── 目标与工作范围 (base_link 坐标系, 米) ──
# y=前进, x=右, z=上
Y_TARGET = 0.20       # 默认抓取理想点 (无 z 信息时用)
X_TARGET = 0.0        # x 对准目标: 正对
X_TOLERANCE = 0.10    # x 对准容差 (±10cm)

# ── 机械臂几何 (与 arm_catch_node 的 WORKSPACE 一致) ──
ARM_REACH = 0.34      # 臂长/可达半径 (m)
ARM_Z_CENTER = 0.056  # joint1 高度, 半球球心 z (m)
ARRIVE_MARGIN = 0.01  # 到位时 r 相对可达半径的安全余量 (m)

# ── 两档固定速度 (开环 PWM 标定) ──
FAST_LINEAR = 0.18    # 快速档 (m/s): duty=0.5 → 54cm/3s
SLOW_LINEAR = 0.047   # 慢速档 (m/s): duty=0.2 → 14cm/3s
GEAR_SWITCH = 0.35    # |y-目标| 超过此值用快速档, 否则慢速档

# ── 转向 ──
KP_X = 3.0
MAX_ANGULAR = 0.50

# ── 时序 ──
SETTLE_TIME = 1.0         # 停下后等待稳定 (s)
BALL_LOST_TIMEOUT = 0.5   # 球丢失容忍 (s)
MOVE_MAX_TIME = 8.0       # 单步移动超时上限 (s)
MAX_ITER = 12             # 二分逼近最大迭代次数

# ── 球太近后退 ──
BACK_OFF_DISTANCE = 0.05  # 球面积过大(太近)时后退距离 (m)
BACK_OFF_SPEED = 0.18     # 后退速度 (m/s), 复用快速档


class ChassisArmBridge(Node):
    """底盘-机械臂协作桥接 — 只控制底盘, 离散二分逼近"""

    def __init__(self):
        super().__init__('chassis_arm_bridge')

        # ── 内部状态 ──
        self._state = 'IDLE'  # IDLE | ALIGN_X | MOVE_Y | SETTLE | BACK_OFF
        self.arm_state = ''
        self.arm_prev_state = ''
        self.chassis_online = False

        # arm 发布的失败目标坐标 (base_link)
        self._fail_target = None    # [x, y, z] 米

        # 实时球坐标 (base_link) + 时间戳
        self._ball_pos = None       # [x, y, z, timestamp]

        # 二分逼近状态
        self._iter = 0
        self._cur_linear = 0.0
        self._cur_gear = 'slow'      # 当前挡位: 'fast' 快速挡 / 'slow' 慢速挡
        self._move_deadline = 0.0
        self._settle_deadline = 0.0
        self._y_target = Y_TARGET  # 当前逼近目标 y (根据球高度 z 动态调整)
        self._r_max = Y_TARGET     # 机械臂实际可达半径 (由球高 z 算出), 用于到位判断

        # ── 订阅 ──
        self.arm_status_sub = self.create_subscription(
            String, '/arm_status', self._arm_status_cb, 10)
        self.fail_target_sub = self.create_subscription(
            Float32MultiArray, '/ik_fail_target', self._fail_target_cb, 10)
        self.ball_pos_sub = self.create_subscription(
            Float32MultiArray, '/ball_pos', self._ball_pos_cb, 10)
        self.chassis_state_sub = self.create_subscription(
            ChassisState, '/chassis_state', self._chassis_state_cb, 10)
        self.back_off_sub = self.create_subscription(
            Bool, '/back_off', self._back_off_cb, 10)
        self.back_off_dist_sub = self.create_subscription(
            Float32, '/back_off_dist', self._back_off_dist_cb, 10)

        # ── 发布 ──
        self.chassis_cmd_pub = self.create_publisher(
            ChassisCmd, '/chassis_cmd', 10)
        self.chassis_busy_pub = self.create_publisher(
            Bool, '/chassis_busy', 10)

        # ── 定时器 (20Hz) ──
        self._timer = self.create_timer(0.05, self._state_machine)

        self.get_logger().info('底盘桥接已启动 (离散二分逼近)')

    # ═══════════════════════════════════════════
    # 回调
    # ═══════════════════════════════════════════

    def _arm_status_cb(self, msg: String):
        self.arm_prev_state = self.arm_state
        self.arm_state = msg.data
        if self.arm_prev_state != self.arm_state:
            self.get_logger().info(f'机械臂状态: {self.arm_prev_state} → {self.arm_state}')

        # arm 进入 SOLVING 时预占 busy, 防止 IK 失败后 arm_catch_node 立即重启
        if self.arm_prev_state != 'SOLVING' and self.arm_state == 'SOLVING':
            self.set_busy(True)

        # 成功离开 SOLVING (进入 GRASPING): 清除 busy, 否则抓取完成后 busy 永久滞留 True,
        # arm_catch_node._check_auto_start() 会一直 return, 无法开始第二次抓取
        if self.arm_prev_state == 'SOLVING' and self.arm_state == 'GRASPING':
            self.set_busy(False)

    def _fail_target_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._fail_target = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
            # 根据目标高度 z 动态计算底盘应靠近到的 y (半球可达约束: r<=sqrt(reach²-(z-z_center)²))
            z = self._fail_target[2]
            dz = z - ARM_Z_CENTER
            r_max = math.sqrt(max(0.0, ARM_REACH**2 - dz**2)) if abs(dz) < ARM_REACH else 0.0
            self._r_max = r_max
            self._y_target = min(Y_TARGET, max(0.08, r_max * 0.9))  # 需要靠近时减小, 上限保持默认 20cm
            self.get_logger().info(
                f'arm 目标坐标 (base_link): x={self._fail_target[0]*100:.0f}cm '
                f'y={self._fail_target[1]*100:.0f}cm z={self._fail_target[2]*100:.0f}cm, '
                f'目标 y={self._y_target*100:.0f}cm'
            )
            # 收到最新失败坐标后触发二分逼近
            self._start_adjust()

    def _ball_pos_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._ball_pos = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2]), time.time()]

    def _chassis_state_cb(self, msg: ChassisState):
        self.chassis_online = True

    def _back_off_cb(self, msg: Bool):
        """球太近 → 后退固定距离 5cm"""
        if msg.data:
            self._back_off_by(BACK_OFF_DISTANCE)

    def _back_off_dist_cb(self, msg: Float32):
        """指定距离后退 (米)"""
        self._back_off_by(abs(float(msg.data)))

    def _back_off_by(self, dist: float):
        """按指定距离后退"""
        if dist <= 0:
            return
        if self._state != 'IDLE':
            self.get_logger().warn(f'状态 {self._state}, 忽略后退请求')
            return
        self.get_logger().info(f'收到后退请求, 后退 {dist * 100:.0f}cm')
        self.set_busy(True)
        self._cur_linear = -BACK_OFF_SPEED
        self._move_deadline = time.time() + dist / BACK_OFF_SPEED
        self._state = 'BACK_OFF'
        self.send_chassis_cmd(self._cur_linear, 0.0)

    # ═══════════════════════════════════════════
    # 状态机
    # ═══════════════════════════════════════════

    def _ball_fresh(self):
        return self._ball_pos is not None and (time.time() - self._ball_pos[3]) < BALL_LOST_TIMEOUT

    def _in_work_range(self):
        """用实时球坐标做水平距离判断: r=√(x²+y²) ≤ 实际可达半径(留余量)"""
        if self._ball_pos is None:
            return False
        x = self._ball_pos[0]
        y = self._ball_pos[1]
        r = math.sqrt(x * x + y * y)
        return r <= self._r_max - ARRIVE_MARGIN

    def _state_machine(self):
        if self._state == 'ALIGN_X':
            self._do_align_x()
        elif self._state == 'MOVE_Y':
            self._do_move_y()
        elif self._state == 'SETTLE':
            self._do_settle()
        elif self._state == 'BACK_OFF':
            self._do_back_off()

    def _start_adjust(self):
        """IK 失败后: 启动离散二分逼近"""
        if self._state != 'IDLE':
            self.get_logger().warn(f'状态 {self._state}, 无法启动调整')
            return

        if self._fail_target is None and not self._ball_fresh():
            self.get_logger().warn('无球坐标, 无法启动调整')
            self._reset()
            return

        self._iter = 0
        self.get_logger().info('开始底盘二分逼近')
        self._state = 'ALIGN_X'

    def _do_align_x(self):
        """先左右对准球 (x→0)"""
        if not self._ball_fresh():
            self.get_logger().warn('对准阶段球丢失, 停止')
            self._reset()
            return

        x = self._ball_pos[0]
        y = self._ball_pos[1]

        # 未对准 → 持续转向
        if abs(x - X_TARGET) > X_TOLERANCE:
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, -(x - X_TARGET) * KP_X))
            self.send_chassis_cmd(0.0, angular)
            return

        # 已对准 → 停止转向, 走 y 半程
        self.stop_chassis()

        if self._in_work_range():
            self.get_logger().info(f'已进入工作范围 (r={math.sqrt(x*x + y*y)*100:.0f}cm), 完成')
            self._reset()
            return

        self._start_move_y(y)

    def _start_move_y(self, y):
        """计算并开始走当前 y 偏差的一半"""
        err = y - self._y_target
        distance = err / 2.0                     # 走偏差一半
        speed = FAST_LINEAR if abs(err) > GEAR_SWITCH else SLOW_LINEAR
        direction = 1.0 if err > 0 else -1.0
        self._cur_linear = direction * speed
        self._cur_gear = 'fast' if abs(err) > GEAR_SWITCH else 'slow'
        duration = min(abs(distance) / speed, MOVE_MAX_TIME)

        self._iter += 1
        if self._iter > MAX_ITER:
            self.get_logger().warn(f'超过最大迭代次数 {MAX_ITER}, 停止')
            self._reset()
            return

        self._move_deadline = time.time() + duration
        self._state = 'MOVE_Y'
        self.get_logger().info(
            f'第 {self._iter} 步: y={y*100:.0f}cm, 走 {distance*100:.1f}cm, '
            f'档位 {"快" if speed > 0.1 else "慢"} ({speed:.3f}m/s), 用时 {duration:.2f}s'
        )
        self.send_chassis_cmd(self._cur_linear, 0.0)

    def _do_move_y(self):
        """走半程: 实时球坐标提前停止 (分挡位) + 球丢失立刻停 + 计时兜底"""
        # 球检测不到 → 立刻停止前进 (不再继续盲走), 进入稳定重测(丢失则触发扫描)
        if not self._ball_fresh():
            self.get_logger().warn('前进中球丢失, 立刻停止')
            self.stop_chassis()
            self._settle_deadline = time.time() + SETTLE_TIME
            self._state = 'SETTLE'
            return

        # 球新鲜 → 分挡位实时提前停
        if self._cur_gear == 'fast':
            # 快速挡: 球进入慢速区间 → 提前停, 下一轮切慢速挡
            err = self._ball_pos[1] - self._y_target
            if abs(err) <= GEAR_SWITCH:
                self.get_logger().info('快速挡: 球已进入慢速区间, 提前停止切换慢速挡')
                self.stop_chassis()
                self._settle_deadline = time.time() + SETTLE_TIME
                self._state = 'SETTLE'
                return
        else:
            # 慢速挡: 球进入可达范围 → 提前停
            if self._in_work_range():
                self.get_logger().info('慢速挡: 球已进入可达范围, 提前停止')
                self.stop_chassis()
                self._settle_deadline = time.time() + SETTLE_TIME
                self._state = 'SETTLE'
                return

        # 计时上限兜底 (走完计划半程就停)
        if time.time() >= self._move_deadline:
            self.stop_chassis()
            self._settle_deadline = time.time() + SETTLE_TIME
            self._state = 'SETTLE'
            self.get_logger().info('走完, 停下等待稳定重测')
            return

        # 持续发送, 避免底盘看门狗超时停 (driver cmd_timeout=0.5s)
        self.send_chassis_cmd(self._cur_linear, 0.0)

    def _do_settle(self):
        """停下等待稳定, 然后重新估算 y"""
        if time.time() < self._settle_deadline:
            return

        if not self._ball_fresh():
            self.get_logger().warn('稳定后仍未检测到球, 停止')
            self._reset()
            return

        y = self._ball_pos[1]
        self.get_logger().info(f'稳定后重测 y={y*100:.0f}cm')

        if self._in_work_range():
            self.get_logger().info('已进入工作范围, 完成')
            self._reset()
            return

        # 未到位 → 回到对准, 继续下一轮二分
        self._state = 'ALIGN_X'

    def _do_back_off(self):
        """后退固定距离, 靠计时"""
        if time.time() >= self._move_deadline:
            self._reset()
            return
        self.send_chassis_cmd(self._cur_linear, 0.0)

    # ═══════════════════════════════════════════
    # 工具方法
    # ═══════════════════════════════════════════

    def send_chassis_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        cmd = ChassisCmd()
        cmd.linear_x = linear_x
        cmd.angular_z = angular_z
        self.chassis_cmd_pub.publish(cmd)

    def stop_chassis(self):
        self.send_chassis_cmd(0.0, 0.0)

    def set_busy(self, busy: bool):
        msg = Bool()
        msg.data = busy
        self.chassis_busy_pub.publish(msg)

    def _reset(self):
        """重置状态, 清除 busy"""
        self.stop_chassis()
        self.set_busy(False)
        self._state = 'IDLE'
        self.get_logger().info('底盘调整结束, 通知 arm_catch_node 重试')

    def destroy_node(self):
        self.stop_chassis()
        self.set_busy(False)
        super().destroy_node()


def main():
    rclpy.init()
    node = ChassisArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
