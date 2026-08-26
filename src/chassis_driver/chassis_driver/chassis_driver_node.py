"""
底盘驱动节点 — BDC 双路有刷电机控制板
支持开环 PWM 和闭环速度控制两种模式
串口协议: 115200 8N1, 文本命令
"""

import rclpy
from rclpy.node import Node
from chassis_msg.msg import ChassisState, ChassisCmd
import serial
import threading
import time
import re


# ========== 配置 ==========
# 底盘 BDC 控制板接在 ttyUSB1 (usb-0:1.3)。机械臂接 ttyUSB0 (usb-0:1.1)。
# 两个 CH340 芯片序列号相同(1a86_USB_Serial), by-id 无法区分, 必须用 by-path。
DEFAULT_PORT = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'
BAUD = 115200
STATE_RATE = 0.1       # 状态发布频率 (10Hz)
STATUS_INTERVAL = 2.0  # TEMP/STAT 查询间隔
CMD_TIMEOUT = 0.5      # 指令超时自动停止 (s)


class ChassisDriverNode(Node):
    """BDC 底盘驱动节点"""

    def __init__(self):
        super().__init__('chassis_driver_node')

        # 参数
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('wheel_radius', 0.06)     # 轮子半径 (m)
        self.declare_parameter('track_width', 0.27)      # 两轮间距 (m) (实测 ~27cm)
        self.declare_parameter('linear_gain', 5.0)      # 闭环: 线速度(m/s)→TARGET (实测 TARGET=1.0 → 0.20m/s)
        self.declare_parameter('angular_gain', 0.675)    # 闭环: 角速度(rad/s)→TARGET 差速 (= linear_gain*track_width/2)
        self.declare_parameter('max_duty', 1.0)           # 最大占空比
        self.declare_parameter('invert', True)           # 反转轮子方向 (默认当前接线)
        self.declare_parameter('swap_motors', True)      # 交换左右电机 (默认当前接线)
        self.declare_parameter('cmd_timeout', 0.5)        # 指令超时 (s)
        self.declare_parameter('use_closed_loop', False)  # 是否使用闭环速度控制 (先回退开环 PWM)
        self.declare_parameter('deadzone_duty', 0.094)    # 开环 PWM 死区占空比 (实测)
        self.declare_parameter('duty_slope', 0.443)       # 开环 PWM 斜率 (m/s)/占空比 (实测: 0.2→0.047, 0.5→0.18)

        # 串口线程
        self._ser = None
        self._ser_lock = threading.Lock()
        self._rx_thread = None
        self._running = False
        self._rx_buffer = b''

        # 遥测数据
        self._motor1_speed = 0.0
        self._motor2_speed = 0.0
        self._motor1_current = 0.0
        self._motor2_current = 0.0
        self._bus_voltage = 0.0
        self._temperature = 0.0
        self._protection_flags = 0
        self._is_moving = False

        # 指令看门狗
        self._cmd_deadline = 0.0
        self._last_left = 0.0
        self._last_right = 0.0

        # 状态定时器
        self._last_status_query = 0.0

        # 订阅 /chassis_cmd
        self.cmd_sub = self.create_subscription(
            ChassisCmd, '/chassis_cmd', self._cmd_callback, 10)

        # 发布 /chassis_state
        self.state_pub = self.create_publisher(ChassisState, '/chassis_state', 10)

        # 定时器
        self._state_timer = self.create_timer(STATE_RATE, self._publish_state)
        self._watchdog_timer = self.create_timer(0.05, self._watchdog)
        self._serial_timer = self.create_timer(0.1, self._serial_poll)

        # 初始化串口
        self._init_serial()

        self.get_logger().info('底盘驱动节点已启动')

    # ── 串口管理 ──

    def _init_serial(self):
        port = self.get_parameter('serial_port').value
        self.get_logger().info(f'连接底盘串口: {port}')
        try:
            self._ser = serial.Serial(port, BAUD, timeout=0.05)
            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self.get_logger().info(f'底盘串口已连接: {port}')

            # 初始化时查询状态
            time.sleep(0.3)
            self._send_raw('PING')
            self._send_raw('MODE')
        except serial.SerialException as e:
            self.get_logger().error(f'底盘串口连接失败: {e}')

    def _send_raw(self, cmd: str):
        """发送原始文本命令到串口"""
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((cmd + '\n').encode())
                except serial.SerialException as e:
                    self.get_logger().error(f'串口发送失败: {e}')

    def _rx_loop(self):
        """串口接收线程"""
        while self._running:
            try:
                with self._ser_lock:
                    if self._ser and self._ser.is_open and self._ser.in_waiting:
                        data = self._ser.read(self._ser.in_waiting)
                        self._rx_buffer += data
                self._parse_rx()
            except (serial.SerialException, OSError):
                pass
            time.sleep(0.02)

    def _parse_rx(self):
        """解析接收缓冲区中的行"""
        while b'\n' in self._rx_buffer:
            line_bytes, self._rx_buffer = self._rx_buffer.split(b'\n', 1)
            line = line_bytes.decode(errors='ignore').strip()
            if not line:
                continue
            self._parse_line(line)

    def _parse_line(self, line: str):
        """解析单行响应"""
        # CSV 数据流 (STREAM ON)
        if line.startswith('$'):
            self._parse_stream_csv(line)
            return

        # OK 响应
        if line.upper().startswith('OK'):
            rest = line[2:].strip()
            if rest.upper() == 'PONG':
                self.get_logger().debug('底盘 PING OK')
            elif 'TEMP' in rest.upper() or rest.replace('.', '').replace('-', '').isdigit():
                # TEMP 响应: OK 45.2
                self._parse_temp_value(rest)
            return

        # 错误
        if line.upper().startswith('ERR'):
            self.get_logger().warn(f'底盘错误: {line}')
            return

    def _parse_stream_csv(self, line: str):
        """解析 STREAM CSV 数据: $M1,I,RAW_I,VA,VB,VAB,VIN,P,DUTY,... M2,..."""
        try:
            # 去掉 $ 前缀
            data = line[1:].strip()
            # 格式: M1,<14个float>,M2,<14个float>
            parts = data.split(',')
            if len(parts) >= 16 and parts[0] == 'M1':
                # 电机1: 索引10=SPEED, 0=I, 5=VIN, 7=DUTY
                m1_end = 15  # M1 + 14 fields
                self._motor1_speed = self._safe_float(parts, 11)  # SPEED
                self._motor1_current = self._safe_float(parts, 1)  # I
                self._bus_voltage = self._safe_float(parts, 6)  # VIN
                self._motor2_speed = self._safe_float(parts, 11 + m1_end) if len(parts) > m1_end + 11 else 0.0
                self._motor2_current = self._safe_float(parts, 1 + m1_end) if len(parts) > m1_end + 1 else 0.0
        except Exception:
            pass

    def _parse_temp_value(self, text: str):
        """解析温度值"""
        try:
            self._temperature = float(text)
        except ValueError:
            pass

    @staticmethod
    def _safe_float(parts, idx):
        try:
            if idx < len(parts):
                return float(parts[idx])
        except (ValueError, IndexError):
            pass
        return 0.0

    # ── 指令回调 ──

    def _cmd_callback(self, msg: ChassisCmd):
        """接收 /chassis_cmd 运动指令"""
        linear = msg.linear_x
        angular = msg.angular_z

        max_duty = self.get_parameter('max_duty').value

        if self.get_parameter('use_closed_loop').value:
            # 闭环: 线/角速度 → TARGET 差速
            linear_gain = self.get_parameter('linear_gain').value
            angular_gain = self.get_parameter('angular_gain').value
            left = linear * linear_gain + angular * angular_gain
            right = linear * linear_gain - angular * angular_gain
        else:
            # 开环: 每轮目标线速度 → 占空比 (死区 + 斜率补偿)
            track_width = self.get_parameter('track_width').value
            v_left = linear + angular * track_width / 2.0
            v_right = linear - angular * track_width / 2.0
            left = self._vel_to_duty(v_left)
            right = self._vel_to_duty(v_right)

        # 反转 (如果接线方向反了)
        if self.get_parameter('invert').value:
            left = -left
            right = -right

        # 限制范围
        left = max(-max_duty, min(max_duty, left))
        right = max(-max_duty, min(max_duty, right))

        # 发送电机指令
        if self.get_parameter('use_closed_loop').value:
            self._send_target(left, right)
        else:
            self._send_pwm(left, right)

        # 更新看门狗
        self._cmd_deadline = time.time() + self.get_parameter('cmd_timeout').value
        self._last_left = left
        self._last_right = right
        self._is_moving = (abs(left) > 0.001 or abs(right) > 0.001)

        if self._is_moving:
            self.get_logger().debug(
                f'底盘: linear={linear:.2f} angular={angular:.2f} → L={left:.3f} R={right:.3f}'
            )

    def _vel_to_duty(self, v: float) -> float:
        """目标线速度 (m/s) → 开环占空比, 含死区补偿"""
        if abs(v) < 1e-4:
            return 0.0
        deadzone = self.get_parameter('deadzone_duty').value
        slope = self.get_parameter('duty_slope').value
        duty = deadzone + abs(v) / slope
        return duty if v > 0 else -duty

    def _send_pwm(self, left: float, right: float):
        """发送开环 PWM 指令"""
        if self.get_parameter('swap_motors').value:
            left, right = right, left
        self._send_raw(f'PWM DUTY={left:.2f},MOTOR=1')
        time.sleep(0.05)
        self._send_raw(f'PWM DUTY={right:.2f},MOTOR=2')

    def _send_target(self, left: float, right: float):
        """发送闭环速度指令"""
        if self.get_parameter('swap_motors').value:
            left, right = right, left
        self._send_raw(f'TARGET TARGET={left:.2f},MOTOR=1')
        time.sleep(0.05)
        self._send_raw(f'TARGET TARGET={right:.2f},MOTOR=2')

    def _stop(self):
        """停止所有电机"""
        if self.get_parameter('use_closed_loop').value:
            self._send_raw('TARGET TARGET=0.0,MOTOR=1')
            time.sleep(0.05)
            self._send_raw('TARGET TARGET=0.0,MOTOR=2')
        else:
            self._send_raw('PWM DUTY=0.0,MOTOR=1')
            time.sleep(0.05)
            self._send_raw('PWM DUTY=0.0,MOTOR=2')
        self._is_moving = False

    # ── 定时器 ──

    def _watchdog(self):
        """看门狗: 超时自动停止"""
        if self._is_moving and time.time() > self._cmd_deadline:
            self.get_logger().debug('指令超时, 自动停止')
            self._stop()

    def _serial_poll(self):
        """定期查询底盘状态"""
        now = time.time()
        if now - self._last_status_query > STATUS_INTERVAL:
            self._send_raw('TEMP') 
            self._last_status_query = now

    def _publish_state(self):
        """发布底盘状态"""
        msg = ChassisState()
        msg.motor1_speed = self._motor1_speed
        msg.motor2_speed = self._motor2_speed
        msg.motor1_current = self._motor1_current
        msg.motor2_current = self._motor2_current
        msg.bus_voltage = self._bus_voltage
        msg.temperature = self._temperature
        msg.is_moving = self._is_moving
        msg.protection_flags = self._protection_flags
        self.state_pub.publish(msg)

    # ── 清理 ──

    def destroy_node(self):
        self._running = False
        self._stop()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()