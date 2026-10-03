import rclpy
from rclpy.node import Node
from arm_msg.msg import ArmJointAngles
from arm_interfaces.srv import SetJointAngle, GetJointAngles
from std_msgs.msg import String
import serial
import threading
import time
import re
import math

# ── 常量 ──────────────────────────────────────────
BAUD = 115200
SERVO_NUM = 6
ANGLE_LIMITS = {
    0: 180.0,  # joint1: ±180°
    1: 135.0,  # joint2: ±135°
    2: 135.0,  # joint3: ±135°
    3: 135.0,  # joint4: ±135°
    4: 135.0,  # joint5: ±135°
    5: 135.0,  # endjoint1: ±135°
}


# ── 串口通信线程 ──────────────────────────────────
class SerialThread(threading.Thread):
    def __init__(self, port, callback):
        super().__init__(daemon=True)
        self.port = port
        self.callback = callback
        self.ser = None
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.05)
            time.sleep(3.0)  # 等待下位机上电自检完成(约2~3秒, 自检期间 $GAA! 不返回角度)
            self.ser.reset_input_buffer()  # 清空输入缓冲区(丢弃自检消息)
            self.ser.reset_output_buffer()  # 清空输出缓冲区
            self.callback("connected", self.port)
        except Exception as e:
            self.callback("error", str(e))
            return
        buf = ""
        while self.running:
            try:
                # 直接用 read(1) + timeout 机制，避免 in_waiting 竞态条件
                data = self.ser.read(1024)
                if data:
                    buf += data.decode("utf-8", errors="replace")
                    while "!" in buf:
                        idx = buf.index("!")
                        line = buf[:idx+1]
                        buf = buf[idx+1:]
                        self.callback("rx", line.strip())
            except serial.SerialException as e:
                self.callback("error", str(e))
                time.sleep(0.1)
            except Exception as e:
                self.callback("error", str(e))
                break

    def send(self, cmd):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd + "\n").encode())
                self.callback("tx", cmd)
            else:
                self.callback("error", f"串口未打开，命令未发送: {cmd}")

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()


# ── ROS2 节点 ─────────────────────────────────────
class ArmJointNode(Node):
    def __init__(self):
        super().__init__('arm_joint_node')

        self.declare_parameter('serial_port', '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0')
        self.declare_parameter('angle_read_rate', 2.0)  # 关节角实时读取频率 (Hz)

        # 当前角度值 (弧度) - 来自下位机 $GAA! 真实反馈
        self.current_angles = [0.0] * SERVO_NUM
        self.angles_lock = threading.Lock()

        # 真实角度读取事件 (供 /get_joint_angles 服务同步等待 $GAA! 响应)
        self._angles_read_event = threading.Event()

        # 发布器: /joint_msg
        self.publisher_ = self.create_publisher(ArmJointAngles, '/joint_msg', 10)

        # 订阅器: /cmd_angles
        self.cmd_sub = self.create_subscription(
            ArmJointAngles, '/cmd_angles', self.cmd_angles_callback, 10)

        # 订阅器: /gripper_cmd (夹爪控制: "catch" / "release")
        self.gripper_sub = self.create_subscription(
            String, '/gripper_cmd', self.gripper_cmd_callback, 10)

        # 服务: /set_joint_angle
        self.set_joint_srv = self.create_service(
            SetJointAngle, '/set_joint_angle', self.set_joint_angle_callback)

        # 服务: /get_joint_angles (读取真实电机角度)
        self.get_joint_srv = self.create_service(
            GetJointAngles, '/get_joint_angles', self.get_joint_angles_callback)

        # 串口通信
        self.serial_thread = None
        self._init_serial()

        # 定时发布 joint_msg (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 定时读取角度 (实时) - 供手眼变换/底盘逼近使用
        read_rate = self.get_parameter('angle_read_rate').value
        self.read_timer = self.create_timer(1.0 / read_rate, self.read_angles_callback)

        self.get_logger().info('Arm joint node has been started.')

    def _init_serial(self):
        port = self.get_parameter('serial_port').value
        self.get_logger().info(f'Connecting to serial port: {port}')
        self.serial_thread = SerialThread(port, self._on_serial_event)
        self.serial_thread.start()

    def _on_serial_event(self, event, data):
        if event == "connected":
            self.get_logger().info(f'Serial connected: {data}')
        elif event == "tx":
            self.get_logger().debug(f'TX: {data}')
        elif event == "rx":
            self._parse_response(data)
        elif event == "error":
            # 静默处理瞬态读取错误（设备报告可读但实际无数据）
            if "device reports readiness to read but returned no data" in str(data):
                return  # 忽略这种瞬态错误
            self.get_logger().error(f'Serial error: {data}')

    def _parse_response(self, data):
        # 解析角度响应: #AA+0.0,+2.5,-1.0,...!
        # $GAA! 返回真实角度(停稳后)。直接更新当前角度供手眼变换/IK使用。
        m = re.match(r"#AA([-+0-9.,]+)!", data)
        if m:
            parts = m.group(1).split(",")
            with self.angles_lock:
                for i, p in enumerate(parts[:SERVO_NUM]):
                    try:
                        deg = float(p)
                        self.current_angles[i] = deg * math.pi / 180.0
                    except Exception:
                        pass
            # 通知等待中的 /get_joint_angles 服务
            self._angles_read_event.set()
            return

    def timer_callback(self):
        msg = ArmJointAngles()
        with self.angles_lock:
            msg.angles = list(self.current_angles)
        self.publisher_.publish(msg)

    def read_angles_callback(self):
        if self.serial_thread:
            self.serial_thread.send("$GAA!")

    def get_joint_angles_callback(self, request, response):
        """服务 /get_joint_angles: 同步读取真实电机角度"""
        if not self.serial_thread or not self.serial_thread.ser or not self.serial_thread.ser.is_open:
            response.success = False
            response.message = '串口未连接'
            return response

        # 发送读取请求并等待真实反馈
        self._angles_read_event.clear()
        self.serial_thread.send("$GAA!")
        if self._angles_read_event.wait(timeout=1.0):
            with self.angles_lock:
                response.angles = list(self.current_angles)
            response.success = True
            response.message = 'ok'
        else:
            response.success = False
            response.message = '读取真实角度超时'
        return response

    def cmd_angles_callback(self, msg):
        if not self.serial_thread:
            self.get_logger().warn('Serial not connected, ignoring cmd_angles')
            return
        degs = [f'{pos*180.0/math.pi:.1f}°' for pos in msg.angles[:SERVO_NUM]]
        self.get_logger().info(f'收到 cmd_angles: [{", ".join(degs)}]')
        motion_time = getattr(msg, 'motion_time_ms', 0) or 0
        with self.angles_lock:
            for i, pos in enumerate(msg.angles[:SERVO_NUM]):
                deg = pos * 180.0 / math.pi
                limit = ANGLE_LIMITS.get(i, 135.0)
                deg = max(-limit, min(limit, deg))
                angle_deci = int(deg * 10)
                if motion_time > 0:
                    cmd = f"$SA:{i},{angle_deci},{motion_time}!"
                else:
                    cmd = f"$SA:{i},{angle_deci}!"
                self.serial_thread.send(cmd)
        self.get_logger().info(f'Sent {len(msg.angles[:SERVO_NUM])} joint commands')

    def send_servo_cmd(self, idx, angle_deg):
        """发送单个舵机角度(度)"""
        if not self.serial_thread:
            self.get_logger().warn('串口未连接，无法发送命令')
            return
        limit = ANGLE_LIMITS.get(idx, 135.0)
        deg = max(-limit, min(limit, angle_deg))
        angle_deci = int(deg * 10)
        cmd = f"$SA:{idx},{angle_deci}!"
        self.get_logger().info(f'发送命令: {cmd}')
        self.serial_thread.send(cmd)

    def gripper_cmd_callback(self, msg: String):
        """夹爪控制: 接收 "catch" 或 "release" 指令, 直接发送串口"""
        if not self.serial_thread:
            self.get_logger().warn('串口未连接，无法发送夹爪命令')
            return
        data = msg.data.strip().lower()
        if data in ('catch', 'release'):
            cmd = f"{data};"
            self.get_logger().info(f'夹爪命令: {cmd}')
            self.serial_thread.send(cmd)
        else:
            self.get_logger().warn(f'无效夹爪命令: "{data}", 应为 catch 或 release')

    def set_joint_angle_callback(self, request, response):
        """服务回调: 设置单个关节角度"""
        joint_id = request.joint_id
        angle = request.angle

        # 验证关节ID
        if joint_id < 0 or joint_id >= SERVO_NUM:
            response.success = False
            response.message = f"关节ID错误: {joint_id}, 应为 0-{SERVO_NUM-1}"
            self.get_logger().warn(response.message)
            return response

        # 验证角度范围
        limit = ANGLE_LIMITS.get(joint_id, 135.0)
        if angle < -limit or angle > limit:
            response.success = False
            response.message = f"角度超范围: {angle:.1f}°, 应为 ±{limit}°"
            self.get_logger().warn(response.message)
            return response

        # 发送命令
        self.send_servo_cmd(joint_id, angle)
        response.success = True
        response.message = f"关节{joint_id} → {angle:+.1f}°"
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = ArmJointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
