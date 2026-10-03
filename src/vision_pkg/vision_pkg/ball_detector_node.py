"""
小球检测与3D定位节点

image_callback: 只检测像素, 发布 /ball_pixel (供居中伺服, 不计算深度, 不做TF)
/get_goal:     居中完成后调用, 此时才做完整3D解算 (深度 + 手眼变换)

流程: 先像素居中 → 再TF解算目标点 → 再IK (与 ball_locator.py 一致)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from arm_msg.msg import ArmJointAngles
from target_msg.srv import GetGoal
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import yaml
import math
import time


# ── 默认参数 (校准文件不存在时使用) ──
DEFAULT_HSV = (0, 20, 152, 255, 172, 255)
DEFAULT_MORPH = (0, 0)
DEFAULT_MIN_AREA = 1000


def _load_hsv_params():
    """从 hsv_params.yaml 加载校准参数, 不存在则返回默认值"""
    cfg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'hsv_params.yaml')
    if os.path.exists(cfg_path):
        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f)
            if data:
                h = (data['h_min'], data['h_max'],
                     data['s_min'], data['s_max'],
                     data['v_min'], data['v_max'])
                m = (data.get('erode_iter', 0), data.get('dilate_iter', 0))
                a = data.get('min_area', 1000)
                return h, m, a
    return DEFAULT_HSV, DEFAULT_MORPH, DEFAULT_MIN_AREA


# 加载校准参数
HSV_THRESH, MORPH_ITER, MIN_AREA = _load_hsv_params()

# 球直径 / 半径
BALL_DIAMETER = 0.040          # 球直径 4cm
BALL_RADIUS = BALL_DIAMETER / 2.0   # 球半径 2cm (表面中心 → 球心补偿)

# 最近球目标锁定迟滞系数: 两球深度相近时, 距离变换全局峰值会在两球间跳变。
# 只有当新候选峰值比上一帧跟踪球大 TRACK_HYSTERESIS 倍时才切换目标, 避免识别框反复跳。
TRACK_HYSTERESIS = 1.15

# 连续性锁定参数 (image_callback 用): 多球时保持锁定目标, 防止机械臂逼近/转动时目标反复切换
LOCK_RADIUS_RATIO = 1.5    # 锁定半径 = 上一帧球直径 × 此系数
LOCK_RADIUS_MIN = 60.0     # 锁定半径下限 (px), 球较小时也保持足够关联范围
LOCK_HOLD_TIME = 1.0       # 目标球暂时丢失(被遮挡/移出)后保持锁定的时间 (s)

# 闭运算迭代次数: 先膨胀后腐蚀, 填补球 mask 的小缺口/裂缝
# (球面反光/颜色不均导致分割缺块, 短轴内切圆半径会偏小, 框只圈住球的一部分)
CLOSE_ITER = 1


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.bridge = CvBridge()

        self._load_camera_intrinsic()
        self._load_hand_eye()

        self.current_angles = [0.0] * 5
        self._joint_received = False  # 是否收到过 /joint_msg (机械臂真实角度就绪)

        # 缓存 (像素检测持续更新, 3D解算仅在 /get_goal 时做)
        self._latest_frame = None    # 最新帧 (供 /get_goal 解算)
        self._latest_px = None       # 最新像素 (cx, cy)
        self._tracked = None         # 持久锁定目标 (cx, cy, pixel_diameter), 抓完当前球前不切换
        self._tracked_lost_at = None  # 锁定目标丢失时刻 (monotonic), None=未丢失

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))

        self.joint_sub = self.create_subscription(
            ArmJointAngles, '/joint_msg', self.joint_callback, 10)

        self.pixel_pub = self.create_publisher(Float32MultiArray, '/ball_pixel', 10)
        self.pos_pub = self.create_publisher(Float32MultiArray, '/ball_pos', 10)
        self.balls_pub = self.create_publisher(Float32MultiArray, '/balls_pixel', 10)

        self.goal_srv = self.create_service(GetGoal, '/get_goal', self.get_goal_callback)

        self.has_display = os.environ.get('DISPLAY') is not None
        if not self.has_display:
            self.get_logger().info('No display detected, GUI disabled')

        self.get_logger().info(
            f'Ball detector node started. '
            f'Camera: {self.fx:.1f}x{self.fy:.1f}, '
            f'Hand-eye: {"loaded" if self.has_handeye else "missing"}'
        )

    def _load_camera_intrinsic(self):
        share_dir = get_package_share_directory('vision_pkg')
        path = os.path.join(share_dir, 'camera_intrinsic.xml')
        if not os.path.exists(path):
            self.get_logger().warn(f'Camera intrinsic not found: {path}')
            self.fx = self.fy = self.cx = self.cy = 0.0
            self.dist_coeffs = np.zeros(5)
            self.K = np.eye(3)
            return

        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        self.K = fs.getNode('K').mat()
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]
        self.dist_coeffs = fs.getNode('distortion').mat().reshape(-1)
        self.f_eq = np.sqrt(self.fx * self.fy)
        fs.release()

    def _load_hand_eye(self):
        share_dir = get_package_share_directory('vision_pkg')
        path = os.path.join(share_dir, 'hand_eye_result.xml')
        self.has_handeye = False
        self.R_cam2link4 = np.eye(3)
        self.t_cam2link4 = np.zeros(3)

        if not os.path.exists(path):
            self.get_logger().warn(f'Hand-eye calibration not found: {path}')
            return

        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            return
        self.R_cam2link4 = fs.getNode('R').mat()
        self.t_cam2link4 = fs.getNode('t').mat().reshape(3)
        fs.release()
        self.has_handeye = True

    def joint_callback(self, msg: ArmJointAngles):
        self._joint_received = True
        self.current_angles = list(msg.angles[:5])

    # ── 像素检测 (每帧, 不计算深度, 不做TF) ──

    def image_callback(self, msg: Image):
        """检测所有球像素中心 + 可视化, 发布 /balls_pixel 与 /ball_pixel(最近球)。不计算深度, 不做TF变换。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        self._latest_frame = cv_image
        display = cv_image.copy()

        # 检测所有球, 按像素直径降序(近→远)
        balls = self._detect_balls(cv_image)

        if not balls:
            self._latest_px = None
            # 锁定目标短时丢失(抓取瞬间遮挡/单帧丢帧): 保持锁定, 超时才释放
            if self._tracked is not None:
                if self._tracked_lost_at is None:
                    self._tracked_lost_at = time.monotonic()
                elif time.monotonic() - self._tracked_lost_at > LOCK_HOLD_TIME:
                    self._tracked = None
                    self._tracked_lost_at = None
            # 无球时也发布哨兵, 让 arm_catch_node 知道检测节点已就绪, 否则永远不会触发无球扫描
            px_msg = Float32MultiArray()
            px_msg.data = [-1.0, -1.0, 0.0]
            self.pixel_pub.publish(px_msg)
            balls_msg = Float32MultiArray()
            balls_msg.data = []
            self.balls_pub.publish(balls_msg)
            if self.has_display:
                cv2.rectangle(display, (0, 0), (display.shape[1], 30), (50, 50, 50), -1)
                cv2.putText(display, 'No ball detected', (10, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                cv2.imshow('Ball Detector', display)
                cv2.waitKey(1)
            return

        # 持久锁定: 一旦锁定最近球, 持续追踪同一颗球直到被抓走, 不因其它球更近而切换
        if self._tracked is not None:
            lcx, lcy, ld = self._tracked
            lock_radius = max(ld * LOCK_RADIUS_RATIO, LOCK_RADIUS_MIN)
            best = None
            best_d = float('inf')
            for b in balls:
                d = math.hypot(b[0] - lcx, b[1] - lcy)
                if d < best_d:
                    best_d = d
                    best = b
            if best is not None and best_d <= lock_radius:
                # 锁定球仍在视野: 继续锁定并更新其最新位置/直径
                target = best
                self._tracked = (best[0], best[1], best[2])
                self._tracked_lost_at = None
            else:
                # 锁定球已消失(被抓走): 释放锁定, 重新锁定当前最近的球
                self._tracked = None
                self._tracked_lost_at = None
                target = balls[0]
                self._tracked = (target[0], target[1], target[2])
        else:
            # 无锁定: 锁定当前最近球
            target = balls[0]
            self._tracked = (target[0], target[1], target[2])
            self._tracked_lost_at = None

        cx, cy, pixel_diameter, bbox = target
        self._latest_px = (cx, cy)
        bx, by, bw, bh = bbox

        # 发布所有球 (近→远): 每球 [cx, cy, diameter]
        balls_msg = Float32MultiArray()
        flat = []
        for (bcx, bcy, bd, _bb) in balls:
            flat += [float(bcx), float(bcy), float(bd)]
        balls_msg.data = flat
        self.balls_pub.publish(balls_msg)

        # 发布最近球 (保持兼容, 供居中伺服)
        px_msg = Float32MultiArray()
        px_msg.data = [float(cx), float(cy), pixel_diameter]
        self.pixel_pub.publish(px_msg)

        # 实时 base_link 坐标 (最近球, 供底盘桥接做闭环)
        base = self._pixel_to_base(cx, cy, pixel_diameter)
        if base is not None:
            pos_msg = Float32MultiArray()
            pos_msg.data = [float(base[0]), float(base[1]), float(base[2])]
            self.pos_pub.publish(pos_msg)

        if self.has_display:
            for i, (bcx, bcy, bd, bbx) in enumerate(balls):
                x0, y0, w, h = bbx
                color = (0, 0, 255) if i == 0 else (255, 0, 0)  # 最近红, 其余蓝
                cv2.rectangle(display, (x0, y0), (x0 + w, y0 + h), color, 2)
                cv2.circle(display, (int(bcx), int(bcy)), 5, (0, 255, 0), cv2.FILLED)
                cv2.line(display, (int(bcx) - 20, int(bcy)), (int(bcx) + 20, int(bcy)), (0, 255, 0), 1)
                cv2.line(display, (int(bcx), int(bcy) - 20), (int(bcx), int(bcy) + 20), (0, 255, 0), 1)
                cv2.putText(display, f'#{i+1}', (x0, y0 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 最近球像素偏移标注
            dx = int(cx) - int(self.cx)
            dy = int(cy) - int(self.cy)
            cv2.putText(display, f'dx={dx:+d} dy={dy:+d} px',
                        (bx, by - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 状态栏
            cv2.rectangle(display, (0, 0), (display.shape[1], 30), (50, 50, 50), -1)
            cv2.putText(display, f'{len(balls)} ball(s) | nearest offset ({dx:+d},{dy:+d})',
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            cv2.imshow('Ball Detector', display)
            cv2.waitKey(1)

    def _detect_balls(self, frame):
        """
        检测所有球, 不估算深度。
        不做 undistort 去畸变 (省 CPU 提帧率; 畸变对中心偏移影响 <2px)。
        返回: [(cx, cy, pixel_diameter, (x, y, w, h)), ...] 按像素直径降序(近→远)
        """
        if self.fx == 0.0:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min, h_max, s_min, s_max, v_min, v_max = HSV_THRESH
        mask = cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))

        e_iter, d_iter = MORPH_ITER
        kernel = np.ones((5, 5), np.uint8)
        if e_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=e_iter)
        if d_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=d_iter)

        # 闭运算: 先膨胀后腐蚀, 填补球 mask 的小缺口/裂缝, 让球恢复完整实心圆
        # (否则距离变换峰值(短轴)只落在球的一部分, 识别框圈不全)
        if CLOSE_ITER > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=CLOSE_ITER)

        return self._find_balls(mask)

    def _find_balls(self, mask):
        """
        找所有球。balls[0] 球心用距离变换峰值定位前方球(最近球),
        半径用外接矩形内切圆 min(w,h)/2 估算; 其余球用 findContours (仅显示)。
        抓取只需前方球, 前方抓完后后方自然露出, 无需完整分割重叠球。
        """
        if cv2.countNonZero(mask) == 0:
            return []

        # 先取外轮廓, 再把轮廓内部填实: 球面反光造成的内部空洞会被填平,
        # 距离变换峰值就能回到真实球心, 而不是被空洞挤开。
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask_filled = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(mask_filled, contours, -1, 255, cv2.FILLED)

        # 距离变换: 在填实后的 mask 上做, 峰值 = 最大内切圆圆心 = 前方球球心
        dist = cv2.distanceTransform(mask_filled, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist)

        cx = float(max_loc[0])
        cy = float(max_loc[1])

        # 迟滞锁定球心: 两球深度相近时, 全局峰值会在两球间反复跳。
        if self._tracked is not None:
            tcx, tcy, td = self._tracked
            tr = td / 2.0
            if math.hypot(cx - tcx, cy - tcy) > tr:
                tx = int(round(tcx))
                ty = int(round(tcy))
                h, w = dist.shape
                if 0 <= tx < w and 0 <= ty < h and dist[ty, tx] > 0:
                    if max_val <= dist[ty, tx] * TRACK_HYSTERESIS:
                        cx, cy = tcx, tcy

        # 找包含球心的轮廓, 用外接矩形内切圆半径 min(w,h)/2 作为球半径。
        # 比纯内切圆(距离变换峰值)圈得全, 比外接圆更抗噪点/斜椭圆。
        balls = []
        nearest_contour = None
        radius = 0.0
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            if cv2.pointPolygonTest(cnt, (cx, cy), False) >= 0:
                x, y, w, h = cv2.boundingRect(cnt)
                er = min(w, h) / 2.0
                if er > radius:
                    radius = er
                    nearest_contour = cnt

        if radius > 0:
            pixel_diameter = 2.0 * radius
            r = int(round(radius))
            bx = int(cx) - r
            by = int(cy) - r
            balls.append((cx, cy, pixel_diameter, (bx, by, 2 * r, 2 * r)))

        # 其余球: findContours (供 /balls_pixel 显示, 不参与抓取)
        rest = []
        nearest_c = (balls[0][0], balls[0][1]) if balls else None
        for cnt in contours:
            if nearest_contour is not None and cnt is nearest_contour:
                continue
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            if nearest_c is not None and cv2.pointPolygonTest(cnt, nearest_c, False) >= 0:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            (_, er) = cv2.minEnclosingCircle(cnt)
            rest.append((float(x + w // 2), float(y + h // 2), 2.0 * er, (x, y, w, h)))
        rest.sort(key=lambda b: b[2], reverse=True)

        return balls + rest

    # ── 完整3D解算 (仅在 /get_goal 被调用时, 居中完成后) ──

    def get_goal_callback(self, request, response):
        """
        服务 /get_goal: 居中完成后调用, 此时才做完整 3D 解算。
        与 ball_locator.py 的 locate() 一致。
        """
        if self._latest_frame is None:
            response.success = False
            self.get_logger().warn('/get_goal: 无图像帧')
            return response

        # 手眼变换使用的关节角来自 /joint_msg (arm_joint_node 实时读取 $GAA! 真实角度)
        self.get_logger().info(
            f'解算前关节角: [{", ".join(f"{math.degrees(a):+.1f}" for a in self.current_angles[:5])}] °'
        )

        base = self._locate(self._latest_frame)
        if base is not None:
            response.goal.x = base[0]
            response.goal.y = base[1]
            response.goal.z = base[2]
            response.success = True
            self.get_logger().info(
                f'/get_goal: ({base[0]*100:.1f}, {base[1]*100:.1f}, {base[2]*100:.1f}) cm'
            )
        else:
            response.success = False
            self.get_logger().warn(
                f'/get_goal: 无有效目标 (handeye={self.has_handeye}, '
                f'angles={len(self.current_angles)}, px={self._latest_px})'
            )

        return response

    def _pixel_to_base(self, cx, cy, pixel_diameter):
        """从像素中心 + 球像素直径 实时估算 base_link 坐标 (复用 /get_goal 的手眼变换)"""
        if self.fx == 0.0 or pixel_diameter <= 0:
            return None
        depth = (BALL_DIAMETER * self.f_eq) / pixel_diameter
        if not self.has_handeye or not self._joint_received:
            return None

        X_cam = (cx - self.cx) * depth / self.fx
        Y_cam = (cy - self.cy) * depth / self.fy
        p_cam = np.array([X_cam, Y_cam, depth])
        p_link4 = self.R_cam2link4 @ p_cam + self.t_cam2link4

        from arm_pkg.ik_solver import forward_kinematics_link4
        R_bl4, t_bl4 = forward_kinematics_link4(*self.current_angles[:4])
        p_base = R_bl4 @ p_link4 + t_bl4
        return p_base

    def _locate(self, frame):
        """
        完整3D定位: 深度估算 + 手眼变换 camera → link4 → base。
        与 ball_locator.py 的 detect_ball + locate 一致。
        """
        if self.fx == 0.0:
            return None

        frame_undist = cv2.undistort(frame, self.K, self.dist_coeffs)
        hsv = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2HSV)

        h_min, h_max, s_min, s_max, v_min, v_max = HSV_THRESH
        mask = cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))

        e_iter, d_iter = MORPH_ITER
        kernel = np.ones((5, 5), np.uint8)
        if e_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=e_iter)
        if d_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=d_iter)

        balls = self._find_balls(mask)
        if not balls:
            return None

        cx, cy, pixel_diameter, _ = balls[0]
        depth = (BALL_DIAMETER * self.f_eq) / pixel_diameter if pixel_diameter > 0 else None
        if depth is None:
            return None

        # 手眼变换: camera → link4 → base
        if not self.has_handeye or len(self.current_angles) < 4:
            return None

        X_cam = (cx - self.cx) * depth / self.fx
        Y_cam = (cy - self.cy) * depth / self.fy

        p_cam = np.array([X_cam, Y_cam, depth])
        p_link4 = self.R_cam2link4 @ p_cam + self.t_cam2link4

        from arm_pkg.ik_solver import forward_kinematics_link4
        R_bl4, t_bl4 = forward_kinematics_link4(*self.current_angles[:4])
        p_base = R_bl4 @ p_link4 + t_bl4

        z = float(p_base[2])
        z = max(z, -0.11)  # 识别 z 下限: 机械臂最低可达 -11cm, 低于此视为估计误差
        return (float(p_base[0]), float(p_base[1]), z)


def main():
    rclpy.init()
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()