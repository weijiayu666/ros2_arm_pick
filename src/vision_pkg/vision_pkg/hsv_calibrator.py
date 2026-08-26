"""
HSV 校准工具 — 从 /image_raw 话题接收图像，手动调参后保存到 hsv_params.yaml

用法:
  ros2 run vision_pkg hsv_calibrator

操作:
  拖动滑块调整 HSV + 腐蚀膨胀 + 最小面积
  按 's' 保存到 hsv_params.yaml
  按 'q' 退出
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import yaml
import os


# 默认参数
DEFAULT_PARAMS = {
    'h_min': 0,  'h_max': 20,
    's_min': 152, 's_max': 255,
    'v_min': 172, 'v_max': 255,
    'erode_iter': 0,
    'dilate_iter': 0,
    'min_area': 1000,
}

# 保存路径
SAVE_DIR = os.path.dirname(os.path.realpath(__file__))
SAVE_PATH = os.path.join(SAVE_DIR, 'hsv_params.yaml')


def load_params():
    """加载已有参数文件"""
    if os.path.exists(SAVE_PATH):
        with open(SAVE_PATH, 'r') as f:
            data = yaml.safe_load(f)
            if data:
                return {**DEFAULT_PARAMS, **data}
    return DEFAULT_PARAMS.copy()


def save_params(params):
    """保存参数到 YAML"""
    os.makedirs(SAVE_DIR, exist_ok=True)
    with open(SAVE_PATH, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    print(f"\n参数已保存到: {SAVE_PATH}")
    print(f"  HSV: H({params['h_min']}-{params['h_max']}) "
          f"S({params['s_min']}-{params['s_max']}) "
          f"V({params['v_min']}-{params['v_max']})")
    print(f"  腐蚀: {params['erode_iter']}  膨胀: {params['dilate_iter']}  "
          f"最小面积: {params['min_area']} px")


class HsvCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.saved = False
        self._quit = False

        # 订阅 /image_raw 话题 (best_effort 与 opencv_camera_node 发布端匹配)
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))

        # 加载历史参数
        params = load_params()

        # 创建 GUI
        cv2.namedWindow("Parameters")
        cv2.resizeWindow("Parameters", 640, 380)

        cv2.createTrackbar("Hue Min",    "Parameters", params['h_min'],  179, self._empty)
        cv2.createTrackbar("Hue Max",    "Parameters", params['h_max'],  179, self._empty)
        cv2.createTrackbar("Sat Min",    "Parameters", params['s_min'],  255, self._empty)
        cv2.createTrackbar("Sat Max",    "Parameters", params['s_max'],  255, self._empty)
        cv2.createTrackbar("Val Min",    "Parameters", params['v_min'],  255, self._empty)
        cv2.createTrackbar("Val Max",    "Parameters", params['v_max'],  255, self._empty)
        cv2.createTrackbar("Erode Iter", "Parameters", params['erode_iter'], 10, self._empty)
        cv2.createTrackbar("Dilate Iter","Parameters", params['dilate_iter'], 10, self._empty)
        cv2.createTrackbar("Min Area",   "Parameters", params['min_area'], 20000, self._empty)

        self.get_logger().info(f'HSV 校准工具已启动，参数文件: {SAVE_PATH}')
        self.get_logger().info('按 s 保存，按 q 退出')

        # 定时器驱动 GUI 更新 (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self._update_gui)

    @staticmethod
    def _empty(_):
        pass

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')

    def _update_gui(self):
        if self.latest_frame is None:
            return

        if self._quit:
            cv2.destroyAllWindows()
            raise SystemExit

        frame = self.latest_frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("Hue Min", "Parameters")
        h_max = cv2.getTrackbarPos("Hue Max", "Parameters")
        s_min = cv2.getTrackbarPos("Sat Min", "Parameters")
        s_max = cv2.getTrackbarPos("Sat Max", "Parameters")
        v_min = cv2.getTrackbarPos("Val Min", "Parameters")
        v_max = cv2.getTrackbarPos("Val Max", "Parameters")
        erode_iter = cv2.getTrackbarPos("Erode Iter", "Parameters")
        dilate_iter = cv2.getTrackbarPos("Dilate Iter", "Parameters")
        min_area = cv2.getTrackbarPos("Min Area", "Parameters")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        if erode_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=erode_iter)
        if dilate_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=dilate_iter)

        # 轮廓检测 + 绘制
        display = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(display, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
                cv2.putText(display, f"Area:{area:.0f}", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 状态栏
        status = "SAVED" if self.saved else "Press 's' to save, 'q' to quit"
        color = (0, 255, 0) if self.saved else (0, 255, 255)
        cv2.rectangle(display, (0, 0), (display.shape[1], 30), (50, 50, 50), -1)
        cv2.putText(display, status, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        cv2.imshow("Mask", mask)
        cv2.imshow("Result", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            params = {
                'h_min': h_min, 'h_max': h_max,
                's_min': s_min, 's_max': s_max,
                'v_min': v_min, 'v_max': v_max,
                'erode_iter': erode_iter,
                'dilate_iter': dilate_iter,
                'min_area': min_area,
            }
            save_params(params)
            self.saved = True

        elif key == ord('q'):
            if not self.saved:
                params = {
                    'h_min': h_min, 'h_max': h_max,
                    's_min': s_min, 's_max': s_max,
                    'v_min': v_min, 'v_max': v_max,
                    'erode_iter': erode_iter,
                    'dilate_iter': dilate_iter,
                    'min_area': min_area,
                }
                save_params(params)
            self._quit = True


def main():
    rclpy.init()
    node = HsvCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("校准完成")


if __name__ == '__main__':
    main()