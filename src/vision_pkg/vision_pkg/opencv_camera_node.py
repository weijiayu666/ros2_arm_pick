"""
OpenCV 相机驱动节点 — 用 V4L2 后端直接读相机, MJPG 格式提帧率, 发布 /image_raw

为什么不用 v4l2_camera_node:
  - v4l2_camera 不支持 MJPG, 且 YUYV->rgb8 软转换在树莓派上只有 ~6fps
  - OpenCV CAP_V4L2 后端 + MJPG 实测 ~20fps
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')

        self.bridge = CvBridge()
        # 图像流用 best_effort + keep_last(1): 丢帧无所谓, 避免 reliable 流控反向阻塞发布端
        self._img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Image, '/image_raw', self._img_qos)

        # 参数
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        dev = self.get_parameter('video_device').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # 强制 V4L2 后端 (默认 GStreamer 后端打开 MJPG 会失败)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开相机 /dev/video{dev}')
            raise RuntimeError('camera open failed')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.get_logger().info(
            f'OpenCV 相机启动: MJPG {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}@{int(self.cap.get(cv2.CAP_PROP_FPS))}'
        )

        # 独立线程读帧 + 发布 (达到相机真实帧率)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        last = time.time()
        cnt = 0
        while self._running and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('读帧失败')
                time.sleep(0.05)
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.pub.publish(msg)

            # 每 5 秒打一次帧率
            cnt += 1
            now = time.time()
            if now - last >= 5.0:
                self.get_logger().info(f'相机帧率: {cnt / (now - last):.1f} fps')
                last = now
                cnt = 0

    def destroy_node(self):
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenCVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
