import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraNativeNode(Node):
    def __init__(self):
        super().__init__('camera_native_node')
        
        # 创建 CvBridge 实例
        self.bridge = CvBridge()
        
        self.has_display = os.environ.get('DISPLAY') is not None
        if not self.has_display:
            self.get_logger().info('No display detected, GUI disabled')
        
        # 订阅摄像头图像话题
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 默认话题名，可根据实际情况修改
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera native node has been started.')
        self.get_logger().info('Subscribed to /image_raw topic.')
    
    def image_callback(self, msg):
        if not self.has_display:
            return
        try:
            # 将 ROS Image 消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 使用 OpenCV 显示图像
            cv2.imshow('Camera View', cv_image)
            
            # 等待按键，30ms 超时，保持窗口响应
            if cv2.waitKey(30) & 0xFF == ord('q'):
                self.get_logger().info('Quit signal received.')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main():
    rclpy.init()
    node = CameraNativeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
