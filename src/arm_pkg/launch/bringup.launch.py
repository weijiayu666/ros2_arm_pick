"""
一键启动: 相机 + 机械臂 + 底盘 + 球检测 + 抓取 + 桥接
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. 相机 (OpenCV V4L2 后端 + MJPG, ~20fps, 发布 /image_raw bgr8)
        Node(
            package='vision_pkg', executable='opencv_camera_node',
            name='opencv_camera_node',
            parameters=[{
                'video_device': 0,
                'width': 640,
                'height': 480,
                'fps': 30,
            }],
        ),

        # 2. 机械臂串口驱动 (用 by-path 区分: 机械臂 ttyUSB0 = usb-0:1.1)
        Node(
            package='arm_pkg', executable='arm_joint_node',
            name='arm_joint_node',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0',
            }],
        ),

        # 3. 底盘串口驱动 (invert/swap 默认 true, 适配当前接线)
        # 底盘 BDC 接 ttyUSB1 (usb-0:1.3); 机械臂接 ttyUSB0 (usb-0:1.1)
        Node(
            package='chassis_driver', executable='chassis_driver_node',
            name='chassis_driver_node',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
            }],
        ),

        # 4. 球检测
        Node(
            package='vision_pkg', executable='ball_detector_node',
            name='ball_detector_node',
        ),

        # 5. 机械臂抓取 (含扫描)
        Node(
            package='arm_pkg', executable='arm_catch_node',
            name='arm_catch_node',
        ),

        # 6. 底盘桥接
        Node(
            package='chassis_driver', executable='chassis_arm_bridge',
            name='chassis_arm_bridge',
        ),

    ])