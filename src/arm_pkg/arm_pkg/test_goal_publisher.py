#!/usr/bin/env python3
"""
测试目标点发布器
"""

import rclpy
from rclpy.node import Node
from target_msg.msg import Goal


class TestGoalPublisher(Node):
    def __init__(self):
        super().__init__('test_goal_publisher')
        self.publisher = self.create_publisher(Goal, '/goal', 10)
        
        # 创建目标消息 - 使用更保守的目标点
        msg = Goal()
        msg.target_id = 1
        msg.x = 0.15  # 前方 15cm
        msg.y = 0.0
        msg.z = 0.20  # 高度 20cm
        msg.qx = 0.0
        msg.qy = 0.0
        msg.qz = 0.0
        msg.qw = 1.0
        
        # 发布
        self.publisher.publish(msg)
        self.get_logger().info(f'发布目标点: x={msg.x}, y={msg.y}, z={msg.z}')


def main():
    rclpy.init()
    node = TestGoalPublisher()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
