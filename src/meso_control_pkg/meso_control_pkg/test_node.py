#!/usr/bin/env python3
import rclpy
import time
from rclpy.logging import get_logger
from rclpy.node import Node
from awi_interfaces.msg import AWIFloatValue


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.test_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v1_status", self.callback_test_subscription, 10)
        
        self.get_logger().info("Test node started")

    def callback_test_subscription(self, msg):
        self.get_logger().info("Test node:" +  str(msg.data))
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
