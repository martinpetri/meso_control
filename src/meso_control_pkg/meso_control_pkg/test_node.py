#!/usr/bin/env python3
import rclpy
import time
from rclpy.logging import get_logger
from rclpy.node import Node
from awi_interfaces.msg import AWIFloatValue
import logging

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        logging.basicConfig(filename='log/meso_control.log', level=logging.DEBUG)
        logging.info("log test")
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
