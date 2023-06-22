#!/usr/bin/env python3
import rclpy
from datetime import datetime
from rclpy.logging import get_logger
from rclpy.node import Node
from osbk_interfaces.msg import OSBKFloatValue, OSBKStringValue
from osbk_interfaces.srv import Modbus, Json
from functools import partial

import logging

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.timer_main_control_ =  self.create_timer(1, self.main_control)
        self.timer_one_ =  self.create_timer(1, self.timer_one)
        self.timer_one_.cancel()
        self.counter_ = 0

        self.get_logger().info("Test node started")

    def main_control(self):
        self.counter_ += 1
        if self.timer_one_.is_canceled():
            self.timer_one_ =  self.create_timer(self.counter_, self.timer_one)
            self.log("resetting timer")   
        else:
         self.log("timer is running")
            
    def timer_one(self):
        self.log("stop")
        self.timer_one_.cancel()
        self.timer_one_.destroy()
        
    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg
        self.get_logger().info(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
