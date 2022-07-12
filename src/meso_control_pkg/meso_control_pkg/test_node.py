#!/usr/bin/env python3
import rclpy
import time
from rclpy.logging import get_logger
from rclpy.node import Node
from awi_interfaces.msg import AWIFloatValue, AWIStringValue
from awi_interfaces.srv import Modbus, Json
from functools import partial

import logging

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        logging.basicConfig(filename='log/meso_control.log', level=logging.DEBUG)
        logging.info("log test")

        self.timer_main_control_ =  self.create_timer(3, self.main_control)
        self.get_logger().info("Test node started")

    def main_control(self):
        self.get_logger().info("Sending wrong info to modbus")
        self.send_modbus_command("v5", 0)
        
    def send_modbus_command(self, json_modbus_key_name, new_status):
        # self.get_logger().info("sending " + json_modbus_key_name  +':'+ str(new_status))
        service_name = "/meso/modbus_tcp_node"
        client = self.create_client(Modbus, service_name)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service " + service_name)
    
        request = Modbus.Request()
        request.key_name = json_modbus_key_name
        request.value_to_send = str(new_status)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_send_modbus_command))

    def callback_send_modbus_command(self, future):
        try:
            response = future.result()
            #self.get_logger().info('modbus request: set ' + str(response.key_name) + ' to ' + str(response.value_to_send))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def callback_modbus_subscription(self, msg):
        self.json_obj_status_ = json.loads(msg.data)
        for k_modbus, v_modbus in self.json_obj_modbus_status_.items():
            for k, v in self.json_obj_status_.items():
                 if k == k_modbus: self.json_obj_modbus_status_[k] = v
        # self.log(str(self.json_obj_modbus_status_))
    
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
