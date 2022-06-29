#!/usr/bin/env python3
from telnetlib import theNULL
import rclpy
import requests
import math
import json

from rclpy.logging import get_logger
from rclpy.node import Node

from awi_interfaces.msg import AWIFloatValue, AWIStringValue
from awi_interfaces.srv import ActuatorControl, Modbus
from functools import partial

class ActuatorNode(Node):
    def __init__(self):
        super().__init__("actuator_node")
        
        self.declare_parameter("sps_api_url", "http://192.168.23.100/awp/api.html")
        self.declare_parameter("name", "actuator_node")
        self.declare_parameter("max_status", 60)
        self.declare_parameter("min_status", 0)
        self.declare_parameter("poll_interval", 1)
        self.declare_parameter("publish_interval", 1)
        self.declare_parameter("modbus_register_key", "actuator_1")
        self.declare_parameter("topic_modbus_values", "json_modbus_values")
        self.declare_parameter("modbus_service_name", "modbus_tcp_node")

        self.sps_api_url_ = self.get_parameter("sps_api_url").value
        self.name_ = self.get_parameter("name").value
        self.max_status_= self.get_parameter("max_status").value
        self.min_status_= self.get_parameter("min_status").value
        self.poll_interval_ = self.get_parameter("poll_interval").value
        self.publish_interval_ = self.get_parameter("publish_interval").value
        self.modbus_register_key_ = self.get_parameter("modbus_register_key").value
        self.topic_modbus_values_  = self.get_parameter("topic_modbus_values").value
        self.modbus_service_name_  = self.get_parameter("modbus_service_name").value

        self.status_ = 0.0
        self.topic_name_ = "/meso/" + self.name_ + "_status"

        self.modbus_subscriber_ = self.create_subscription(AWIStringValue, self.topic_modbus_values_, self.callback_modbus_subscription, 10)

        self.status_publisher_ = self.create_publisher(AWIFloatValue, self.topic_name_, 10)
        self.service_ = self.create_service(ActuatorControl, self.name_ + "_control", self.callback_control)
        self.timer1_ = self.create_timer(self.publish_interval_, self.publish_status)
        #self.timer2_ = self.create_timer(self.poll_interval_, self.poll_status)
        
        self.get_logger().info(self.name_ + " node has been started.")
        self.poll_status()

    def publish_status(self):
        msg = AWIFloatValue()
        msg.topic_name = self.topic_name_ 
        msg.data = self.status_
        msg.unit = 'deg'
        self.status_publisher_.publish(msg)
    
    def poll_status(self):
        self.status_ = self.status_
        # try:
        #     response = requests.get(self.sps_api_url_, verify = False, timeout=5)
        #     data = response.json()
        #     self.status_ = round(float(data[self.valve_name_]),1)    
        # except Exception as e:
        #     self.status_ = 0.0
        #     self.get_logger().error("Failed to get valve_status %r" % (e,))

    def callback_modbus_subscription(self, msg):
        json_obj = json.loads(msg.data)
        for r in json_obj:
            if r == self.modbus_register_key_: self.status_ = float(json_obj[r])

    def callback_control(self, request, response):
        new_status = float(f'{request.new_status:.1f}')
        response.requested_status = new_status
        
        if new_status > self.max_status_: new_status = self.max_status_
        if new_status < self.min_status_: new_status = self.min_status_

        
        if new_status != self.status_: self.move(new_status)
        return response

    #def move(self, new_status):
        
        #self.status_ = float(f'{new_status:.1f}')
        
        # try:
        #     headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        #     request_body = '"testValve".' + str(self.valve_name_) + '=' + str(new_status)
        #     response = requests.post(self.sps_api_url_, data=request_body, headers=headers, verify = False)
        #     data = response.json()
        #     self.get_logger().info("status: " + str(self.status_))
        # except Exception as e:
        #     self.status_ = 0.0
        #     self.get_logger().error("Failed to move valve %r" % (e,))
    
    def move(self, new_status):
        service_name = self.modbus_service_name_
        client = self.create_client(Modbus, service_name)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service " + service_name)
    
        request = Modbus.Request()
        request.key_name = self.modbus_register_key_
        request.value_to_send = str(new_status)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_move))
        
    def callback_move(self, future):
        try:
            response = future.result()
            # self.get_logger().info('modbus request: set ' + str(response.key_name) + ' to ' + str(response.value_to_send))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
