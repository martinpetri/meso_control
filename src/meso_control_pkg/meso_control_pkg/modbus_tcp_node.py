#!/usr/bin/env python3
from operator import truediv
from unittest.util import _count_diff_hashable
import rclpy
import time
import json
from rclpy.logging import get_logger
from rclpy.node import Node
from awi_interfaces.msg import AWIFloatValue
from awi_interfaces.msg import AWIStringValue
from awi_interfaces.srv import Modbus

from pymodbus.client.sync import ModbusTcpClient

class ModbusTcpNode(Node):
    def __init__(self):
        super().__init__("modbus_tcp_node")
        
        self.declare_parameter("name", "modbus_tcp_node")
        self.declare_parameter("publish_interval", 1)
        self.declare_parameter("topic_name", "json_modbus_values")
        self.declare_parameter("read_modbus_interval", 1)
        self.declare_parameter("modbus_host_ip", '192.168.178.51')
        self.declare_parameter("modbus_host_port", '502')
        self.declare_parameter("modbus_service_name", 'modbus_tcp_node')
        self.declare_parameter("json_modbus_registers", '{"v1":"32000", "v2":"32001"}')
        
        self.name_ = self.get_parameter("name").value
        self.publish_interval_ = self.get_parameter("publish_interval").value
        self.topic_name_ =  self.get_parameter("topic_name").value
        self.read_modbus_interval_ = self.get_parameter("read_modbus_interval").value
        self.modbus_host_ip_ = self.get_parameter("modbus_host_ip").value
        self.modbus_service_name_ = self.get_parameter("modbus_service_name").value
        self.modbus_host_port_ = self.get_parameter("modbus_host_port").value

        json_string = json.dumps(self.get_parameter("json_modbus_registers").value)
        self.json_obj_modbus_registers_ = json.loads(self.get_parameter("json_modbus_registers").value)
        self.json_data_modbus_values_ = {}

        self.status_publisher_ = self.create_publisher(AWIStringValue, self.topic_name_, 10)

        self.service_ = self.create_service(Modbus, self.modbus_service_name_, self.callback_service)
        self.modbus_client_ = ModbusTcpClient(host=self.modbus_host_ip_ ,port=self.modbus_host_port_)

        self.timer_publish_ = self.create_timer(self.publish_interval_, self.publish_status)     
        self.timer_read_modbus_ = self.create_timer(self.read_modbus_interval_, self.read_modbus)
        self.timer_prepare_modbus_ = self.create_timer(1, self.prepare_modbus_connection)

        self.timer_publish_.cancel()
        self.timer_read_modbus_.cancel()
                
        self.get_logger().info("ModbusTcp node started")
    
    def callback_service(self, request, response):
        
        if not request.register_address:
            register = self.get_register(request.key_name)
        else:
            register = int(float(request.register_address))
        
        value = int(float(request.value_to_send))
        self.write_modbus(register, value)
        
        response.key_name = request.key_name
        response.register_address = str(register)
        response.value_to_send = str(value)
        
        return response
    
    def publish_status(self):
        msg = AWIStringValue()
        msg.topic_name = self.topic_name_ 
        msg.data = str(self.json_data_modbus_values_)
        msg.unit = 'json'
        self.status_publisher_.publish(msg)
        
    def read_modbus(self):
        data = {}
        for r in self.json_obj_modbus_registers_:
            if self.modbus_client_.is_socket_open():
                try:
                    rr = self.modbus_client_.read_holding_registers(int(self.json_obj_modbus_registers_[r]), 1, unit=1)
                    data[r] = rr.registers[0]
                except:
                    self.get_logger().info("Can't read modbus register - connection broken?")
            else:
                self.timer_publish_.cancel()
                self.timer_read_modbus_.cancel()
                self.timer_prepare_modbus_.reset()
                break

        self.json_data_modbus_values_ = json.dumps(data)
    
    def write_modbus(self, register, value):
        if self.modbus_client_.is_socket_open():
            # self.get_logger().info("Writing to modbus: " + str(register) + ':' + str(value))
            try:
                self.modbus_client_.write_register(address=register, value=value)
            except:
                self.get_logger().info("Can't write to modbus - connection broken?")
        
    def prepare_modbus_connection(self):
        connection = self.modbus_client_.connect()
        if self.modbus_client_.is_socket_open():
            self.timer_read_modbus_.reset()
            self.timer_publish_.reset()
            self.timer_prepare_modbus_.cancel()
            self.get_logger().info("Modbus tcp connected")
        else:
            self.get_logger().info("modbus tcp connection: " + str(self.modbus_client_))
            self.get_logger().info("Waiting for modbus tcp connection")
    
    def get_register(self, key_name):
        register = 0
        for r in self.json_obj_modbus_registers_:
            if key_name == r:
                register = int(self.json_obj_modbus_registers_[r])
                break
        
        return register

def main(args=None):
    rclpy.init(args=args)
    node = ModbusTcpNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()