#!/usr/bin/env python3
from operator import truediv
from unittest.util import _count_diff_hashable
import rclpy
import json
from datetime import datetime
from rclpy.logging import get_logger
from rclpy.node import Node
from awi_interfaces.msg import AWIFloatValue
from awi_interfaces.msg import AWIStringValue
from awi_interfaces.srv import Modbus, Json

from pymodbus.client.sync import ModbusTcpClient

class ModbusTcpNode(Node):
    def __init__(self):
        super().__init__("modbus_tcp_node")
        
        self.declare_parameter("name", "modbus_tcp_node")
        self.declare_parameter("heartbeat_interval", 5)
        self.declare_parameter("publish_interval", 1)
        self.declare_parameter("topic_name", "json_modbus_values")
        self.declare_parameter("read_modbus_interval", 1)
        self.declare_parameter("modbus_host_ip", '192.168.177.25')
        self.declare_parameter("modbus_host_port", '502')
        self.declare_parameter("modbus_service_name", 'modbus_tcp_node')
        self.declare_parameter("json_modbus_write_registers", '{"v1":"32000", "v2":"32001"}')
        self.declare_parameter("json_modbus_feedback_registers", '{"v1":"1", "v2":"2"}')
        
        self.name_ = self.get_parameter("name").value
        self.heartbeat_interval_ = self.get_parameter("heartbeat_interval").value
        self.publish_interval_ = self.get_parameter("publish_interval").value
        self.topic_name_ =  self.get_parameter("topic_name").value
        self.read_modbus_interval_ = self.get_parameter("read_modbus_interval").value
        self.modbus_host_ip_ = self.get_parameter("modbus_host_ip").value
        self.modbus_service_name_ = self.get_parameter("modbus_service_name").value
        self.modbus_host_port_ = self.get_parameter("modbus_host_port").value

        self.json_obj_modbus_write_registers_ = json.loads(self.get_parameter("json_modbus_write_registers").value)
        self.json_obj_modbus_feedback_registers_ = json.loads(self.get_parameter("json_modbus_feedback_registers").value)
        self.log(json.dumps(self.json_obj_modbus_feedback_registers_))
        self.json_data_modbus_values_ = {}

        self.status_publisher_ = self.create_publisher(AWIStringValue, self.topic_name_, 10)

        self.service_register_ = self.create_service(Modbus, self.modbus_service_name_, self.callback_service)
        self.modbus_client_ = ModbusTcpClient(host=self.modbus_host_ip_ ,port=self.modbus_host_port_)

        self.timer_heartbeat_ = self.create_timer(self.heartbeat_interval_, self.heartbeat) 
        self.timer_publish_ = self.create_timer(self.publish_interval_, self.publish_status)     
        self.timer_read_modbus_ = self.create_timer(self.read_modbus_interval_, self.read_modbus)
        self.timer_prepare_modbus_ = self.create_timer(1, self.prepare_modbus_connection)

        self.timer_publish_.cancel()
        self.timer_read_modbus_.cancel()
                
        self.log("ModbusTcp node started")
    
    def heartbeat(self):
        register = self.get_register('heartbeat')
        self.write_modbus(register, 1)
        #self.log('heartbeat: ' + str(register))

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
        try:
            data = {}

            for k,v in self.json_obj_modbus_feedback_registers_.items():
                if self.modbus_client_.is_socket_open():
                    try:
                        rr = self.modbus_client_.read_holding_registers(int(v), 1, unit=1)
                        data[k] = rr.registers[0]
                        # self.log('K:' + str(k) + ' R:' + str(v) + ' V:' + str(data[k]))
                    except:
                        self.log("Can't read modbus register - connection broken?")
                else:
                    self.timer_publish_.cancel()
                    self.timer_read_modbus_.cancel()
                    self.timer_prepare_modbus_.reset()
                    break
            self.json_data_modbus_values_ = json.dumps(data)
        except:
            self.log("Can't read from modbus")
    
    def write_modbus(self, register, value):
        try:
            if self.modbus_client_.is_socket_open():
                #self.log("Writing to modbus: " + str(register) + ':' + str(value))
                self.modbus_client_.write_register(address=register, value=value)
        except:
            self.log("Can't write to modbus")
        
    def prepare_modbus_connection(self):
        try: 
            self.log("Preparing modbus connection")
            connection = self.modbus_client_.connect()
            if self.modbus_client_.is_socket_open():
                self.timer_read_modbus_.reset()
                self.timer_publish_.reset()
                self.timer_prepare_modbus_.cancel()
                self.log("Modbus tcp connected")
            else:
                self.log("modbus tcp connection: " + str(self.modbus_client_))
                self.log("Waiting for modbus tcp connection")
        except:
            self.log("Modbus preparation failed.")

    
    def get_register(self, key_name):
        register = 0
        for k,v in self.json_obj_modbus_write_registers_.items():
            if key_name == k:
                register = int(v)
                break
        
        return register
    
    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModbusTcpNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()