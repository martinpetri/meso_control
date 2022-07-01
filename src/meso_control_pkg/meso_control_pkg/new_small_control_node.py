#!/usr/bin/env python3
from ast import Return
import rclpy
import time
import json
from rclpy.logging import get_logger
from rclpy.node import Node
from datetime import datetime

from awi_interfaces.msg import AWIFloatValue, AWIStringValue
from awi_interfaces.srv import ActuatorControl, Modbus, Json
from functools import partial

STATUS_STOP = json.loads('{"v1":0, "v2":0, "v3":0, "v4":0, "v5":0, "v6":0, "v7":0, "p1":0}')
STATUS_PREP_TANK_A = json.loads('{"v1":1, "v2":1, "v3":0, "v4":0, "v5":0, "v6":0, "v7":0, "p1":0}')
STATUS_START_TANK_A = json.loads('{"v1":1, "v2":1, "v3":0, "v4":0, "v5":0, "v6":0, "v7":0, "p1":1}')
STATUS_PREP_FLUSH = json.loads('{"v1":0, "v2":0, "v3":0, "v4":0, "v5":1, "v6":1, "v7":0, "p1":0}')
STATUS_START_FLUSH = json.loads('{"v1":0, "v2":0, "v3":0, "v4":0, "v5":1, "v6":1, "v7":0, "p1":0}')
STATUS_PREP_TANK_B = json.loads('{"v1":0, "v2":0, "v3":1, "v4":1, "v5":0, "v6":0, "v7":0, "p1":1}')
STATUS_START_TANK_B = json.loads('{"v1":0, "v2":0, "v3":1, "v4":1, "v5":0, "v6":0, "v7":0, "p1":1}')

class NewSmallControlNode(Node):
    def __init__(self):
        super().__init__("new_small_control_node")

        self.declare_parameter("name", "control_node")
        self.declare_parameter("temp_setpoint", 12)
        self.declare_parameter("temp_hysteresis", 1.0)
        self.declare_parameter("temp_check_interval", 2)
        self.declare_parameter("tank_switch_interval", 30)
        self.declare_parameter("flush_duration", 10)
        self.declare_parameter("topic_modbus_values", "json_modbus_values")
        self.declare_parameter("modbus_service_register_name", "modbus_service_register")
        self.declare_parameter("modbus_service_json_name", "modbus_service_json")
     
        self.name_ = self.get_parameter("name").value
        self.temp_setpoint_ = self.get_parameter("temp_setpoint").value    
        self.temp_hysteresis_ = self.get_parameter("temp_hysteresis").value
        self.temp_check_interval_ = self.get_parameter("temp_check_interval").value
        self.tank_switch_interval_ = self.get_parameter("tank_switch_interval").value
        self.flush_duration_ = self.get_parameter("flush_duration").value
        self.topic_modbus_values_ = self.get_parameter("topic_modbus_values").value
        self.modbus_service_name_ = self.get_parameter("modbus_service_json_name").value

        self.json_obj_modbus_status_ = json.loads('{}')
        self.json_obj_target_status_ = STATUS_STOP

        self.timer_main_control_ =  self.create_timer(1, self.main_control)
        self.timer_temp_control_ =  self.create_timer(self.tank_switch_interval_, self.temp_control)
        self.timer_stop_flushing_ = self.create_timer(self.flush_duration_, self.stop_flushing)
        
        self.timer_temp_control_.cancel()
        self.timer_stop_flushing_.cancel()

        self.modbus_subscriber_ = self.create_subscription(
            AWIStringValue, self.topic_modbus_values_,
            self.callback_modbus_subscription, 10
        )
               
        self.log("Node " + self.name_ + " has been started")

        self.status_list_ = []
        self.fill_status_list()

        self.counter = 0
            
    def main_control(self):
        self.log("main_control")
        if not self.json_obj_target_status_ == self.json_obj_modbus_status_:
            json_string = json.dumps(self.json_obj_target_status_)
            self.send_modbus_command(json_string)
        else:
            current_status = self.json_obj_target_status_
            self.log(str(current_status))
        
            if ( current_status == STATUS_PREP_TANK_A or 
                 current_status == STATUS_PREP_FLUSH or
                 current_status == STATUS_PREP_TANK_B or
                 current_status == STATUS_STOP): 
                self.log("Node " + self.name_ + " has been started")
                self.set_next_target_status
        
            if (current_status == STATUS_START_TANK_A or current_status == STATUS_START_TANK_B):
                self.timer_main_control.cancel()
                self.timer_temp_control.reset()
        
            if (current_status == STATUS_START_FLUSH):
                self.timer_main_control.cancel()
                self.timer_stop_flush.reset()


    def temp_control(self):
        self.set_next_target_status()
        self.timer_main_control_.reset()
        self.timer_temp_control_.cancel()

    def stop_flushing(self):
        self.set_next_target_status()
        self.timer_main_control_.reset()
        self.timer_stop_flushing_.cancel()
    
    def log_status(self):
        self.log(str(self.json_obj_modbus_status_))


    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg
        self.get_logger().info(msg)
    
    def send_modbus_command(self, json_string):
            
            service_name = self.modbus_service_name_
            client = self.create_client(Json, service_name)
            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for service " + service_name)
        
            request = Json.Request()
            request.json = json_string
            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_send_modbus_command))
            
        
    def callback_send_modbus_command(self, future):
        try:
            response = future.result()
            # self.get_logger().info('modbus request: set ' + str(response.key_name) + ' to ' + str(response.value_to_send))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def callback_modbus_subscription(self, msg):
        self.json_obj_modbus_status_ = json.loads(msg.data)

    def set_next_target_status(self):
        counter = 0
        for s in self.status_list_:
            if self.status_list_[s] == self.json_obj_target_status_:
                if counter + 1 == len(self.status_list_):
                    self.json_obj_target_status_ = self.status_list_[0]
                else:
                    self.json_obj_target_status_ = self.status_list_[counter + 1]
        
        #self.log('next_target_status: ' + str(self.json_obj_target_status_))

    def fill_status_list(self):
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_TANK_A)
        self.status_list_.append(STATUS_START_TANK_A)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_FLUSH)
        self.status_list_.append(STATUS_START_FLUSH)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_TANK_B)
        self.status_list_.append(STATUS_START_TANK_B)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_FLUSH)
        self.status_list_.append(STATUS_START_FLUSH)

def main(args=None):
    rclpy.init(args=args)
    node = NewSmallControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
