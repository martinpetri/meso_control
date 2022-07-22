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
STATUS_PREP_TANK_A = json.loads('{"v1":1, "v2":1, "v3":0, "v4":0, "v5":1, "v6":0, "v7":0, "p1":0}')
STATUS_VENT_TANK_A = json.loads('{"v1":1, "v2":1, "v3":0, "v4":0, "v5":1, "v6":0, "v7":0, "p1":1}')
STATUS_START_TANK_A = json.loads('{"v1":1, "v2":1, "v3":0, "v4":0, "v5":0, "v6":0, "v7":0, "p1":1}')
STATUS_START_EMPTY = json.loads('{"v1":0, "v2":0, "v3":0, "v4":0, "v5":1, "v6":1, "v7":0, "p1":0}')
STATUS_PREP_TANK_B = json.loads('{"v1":0, "v2":0, "v3":1, "v4":1, "v5":1, "v6":0, "v7":0, "p1":0}')
STATUS_VENT_TANK_B = json.loads('{"v1":0, "v2":0, "v3":1, "v4":1, "v5":1, "v6":0, "v7":0, "p1":1}')
STATUS_START_TANK_B = json.loads('{"v1":0, "v2":0, "v3":1, "v4":1, "v5":0, "v6":0, "v7":0, "p1":1}')

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.declare_parameter("name", "control_node")
        self.declare_parameter("temp_setpoint", 12)
        self.declare_parameter("temp_setpoint_A", 12)
        self.declare_parameter("temp_hysteresis_A", 0.1)
        self.declare_parameter("temp_setpoint_B", 12)
        self.declare_parameter("temp_hysteresis_B", 0.1)
        self.declare_parameter("temp_check_interval", 5)
        self.declare_parameter("tank_switch_interval", 30)
        self.declare_parameter("empty_duration", 10)
        self.declare_parameter("topic_modbus_values", "json_modbus_values")
        self.declare_parameter("modbus_service_register_name", "modbus_service_register")
        self.declare_parameter("modbus_service_name", "modbus_service")
     
        self.name_ = self.get_parameter("name").value

        self.temp_setpoint_A_ = float(self.get_parameter("temp_setpoint_A").value)
        self.temp_hysteresis_A_ = float(self.get_parameter("temp_hysteresis_A").value)
        self.temp_setpoint_B_ = float(self.get_parameter("temp_setpoint_B").value)
        self.temp_hysteresis_B_ = float(self.get_parameter("temp_hysteresis_B").value)
        
        self.temp_check_interval_ = self.get_parameter("temp_check_interval").value
        self.tank_switch_interval_ = self.get_parameter("tank_switch_interval").value
        self.empty_duration_ = self.get_parameter("empty_duration").value
        self.topic_modbus_values_ = self.get_parameter("topic_modbus_values").value
        self.modbus_service_name_ = self.get_parameter("modbus_service_name").value

        self.json_obj_status_ = json.loads('{}')
        self.json_obj_modbus_status_ = STATUS_STOP.copy()
        self.json_obj_target_status_ = STATUS_STOP.copy()

        self.timer_main_control_ =  self.create_timer(5, self.main_control)
        self.timer_tank_switch_ =  self.create_timer(self.tank_switch_interval_, self.tank_switch)
        self.timer_temp_control_ =  self.create_timer(self.temp_check_interval_, self.temp_control)
        self.timer_stop_emptying_ = self.create_timer(self.empty_duration_, self.stop_emptying)
        self.timer_status_watchdog_ = self.create_timer(2, self.status_watchdog)
        self.timer_unlock_modbus_ = self.create_timer(5, self.unlock_modbus)

        self.timer_tank_switch_.cancel()
        self.timer_temp_control_.cancel()
        self.timer_stop_emptying_.cancel()

        self.modbus_locked_ = False

        self.modbus_subscriber_ = self.create_subscription(
            AWIStringValue, self.topic_modbus_values_,
            self.callback_modbus_subscription, 10
        )

        self.client_modbus_ = self.create_client(Modbus, self.modbus_service_name_)

        self.log("Node " + self.name_ + " has been started")

        self.status_list_ = []
        self.target_status_index_ = -1
        self.fill_status_list()
    
    def main_control(self):

        if self.status_consistent():
            if (self.json_obj_modbus_status_ == STATUS_PREP_TANK_A or
                self.json_obj_modbus_status_ == STATUS_VENT_TANK_A or
                self.json_obj_modbus_status_ == STATUS_PREP_TANK_B or
                self.json_obj_modbus_status_ == STATUS_VENT_TANK_B or
                self.json_obj_modbus_status_ == STATUS_STOP):
                self.set_next_target_status()

            if (self.json_obj_modbus_status_ == STATUS_START_TANK_A or self.json_obj_modbus_status_ == STATUS_START_TANK_B):
                if self.json_obj_modbus_status_ == STATUS_START_TANK_A:
                    self.log("Tank A running")
                if self.json_obj_modbus_status_ == STATUS_START_TANK_B:
                    self.log("Tank B running")
                self.timer_main_control_.cancel()
                self.timer_tank_switch_.reset()
                self.timer_temp_control_.reset()
        
            if (self.json_obj_modbus_status_ == STATUS_START_EMPTY):
                self.log("Emptying system")
                self.timer_main_control_.cancel()
                self.timer_stop_emptying_.reset()

    def unlock_modbus(self):
        self.timer_unlock_modbus_.cancel()
        self.modbus_locked_ = False
        #self.log("Modbus unlocked")

    def correct_status(self, dict_values):
        json_string = json.dumps(dict_values)
        self.log_status("Current status", self.json_obj_modbus_status_)
        self.log_status("Sending target status to modbus", self.json_obj_target_status_)
        self.json_to_modbus(json_string)
        self.modbus_locked_ = True
        self.timer_unlock_modbus_.reset()
        #self.log("Modbus locked")

    def tank_switch(self):
        self.log("Switching tank")
        self.set_next_target_status()
        self.timer_temp_control_.cancel()
        self.timer_tank_switch_.cancel()
        self.timer_main_control_.reset()

    def status_watchdog(self):
        if not self.modbus_locked_:
            #self.log("status watchdog")
            if not self.status_consistent():
                dict_status_diff = self.get_status_diff()
                self.correct_status(dict_status_diff)
    
    def get_status_diff(self):
        dict_diff = {}
        for k,v in self.json_obj_target_status_.items():
            if not self.json_obj_modbus_status_[k] == v: dict_diff[k] = v

        return dict_diff

    def temp_control(self):

        # Normal temp control routine
        set_point_A = int(float(f'{self.temp_setpoint_A_:.2f}')*100)
        hysteresis_A = int(float(f'{self.temp_hysteresis_A_:.2f}')*100)
        
        set_point_B = int(float(f'{self.temp_setpoint_B_:.2f}')*100)
        hysteresis_B = int(float(f'{self.temp_hysteresis_B_:.2f}')*100)

        dict_temps = {}
        dict_temps['temp_setpoint_A'] = set_point_A
        dict_temps['temp_hysteresis_A'] = hysteresis_A
        dict_temps['temp_setpoint_B'] = set_point_B
        dict_temps['temp_hysteresis_B'] = hysteresis_B

        json_string = json.dumps(dict_temps)
        #self.log("Sending temp setpoints to modbus")
        self.log(str(dict_temps))
        self.json_to_modbus(json_string)
        self.modbus_locked_ = True
        self.timer_unlock_modbus_.reset()
        #self.log("Modbus locked")
    
    def stop_emptying(self):
        self.log("Waiting for emptying to stop")
        self.set_next_target_status()
        self.timer_main_control_.reset()
        self.timer_stop_emptying_.cancel()
    
    def log_status(self):
        self.log(str(self.json_obj_status_))
    
    def log_status(self, prefix = "", json_status = {}):
        if json_status == STATUS_STOP: status = "STOP"
        elif json_status == STATUS_PREP_TANK_A: status = "PREP_TANK_A"
        elif json_status == STATUS_START_TANK_A: status = "START_TANK_A"
        elif json_status == STATUS_VENT_TANK_A: status = "VENT_TANK_A"
        elif json_status == STATUS_PREP_TANK_B: status = "PREP_TANK_B"
        elif json_status == STATUS_START_TANK_B: status = "START_TANK_B"
        elif json_status == STATUS_VENT_TANK_B: status = "VENT_TANK_B"
        elif json_status == STATUS_START_EMPTY: status = "START_EMPTY"
        else:  status = "STATUS_UNKNOWN"

        self.log(prefix + ': ' + status)
    
    def status_consistent(self):
        if self.json_obj_target_status_ == self.json_obj_modbus_status_:
            return True
        else:
            return False

    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg
        self.get_logger().info(msg)
    
    def json_to_modbus(self, json_string):
        json_obj = json.loads(json_string)
        for k,v in json_obj.items():
            self.send_modbus_command(k, v)

    def send_modbus_command(self, json_modbus_key_name, new_status):
        # self.get_logger().info("sending " + json_modbus_key_name  +':'+ str(new_status))
        
        while not self.client_modbus_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service " + self.modbus_service_name_)
    
        request = Modbus.Request()
        request.key_name = json_modbus_key_name
        request.value_to_send = str(new_status)

        future = self.client_modbus_.call_async(request)
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

    def set_next_target_status(self):

        if self.target_status_index_ < len(self.status_list_) - 1:
            self.target_status_index_ += 1
        else:
            self.target_status_index_ = 0

        self.json_obj_target_status_ = self.status_list_[self.target_status_index_].copy()


        #self.log_status("Set target status to", self.json_obj_target_status_)

    def fill_status_list(self):
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_TANK_A)
        self.status_list_.append(STATUS_VENT_TANK_A)
        self.status_list_.append(STATUS_START_TANK_A)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_START_EMPTY)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_PREP_TANK_B)
        self.status_list_.append(STATUS_VENT_TANK_B)
        self.status_list_.append(STATUS_START_TANK_B)
        self.status_list_.append(STATUS_STOP)
        self.status_list_.append(STATUS_START_EMPTY)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
