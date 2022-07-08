#!/usr/bin/env python3
from ast import Return
import rclpy
import json
from rclpy.logging import get_logger
from rclpy.node import Node
from datetime import datetime
import logging

from awi_interfaces.msg import AWIStringValue
from awi_interfaces.srv import Modbus
from functools import partial

CONST_OPEN = 1
CONST_CLOSED = 0
CONST_ON = 1
CONST_OFF = 0

class SmallControlNode(Node):
    def __init__(self):
        super().__init__("small_control_node")

        self.declare_parameter("name", "control_node")
        self.declare_parameter("temp_setpoint_A", 12)
        self.declare_parameter("temp_hysteresis_A", 0.1)
        self.declare_parameter("temp_setpoint_B", 12)
        self.declare_parameter("temp_hysteresis_B", 0.1)
        self.declare_parameter("temp_check_interval", 5)
        self.declare_parameter("tank_switch_interval", 30)
        self.declare_parameter("flush_duration", 10)
        self.declare_parameter("topic_modbus_values", "json_modbus_values")
        self.declare_parameter("modbus_service_name", "modbus_tcp_node")

        self.last_tank_ = 'A'
        self.next_status_ = 'A'

        self.name_ = self.get_parameter("name").value
        self.temp_setpoint_A_ = float(self.get_parameter("temp_setpoint_A").value)
        self.temp_hysteresis_A_ = float(self.get_parameter("temp_hysteresis_A").value)
        self.temp_setpoint_B_ = float(self.get_parameter("temp_setpoint_B").value)
        self.temp_hysteresis_B_ = float(self.get_parameter("temp_hysteresis_B").value)
        self.temp_check_interval_ = self.get_parameter("temp_check_interval").value
        self.tank_switch_interval_ = self.get_parameter("tank_switch_interval").value
        self.flush_duration_ = self.get_parameter("flush_duration").value
        self.topic_modbus_values_ = self.get_parameter("topic_modbus_values").value
        self.modbus_service_name_ = self.get_parameter("modbus_service_name").value

        self.json_obj_modbus_status_ = json.loads('{}')
        self.json_obj_target_status_ = json.loads('{"v1":0, "v2":0, "v3":0, "v4":0, "v5":0, "v6":0, "v7":0, "p1":0}')
        
        self.timer_check_system_stopped_ =  self.create_timer(1, self.check_system_stopped)
        self.timer_check_tank_prepared_ =  self.create_timer(1, self.check_tank_prepared)
        self.timer_check_pump_started_ =  self.create_timer(1, self.check_pump_started)
        self.timer_check_flushing_started_ =  self.create_timer(1, self.check_flushing_started)
        self.timer_switch_tank_ = self.create_timer(self.tank_switch_interval_, self.switch_tank)
        self.timer_control_temp_ = self.create_timer(self.temp_check_interval_, self.control_temp)
        self.timer_stop_flushing_ = self.create_timer(self.flush_duration_, self.stop_flushing)

        self.timer_check_tank_prepared_.cancel()
        self.timer_check_pump_started_.cancel()
        self.timer_check_flushing_started_.cancel()
        self.timer_switch_tank_.cancel()
        self.timer_control_temp_.cancel()
        self.timer_stop_flushing_.cancel()

        self.modbus_subscriber_ = self.create_subscription(
            AWIStringValue, self.topic_modbus_values_,
            self.callback_modbus_subscription, 10
        )
        
        self.stop_system()
        self.log("Node " + self.name_ + " has been started")

        self.counter = 0

    def control_temp(self):

        if not (self.tank_started(self.last_tank_) and self.pump_started()):
            self.timer_control_temp_.cancel()
            self.timer_switch_tank_.cancel()
            self.timer_check_tank_prepared_.reset()

        if self.last_tank_ == 'A':
            temperature = int(float(f'{self.temp_setpoint_A_:.2f}')*100)
            hysteresis = int(float(f'{self.temp_hysteresis_A_:.2f}')*100)
            self.set_control_value('temp_setpoint_A', temperature)
            self.set_control_value('temp_hysteresis_A', hysteresis)
            self.set_control_value('temp_control_A', CONST_ON)
            self.log("temp_setpoint_A:" +  str(temperature))

        if self.last_tank_ == 'B':
            temperature = int(float(f'{self.temp_setpoint_B_:.2f}')*100)
            hysteresis = int(float(f'{self.temp_hysteresis_B_:.2f}')*100)
            self.set_control_value('temp_setpoint_B', temperature)
            self.set_control_value('temp_hysteresis_B', hysteresis)
            self.set_control_value('temp_control_B', CONST_ON)
            self.log("temp_setpoint_B:" +  str(temperature))
        
    def check_system_stopped(self):
        self.counter += 1
        if self.system_stopped():
            
            self.timer_check_system_stopped_.cancel()
            self.log("System stopped")
            self.log_status()

            self.counter = 0
            if (self.next_status_ == 'A' or self.next_status_ == 'B'):
                self.start_tank(self.next_status_)
                self.timer_check_tank_prepared_.reset()

            if self.next_status_ == 'flush':
                self.start_flushing()
                self.timer_check_flushing_started_.reset()
        else:
            self.log("Waiting for system to be stopped")
            if self.counter >= 5:
                self.stop_system()
    
    def check_tank_prepared(self):
        self.counter += 1
        
        if not self.tank_started(self.next_status_):
            if self.counter >= 5:
                self.log("Waiting for tank " + self.next_status_ + " to be prepared")
                self.counter = 0
                self.start_tank(self.next_status_)
        else:
            self.log("Tank " + self.next_status_ + " prepared")
            self.log_status()
            self.start_pump()
            self.timer_check_tank_prepared_.cancel()
            self.timer_check_pump_started_.reset()
    
    def check_pump_started(self):
        self.counter += 1
        
        if not self.pump_started():
            if self.counter >= 5:
                self.log("Waiting for pump to be started")
                self.counter = 0
                self.start_pump()
        else:
            self.timer_check_pump_started_.cancel()
            self.log("Pump started")
            self.log_status()
            self.timer_control_temp_.reset()
            self.timer_switch_tank_.reset()
        
    def check_flushing_started(self):
        self.counter += 1
        if not self.flushing_started():
            if self.counter >= 5:
                self.log("Waiting for flushing to be started")
                self.counter = 0
                self.start_flushing()
        else:
            self.timer_check_flushing_started_.cancel()
            self.log("Flushing started")
            self.log_status()
            self.timer_stop_flushing_.reset()
   
    def start_tank(self, tank):
        self.last_tank_ = tank
        if tank == 'A':
            self.set_control_value('v1', CONST_OPEN)
            self.set_control_value('v2', CONST_OPEN)

        if tank == 'B':
            self.set_control_value('v3', CONST_OPEN)
            self.set_control_value('v4', CONST_OPEN)
    
    def start_pump(self):
        self.set_control_value('p1', CONST_ON)

    def switch_tank(self):
        self.timer_control_temp_.cancel()
        self.timer_switch_tank_.cancel()
        self.next_status_ = 'flush'
        self.stop_system()

    def start_flushing(self):
        self.set_control_value('v5', CONST_OPEN)
        self.set_control_value('v6', CONST_OPEN)

    def stop_flushing(self):
        self.timer_stop_flushing_.cancel()
        if self.last_tank_ == 'A': self.next_status_ = 'B'
        if self.last_tank_ == 'B': self.next_status_ = 'A'
        self.stop_system()

    def system_stopped(self):
        try:
            if (self.json_obj_modbus_status_['v1'] == CONST_CLOSED and
                self.json_obj_modbus_status_['v2'] == CONST_CLOSED and
                self.json_obj_modbus_status_['v3'] == CONST_CLOSED and 
                self.json_obj_modbus_status_['v4'] == CONST_CLOSED and
                self.json_obj_modbus_status_['v5'] == CONST_CLOSED and
                self.json_obj_modbus_status_['v6'] == CONST_CLOSED and
                self.json_obj_modbus_status_['temp_control_A'] == CONST_OFF and
                self.json_obj_modbus_status_['temp_control_B'] == CONST_OFF and
                self.json_obj_modbus_status_['p1'] == CONST_OFF):
                return True
            else:
                return False
        except:
            return False
    
    def tank_started(self, tank):
        try:
            if tank == 'A':
                if (self.json_obj_modbus_status_['v1'] == CONST_OPEN and self.json_obj_modbus_status_['v2'] == CONST_OPEN): return True
            elif tank == 'B':
                if (self.json_obj_modbus_status_['v3'] == CONST_OPEN and self.json_obj_modbus_status_['v4'] == CONST_OPEN): return True
            else: return False
        except:
            return False
    
    def pump_started(self):
        try:
            if self.json_obj_modbus_status_['p1'] == CONST_ON: return True
            else: return False
        except:
            return False

    def flushing_started(self):
        try:
            if (self.json_obj_modbus_status_['v5'] == CONST_OPEN and self.json_obj_modbus_status_['v6'] == CONST_OPEN): return True
            else: return False
        except:
            return False
    
    def log_status(self):
        self.log('STATUS: ' + str(self.json_obj_modbus_status_))

    def stop_system(self):
        try:
            self.set_control_value('p1', CONST_OFF)
            self.set_control_value('v1', CONST_CLOSED)
            self.set_control_value('v2', CONST_CLOSED)
            self.set_control_value('v3', CONST_CLOSED)
            self.set_control_value('v4', CONST_CLOSED)
            self.set_control_value('v5', CONST_CLOSED)
            self.set_control_value('v6', CONST_CLOSED)
            self.set_control_value('temp_control_A', CONST_OFF)
            self.set_control_value('temp_control_B', CONST_OFF)
            self.counter = 0
            self.timer_check_system_stopped_.reset()
        except:
            return False

    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg

        self.get_logger().info(msg)
        logging.info(msg)

    def set_control_value(self, json_modbus_key_name, new_status):
        service_name = self.modbus_service_name_
        client = self.create_client(Modbus, service_name)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service " + service_name)
    
        request = Modbus.Request()
        request.key_name = json_modbus_key_name
        request.value_to_send = str(new_status)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_control_value))
        
    def callback_set_control_value(self, future):
        try:
            response = future.result()
            # self.get_logger().info('modbus request: set ' + str(response.key_name) + ' to ' + str(response.value_to_send))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def callback_modbus_subscription(self, msg):
        self.json_obj_modbus_status_ = json.loads(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SmallControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
