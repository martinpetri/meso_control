#!/usr/bin/env python3
from ast import Return
import rclpy
import time
from rclpy.logging import get_logger
from rclpy.node import Node
from datetime import datetime

from awi_interfaces.msg import AWIFloatValue
from awi_interfaces.srv import ActuatorControl
from functools import partial

CONST_OPEN = 1
CONST_CLOSED = 0
CONST_ON = 1
CONST_OFF = 0

class BigControlNode(Node):
    def __init__(self):
        super().__init__("big_control_node")

        self.declare_parameter("name", "control_node")
        self.declare_parameter("temp_setpoint", 12)
        self.declare_parameter("temp_hysteresis", 1.0)
        self.declare_parameter("temp_check_interval", 2)
        self.declare_parameter("tank_switch_interval", 30)
        self.declare_parameter("flush_duration", 10)
        
        self.temp_ = 0
        self.v1_ = CONST_OPEN
        self.v2_ = CONST_OPEN
        self.v3_ = CONST_OPEN
        self.v4_ = CONST_OPEN
        self.v5_ = CONST_OPEN
        self.v6_ = CONST_OPEN
        self.v7_ = CONST_OPEN
        self.p1_ = CONST_ON
        
        self.last_tank_ = 'A'
        self.next_status_ = 'A'

        self.v1_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v1_status", self.callback_v1, 10)
        self.v2_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v2_status", self.callback_v2, 10)
        self.v3_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v3_status", self.callback_v3, 10)
        self.v4_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v4_status", self.callback_v4, 10)
        self.v5_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v5_status", self.callback_v5, 10)
        self.v6_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v6_status", self.callback_v6, 10)
        self.v7_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/v7_status", self.callback_v7, 10)
        self.p1_subscriber_ = self.create_subscription(AWIFloatValue, "/meso/p1_status", self.callback_p1, 10)

        self.name_ = self.get_parameter("name").value
        self.temp_setpoint_ = self.get_parameter("temp_setpoint").value    
        self.temp_hysteresis_ = self.get_parameter("temp_hysteresis").value
        self.temp_check_interval_ = self.get_parameter("temp_check_interval").value
        self.tank_switch_interval_ = self.get_parameter("tank_switch_interval").value
        self.flush_duration_ = self.get_parameter("flush_duration").value

        self.timer_check_system_stopped_ =  self.create_timer(1, self.check_system_stopped)
        self.timer_check_tank_started_ =  self.create_timer(1, self.check_tank_started)
        self.timer_check_flushing_started_ =  self.create_timer(1, self.check_flushing_started)
        self.timer_switch_tank_ = self.create_timer(self.tank_switch_interval_, self.switch_tank)
        self.timer_control_temp_ = self.create_timer(self.temp_check_interval_, self.control_temp)
        self.timer_stop_flushing_ = self.create_timer(self.flush_duration_, self.stop_flushing)
        
        self.timer_check_tank_started_.cancel()
        self.timer_check_flushing_started_.cancel()
        self.timer_switch_tank_.cancel()
        self.timer_control_temp_.cancel()
        self.timer_stop_flushing_.cancel()
        
        self.stop_system()
        self.log("Node " + self.name_ + " has been started")

        self.counter = 0
    
    def callback_v1(self, msg): self.v1_ = int(msg.data)
    def callback_v2(self, msg): self.v2_ = int(msg.data)
    def callback_v3(self, msg): self.v3_ = int(msg.data)
    def callback_v4(self, msg): self.v4_ = int(msg.data)
    def callback_v5(self, msg): self.v5_ = int(msg.data)
    def callback_v6(self, msg): self.v6_ = int(msg.data)
    def callback_v7(self, msg): self.v7_ = int(msg.data)
    def callback_p1(self, msg): self.p1_ = int(msg.data)
    
    # def callback_temp(self, msg): self.v7_ = float(f'{msg.data:.2f}')

    def check_system_stopped(self):
        self.counter += 1
        if self.system_stopped():
            
            self.timer_check_system_stopped_.cancel()
            self.log("System stopped")
            self.log_status()

            self.counter = 0
            if (self.next_status_ == 'A' or self.next_status_ == 'B'):
                self.start_tank(self.next_status_)
                self.timer_check_tank_started_.reset()

            if self.next_status_ == 'flush':
                self.start_flushing()
                self.timer_check_flushing_started_.reset()
        else:
            self.log("Waiting for system to stop...")
            if self.counter >= 5:
                self.stop_system()
    
    def check_tank_started(self):
        self.counter += 1
        
        if not self.tank_started(self.next_status_):
            if self.counter >= 5:
                self.log("Waiting for tank " + self.next_status_ + " to start")
                self.counter = 0
                self.start_tank(self.next_status_)
        else:
            self.timer_check_tank_started_.cancel()
            self.log("Tank " + self.next_status_ + " started")
            self.log_status()
            self.timer_control_temp_.reset()
            self.timer_switch_tank_.reset()
    
    def check_flushing_started(self):
        self.counter += 1
        if not self.flushing_started():
            if self.counter >= 5:
                self.log("Waiting for flushing to start")
                self.counter = 0
                self.start_flushing()
        else:
            self.timer_check_flushing_started_.cancel()
            self.log("Flushing started")
            self.log_status()
            self.timer_stop_flushing_.reset()
            
            
    def start_tank(self, tank):
        if tank == 'A':
            self.move_actuator('v1', CONST_OPEN)
            self.move_actuator('v2', CONST_OPEN)

        if tank == 'B':
            self.move_actuator('v3', CONST_OPEN)
            self.move_actuator('v4', CONST_OPEN)

    def switch_tank(self):
        self.timer_control_temp_.cancel()
        self.timer_switch_tank_.cancel()
        self.next_status_ = 'flush'
        self.stop_system()

    def start_flushing(self):
        self.move_actuator('v5', CONST_OPEN)
        self.move_actuator('v6', CONST_OPEN)

    def stop_flushing(self):
        self.timer_stop_flushing_.cancel()
        if self.last_tank_ == 'A': self.next_status_ = 'B'
        if self.last_tank_ == 'B': self.next_status_ = 'A'
        self.stop_system()

    def control_temp(self):
        a = 1
        # self.get_logger().info("Temp: " + str(self.temp_))
        
        # if self.temp_setpoint_ > self.temp_ + self.temp_hysteresis_:
        #     self.get_logger().info("Heating pls")
        # if self.temp_setpoint_ < self.temp_ - self.temp_hysteresis_:
        #     self.get_logger().info("Cooling pls")

    def system_stopped(self):
        if (self.v1_ == CONST_CLOSED and
            self.v2_ == CONST_CLOSED and
            self.v3_ == CONST_CLOSED and 
            self.v4_ == CONST_CLOSED and
            self.v5_ == CONST_CLOSED and
            self.v6_ == CONST_CLOSED and
            self.p1_ == CONST_OFF):
            return True
        else:
            return False
    
    def tank_started(self, tank):
        if tank == 'A':
            if (self.v1_ == CONST_OPEN and self.v2_ == CONST_OPEN): return True
        elif tank == 'B':
            if (self.v3_ == CONST_OPEN and self.v4_ == CONST_OPEN): return True
        else:
            return False
    
    def flushing_started(self):
        if (self.v5_ == CONST_OPEN and self.v6_ == CONST_OPEN): return True
        else: return False
    
    def log_status(self):
        self.log(
                "v1: " + str(self.v1_) +
                ", v2: " + str(self.v2_) +
                ", v3: " + str(self.v3_) +
                ", v4: " + str(self.v4_) +
                ", v5: " + str(self.v5_) +
                ", v6: " + str(self.v6_) +
                ", p1: " + str(self.p1_)
                )

    def stop_system(self):
        self.move_actuator('p1', CONST_OFF)
        self.move_actuator('v1', CONST_CLOSED)
        self.move_actuator('v2', CONST_CLOSED)
        self.move_actuator('v3', CONST_CLOSED)
        self.move_actuator('v4', CONST_CLOSED)
        self.move_actuator('v5', CONST_CLOSED)
        self.move_actuator('v6', CONST_CLOSED)
        self.log('Waiting for system to stop: ')
        self.counter = 0
        self.timer_check_system_stopped_.reset()

    def move_actuator(self, actuator_name , new_status):
        service_name = "/meso/" + actuator_name + "_control"
        client = self.create_client(ActuatorControl, service_name)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service " + service_name)
    
        request = ActuatorControl.Request()
        request.new_status = float(new_status)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_actuator_control, actuator_name=actuator_name))
        
    def callback_actuator_control(self, future, actuator_name):
        try:
            response = future.result()
            requested_status = float(f'{response.requested_status:.1f}')
            #self.get_logger().info("Requested status " + actuator_name + ": " + str(requested_status))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def log(self, msg = "", include_datetime = True):
        
        if include_datetime:
            now = datetime.now()
            msg = now.strftime("%d/%m/%Y %H:%M:%S") + ' - ' + msg
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BigControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
