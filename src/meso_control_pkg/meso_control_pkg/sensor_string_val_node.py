#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from awi_interfaces.msg import AWIStringValue

class SensorStringValNode(Node):
    def __init__(self):
        super().__init__("sensor_string_val_node")

        self.current_value_ = 'Start Value: 0'
        self.declare_parameter("sps_api_url", "http://192.168.23.100/awp/api.html")
        self.declare_parameter("name", "MultiSensor")
        self.declare_parameter("topic_name", "multi_sensor")
        self.declare_parameter("poll_interval", 1)
        self.declare_parameter("publish_interval", 1)

        self.sps_api_url_ = self.get_parameter("sps_api_url").value
        self.name_ = self.get_parameter("name").value
        self.topic_name_ = self.get_parameter("topic_name").value
        self.poll_interval_ = self.get_parameter("poll_interval").value
        self.publish_interval_ = self.get_parameter("publish_interval").value

        self.value_publisher_ = self.create_publisher(AWIStringValue, self.topic_name_, 10)
        self.timer1_ = self.create_timer(self.publish_interval_, self.publish_value)
        self.timer2_ = self.create_timer(self.poll_interval_, self.poll_value)
        
    def publish_value(self):
        msg = AWIStringValue()
        msg.topic_name = self.topic_name_
        msg.data = self.current_value_
        self.value_publisher_.publish(msg)

    def poll_value(self):
        self.current_value_ = '{"temp":0.0}'
        # try:
        #     response = requests.get(self.sps_api_url_, verify = False, timeout=5)
        #     data = response.json()
        #     self.position_ = round(float(data[self.valve_name_]),1)    
        # except Exception as e:
        #     self.position_ = 0.0
        #     self.get_logger().error("Failed to get valve_position %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SensorStringValNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()