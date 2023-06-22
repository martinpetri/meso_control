import rclpy
from rclpy.node import Node

import json

from osbk_interfaces.msg import OSBKStringValue


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(OSBKStringValue, "modbus_tcp_node/json_modbus_values", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = {
            "RO_TEMP": 100,
            "RO_COND": 200,
            "RO_OXYGEN": 300,
            "RO_PH": 400
        }
        msg = OSBKStringValue()
        msg.topic_name = "modbus_tcp_node/json_modbus_values"
        msg.data = json.dumps(data)
        msg.unit = "json"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()