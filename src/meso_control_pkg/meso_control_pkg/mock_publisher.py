import rclpy
from rclpy.node import Node

import json

from osbk_interfaces.msg import OSBKStringValue
from osbk_interfaces.srv import Modbus


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mock_publisher')
        self.publisher_ = self.create_publisher(OSBKStringValue, "modbus_tcp_node/read", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.service = self.create_service(Modbus, "modbus_tcp_node/write", self.service_callback)

        self.feedback_values = {
            "V1":"1",
            "V2":"0",
            "V3":"0",
            "V4":"0",
            "PUMP":"1",
            "UPPER_MAGNETIC_VALVE":"0",
            "LOWER_MAGNETIC_VALVE":"0",
            "HEAT_A":"0",
            "HEAT_B":"0",
            "COOL_A":"0",
            "COOL_B":"0",
            "TIDE_A":"100",
            "TIDE_B":"100",
            "TEMP_SETPOINT_A":"1200",
            "TEMP_SETPOINT_B":"1200",
            "SPARE_16":"0",
            "SPARE_17":"0",
            "SPARE_18":"0",
            "LAST_STEP_NUMBER":"2",
            "MANUAL_CONTROL":"0",
            "RO_TEMP":"1250",
            "RO_PH":"1360",
            "RO_COND":"1470",
            "RO_OXYGEN":"1580",
            "RO_SPARE_25":"0",
            "RO_SPARE_26":"0",
            "RO_SPARE_27":"0",
            "RO_SPARE_28":"0",
            "RO_SPARE_29":"0",
            "RO_SPARE_30":"0",
            "RO_SPARE_31":"0",
            "RO_SPARE_32":"0"
        }

        self.write_values = {
            "V1":"1",
            "V2":"0",
            "V3":"0",
            "V4":"0",
            "PUMP":"1",
            "UPPER_MAGNETIC_VALVE":"0",
            "LOWER_MAGNETIC_VALVE":"0",
            "HEAT_A":"0",
            "HEAT_B":"0",
            "COOL_A":"0",
            "COOL_B":"0",
            "TIDE_A":"100",
            "TIDE_B":"100",
            "TEMP_SETPOINT_A":"1200",
            "TEMP_SETPOINT_B":"1200",
            "SPARE_16":"0",
            "SPARE_17":"0",
            "SPARE_18":"0",
            "SPARE_19":"0",
            "SPARE_20":"0",
            "SPARE_21":"0",
            "SPARE_22":"0",
            "SPARE_23":"0",
            "SPARE_24":"0",
            "SPARE_25":"0",
            "SPARE_26":"0",
            "SPARE_27":"0",
            "SPARE_28":"0",
            "SPARE_29":"0",
            "HEARTBEAT":"0",
            "LAST_STEP_NUMBER":"2",
            "MANUAL_CONTROL":"0"
        }

    def timer_callback(self):
        msg = OSBKStringValue()
        msg.topic_name = "modbus_tcp_node/read"
        msg.data = json.dumps(self.feedback_values)
        msg.unit = "json"
        self.publisher_.publish(msg)
        string = ""
        for i in range(32):
            string += f"{list(self.feedback_values)[i]:>20}: {list(self.feedback_values.values())[i]:4},\t{list(self.write_values)[i]:>20}: {list(self.write_values.values())[i]:4}\n"
        self.get_logger().info("\nFeedback-Values:\t\tWrite-Values:\n" + string)

    def service_callback(self,
                         request: Modbus.Request,
                         response: Modbus.Response
                         ) -> Modbus.Response:
        self.write_values[request.key_name] = request.value_to_send
        if request.key_name in self.feedback_values:
            self.feedback_values[request.key_name] = request.value_to_send
        response.key_name = request.key_name
        response.value_to_send = request.value_to_send
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except(KeyboardInterrupt):
        minimal_publisher.get_logger().info("Shutting down.")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()