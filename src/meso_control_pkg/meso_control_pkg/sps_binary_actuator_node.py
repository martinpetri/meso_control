import rclpy
from rclpy.subscription import Subscription
from rclpy.client import Client

import json

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import DiscreteActuatorControl, Modbus
from osbk_interfaces.msg import DiscreteActuatorState, OSBKStringValue


class SpsBinaryActuator(ActuatorBase):
    """Simple Actuator, that influences a shutoff-valve controlled by an SPS."""

    def __init__(self) -> None:
        super().__init__("sps_valve", False, 1)

        self.declare_parameter("modbus_read_topic", "modbus_tcp_node/read")
        self.declare_parameter("read_key", "V1")
        self.declare_parameter("modbus_write_service", "modbus_tcp_node/write")
        self.declare_parameter("write_key", "V1")

        self.last_state: int = 0

        self.modbus_subscription: Subscription = self.create_subscription(
            OSBKStringValue,
            self.get_parameter("modbus_read_topic").value,
            self._subscription_callback,
            10)
        
        self.write_client: Client = self.create_client(
            Modbus,
            self.get_parameter("modbus_write_service").value
        )

    def set_actuator(self,
                     setpoint: DiscreteActuatorControl.Request
                     ) -> DiscreteActuatorControl.Response:
        modbus_request = Modbus.Request()
        modbus_request.key_name = self.get_parameter("write_key").value
        if setpoint.new_status > 0:
            modbus_request.value_to_send = "1"
        else:
            modbus_request.value_to_send = "0"

        self.future = self.write_client.call_async(modbus_request)

        response = DiscreteActuatorControl.Response()
        response.requested_status = setpoint.new_status
        return response

    def poll_status(self) -> DiscreteActuatorState:
        msg = DiscreteActuatorState()
        msg.topic_name = self.publish_topic
        msg.state = self.last_state
        return msg
    
    def _subscription_callback(self, msg: OSBKStringValue) -> None:
        feedback_values = json.loads(msg.data)
        key = self.get_parameter("read_key").value
        self.last_state = int(feedback_values[key])



def main():
    rclpy.init()
    sps_binary_actuator_node = SpsBinaryActuator()
    try:
        rclpy.spin(sps_binary_actuator_node)
    except(KeyboardInterrupt):
        sps_binary_actuator_node.get_logger().info("Shutting down.")

    sps_binary_actuator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()