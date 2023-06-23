import rclpy
from rclpy.subscription import Subscription
from rclpy.client import Client

import json

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import ContinuousActuatorControl, Modbus
from osbk_interfaces.msg import ContinuousActuatorState, OSBKStringValue


class SpsContinuousActuator(ActuatorBase):
    """Simple Actuator, that influences a shutoff-valve controlled by an SPS."""

    def __init__(self) -> None:
        super().__init__("temp_control_a", True, 1)

        self.declare_parameter("modbus_read_topic", "modbus_tcp_node/read")
        self.declare_parameter("read_key", "TEMP_SETPOINT_A")
        self.declare_parameter("modbus_write_service", "modbus_tcp_node/write")
        self.declare_parameter("write_key", "TEMP_SETPOINT_A")

        self.last_state: float = 0.0

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
                     setpoint: ContinuousActuatorControl.Request
                     ) -> ContinuousActuatorControl.Response:
        modbus_request = Modbus.Request()
        modbus_request.key_name = self.get_parameter("write_key").value
        modbus_request.value_to_send = str(round(setpoint.new_status * 100.0))

        self.future = self.write_client.call_async(modbus_request)

        response = ContinuousActuatorControl.Response()
        response.requested_status = setpoint.new_status
        return response

    def poll_status(self) -> ContinuousActuatorState:
        msg = ContinuousActuatorState()
        msg.topic_name = self.publish_topic
        msg.state = self.last_state
        return msg
    
    def _subscription_callback(self, msg: OSBKStringValue) -> None:
        feedback_values = json.loads(msg.data)
        key = self.get_parameter("read_key").value
        self.last_state = round(float(feedback_values[key]) / 100.0, 2)



def main():
    rclpy.init()
    sps_coontinuous_actuator_node = SpsContinuousActuator()
    try:
        rclpy.spin(sps_coontinuous_actuator_node)
    except(KeyboardInterrupt):
        sps_coontinuous_actuator_node.get_logger().info("Shutting down.")

    sps_coontinuous_actuator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
