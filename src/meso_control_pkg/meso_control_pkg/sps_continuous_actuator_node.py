import rclpy
from rclpy.subscription import Subscription
from rclpy.client import Client

import json

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import ContinuousActuatorControl, Modbus
from osbk_interfaces.msg import ContinuousActuatorState, OSBKStringValue


class SpsContinuousActuator(ActuatorBase):
    """
    A node for a continuous actuator controlled by an SPS, connected via Modbus.

    Uses the ``ModbusTcpNode``.

    :param last_state: The last state received from the ``ModbusTcpNode``.
    :type last_state: float

    :param modbus_subscription: The subscriber listening to the ``ModbusTcpNode``.
    :type modbus_subscription: Subscription

    :param write_client: The client for writing to the SPS via the ``ModbusTcpNode``.
    :type write_client: Client
    """

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
        """
        Send a new setpoint to the actuator.

        :param setpoint: A service-request-object that contains the new setpoint.
        :type setpoint: ContinuousActuatorControl.Request

        :return: A service-response-object to confirm the request.
        :rtype: ContinuousActuatorControl.Response
        """
        modbus_request = Modbus.Request()
        modbus_request.key_name = self.get_parameter("write_key").value
        modbus_request.value_to_send = str(round(setpoint.new_status * 100.0))

        while not self.write_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting...')
        self.future = self.write_client.call_async(modbus_request)

        response = ContinuousActuatorControl.Response()
        response.requested_status = setpoint.new_status
        return response

    def poll_status(self) -> ContinuousActuatorState:
        """
        Return the last transmitted actuator-state by the SPS.

        :return: A message object containing the actuator-state.
        :rtype: ContinuousActuatorState
        """
        msg = ContinuousActuatorState()
        msg.topic_name = self.publish_topic
        msg.state = self.last_state
        return msg
    
    def _subscription_callback(self, msg: OSBKStringValue) -> None:
        feedback_values = json.loads(msg.data)
        key = self.get_parameter("read_key").value
        if key in feedback_values:
            self.last_state = round(float(feedback_values[key]) / 100.0, 2)


def main():
    rclpy.init()
    sps_continuous_actuator_node = SpsContinuousActuator()
    try:
        rclpy.spin(sps_continuous_actuator_node)
    except(KeyboardInterrupt):
        sps_continuous_actuator_node.get_logger().info("Shutting down.")

    sps_continuous_actuator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
