import rclpy
from rclpy.subscription import Subscription
from rclpy.client import Client

import json

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import DiscreteActuatorControl, Modbus
from osbk_interfaces.msg import DiscreteActuatorState, OSBKStringValue


class SpsBinaryActuator(ActuatorBase):
    """
    A node for a binary actuator controlled by an SPS, connected via Modbus.

    Uses the ``ModbusTcpNode``.

    :param last_state: the last state received from the ``ModbusTcpNode``, either 0 or 1
    :type last_state: int

    :param modbus_subscription: the subscriber listening to the ``ModbusTcpNode``
    :type modbus_subscription: Subscription

    :param write_client: the client for writing to the SPS via the ``ModbusTcpNode``
    :type write_client: Client
    """

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
        """
        Send a new setpoint to the actuator.

        :param setpoint: a service-request-object that contains the new setpoint
        :type setpoint: DiscreteActuatorControl.Request

        :return: a service-response-object to confirm the request
        :rtype: DiscreteActuatorControl.Response
        """
        modbus_request = Modbus.Request()
        modbus_request.key_name = self.get_parameter("write_key").value
        if setpoint.new_status > 0:
            modbus_request.value_to_send = "1"
        else:
            modbus_request.value_to_send = "0"

        while not self.write_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting...')
        self.future = self.write_client.call_async(modbus_request)

        response = DiscreteActuatorControl.Response()
        response.requested_status = setpoint.new_status
        return response

    def poll_status(self) -> DiscreteActuatorState:
        """
        Return the last transmitted actuator-state by the SPS.

        :return: a message object containing the actuator-state
        :rtype: DiscreteActuatorState
        """
        msg = DiscreteActuatorState()
        msg.topic_name = self.publish_topic
        msg.state = self.last_state
        return msg
    
    def _subscription_callback(self, msg: OSBKStringValue) -> None:
        feedback_values = json.loads(msg.data)
        key = self.get_parameter("read_key").value
        if key in feedback_values:
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
