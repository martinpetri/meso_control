import rclpy
from rclpy.node import Node
from rclpy.task import Future

from typing import TypeVar
from threading import Thread
import warnings

from osbk_interfaces.srv import DiscreteActuatorControl, ContinuousActuatorControl
from osbk_interfaces.msg import DiscreteActuatorState, ContinuousActuatorState
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)


class Actuator(Node):
    """Simple node to mock up an actuator with a Publisher and Service."""

    def __init__(self, name: str, srv_name: str, topic: str):
        super().__init__(name)

        self.publisher = self.create_publisher(ContinuousActuatorState, topic, 10)
        self.service = self.create_service(ContinuousActuatorControl, srv_name, self._callback)

    def _callback(self,
                  request: ContinuousActuatorControl.Request,
                  response: ContinuousActuatorControl.Response
                  ) -> ContinuousActuatorControl.Response:
        self.get_logger().warn(f"request received:{request}")
        response.requested_status = request.new_status
        msg = ContinuousActuatorState()
        msg.state = request.new_status
        msg.topic_name = self.publisher.topic_name
        self.publisher.publish(msg)
        return response


def main():
    rclpy.init()
    test_actuator_node = Actuator("test_actuator", "test_actuator/control", "test_actuator/state")
    rclpy.spin(test_actuator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
