import rclpy

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import ContinuousActuatorControl
from osbk_interfaces.msg import ContinuousActuatorState


class ExampleActuator(ActuatorBase):
    """Simple Actuator, that influences a simple float variable self.status."""

    def __init__(self):
        super().__init__("example_actuator", True, 1)

        self.status: float = 0.0

    def set_actuator(self,
                     setpoint: ContinuousActuatorControl.Request
                     ) -> ContinuousActuatorControl.Response:
        self.status = setpoint.new_status

        response = ContinuousActuatorControl.Response()
        response.requested_status = setpoint.new_status

        return response

    def poll_status(self) -> ContinuousActuatorState:
        msg = ContinuousActuatorState()
        msg.topic_name = self.publish_topic
        msg.state = self.status
        return msg


def main():
    rclpy.init()
    test_actuator_node = ExampleActuator()
    rclpy.spin(test_actuator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
