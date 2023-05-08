import pytest
import rclpy

from typing import TypeVar

from osbk_devices.actuator_base import ActuatorBase
from awi_interfaces.srv import ActuatorControl

SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')

ACTUATOR_NAME = 'test_actuator'


class MockActuator(ActuatorBase):
    """Subclass of ActuatorBase for testing."""

    def __init__(self, name: str) -> None:
        """Construct object of MockActuator."""
        super().__init__(name)

        self.status: float = 0

    def set_actuator(self, setpoint: ActuatorControl.Request) -> ActuatorControl.Response:
        """Overwrite abstract set_actuator to just set a status variable."""
        self.status = setpoint.new_status

        response = ActuatorControl.Response()
        response.requested_status = self.status

        return response


@pytest.fixture
def actuator_obj():
    rclpy.init()
    return MockActuator(ACTUATOR_NAME)


def test_object_creation():
    rclpy.init()
    obj = MockActuator(ACTUATOR_NAME)

    assert obj is not None
    rclpy.shutdown()


def test_set_actuator(actuator_obj: MockActuator):
    request = ActuatorControl.Request()
    request.new_status = 5.0

    assert actuator_obj.status == 0.0

    response = actuator_obj.set_actuator(request)

    assert response.requested_status == request.new_status
    assert actuator_obj.status == request.new_status

    rclpy.shutdown()
