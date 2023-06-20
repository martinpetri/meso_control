import pytest
import rclpy
from rclpy.node import Node

from typing import TypeVar
from threading import Thread

from osbk_devices.actuator_base import ActuatorBase
from osbk_interfaces.srv import ContinuousActuatorControl
from osbk_interfaces.msg import ContinuousActuatorState

SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')
MsgType = TypeVar('MsgType')

ACTUATOR_NAME = 'test_actuator'


class MockActuator(ActuatorBase):
    """Subclass of ActuatorBase for testing."""

    def __init__(self, name: str) -> None:
        """Construct object of MockActuator."""
        super().__init__(name)

        self.status: float = 0.0

    def set_actuator(self,
                     setpoint: ContinuousActuatorControl.Request
                     ) -> ContinuousActuatorControl.Response:
        """Overwrite abstract set_actuator to just set a status variable."""
        self.status = setpoint.new_status

        response = ContinuousActuatorControl.Response()
        response.requested_status = self.status

        return response

    def poll_status(self) -> ContinuousActuatorState:
        status = ContinuousActuatorState()
        status.state = self.status
        status.topic_name = self.publish_topic
        return status


class SubscriberClass(Node):
    """Simple node that stores the last published message."""

    def __init__(self, topic: str, type: MsgType):
        super().__init__("subscriber")

        self.subscription = self.create_subscription(type, topic, self._callback, 10)
        self.last_message: MsgType = None

    def _callback(self, msg: MsgType):
        self.last_message = msg


@pytest.fixture
def actuator_obj():
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
        rclpy.init()
    return MockActuator(ACTUATOR_NAME)


@pytest.fixture
def subscriber():
    return SubscriberClass(f"{ACTUATOR_NAME}/state", ContinuousActuatorState)


def test_object_creation():
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
        rclpy.init()
    obj = MockActuator(ACTUATOR_NAME)

    assert obj is not None
    rclpy.shutdown()


def test_set_actuator(actuator_obj: MockActuator):
    request = ContinuousActuatorControl.Request()
    request.new_status = 5.0

    assert actuator_obj.status == 0.0

    response = actuator_obj.set_actuator(request)

    assert response.requested_status == request.new_status
    assert actuator_obj.status == request.new_status

    rclpy.shutdown()


def test__command_callback(actuator_obj: MockActuator):
    request = ContinuousActuatorControl.Request()
    response = ContinuousActuatorControl.Response()

    assert actuator_obj.status == 0.0

    request.new_status = 5.0
    response = actuator_obj._command_callback(request, response)
    assert response.requested_status == request.new_status
    assert response.requested_status == 5.0
    assert actuator_obj.status == request.new_status
    rclpy.shutdown()


def test__publish_status(actuator_obj: MockActuator, subscriber: SubscriberClass):
    assert actuator_obj.status == 0.0
    assert subscriber.last_message is None

    subscriber_thread = Thread(target=lambda: rclpy.spin_once(subscriber))
    subscriber_thread.start()
    actuator_obj._publish_status()
    subscriber_thread.join()

    assert subscriber.last_message.state == 0.0

    request = ContinuousActuatorControl.Request()
    request.new_status = 5.0
    actuator_obj.set_actuator(request)

    subscriber_thread = Thread(target=lambda: rclpy.spin_once(subscriber))
    subscriber_thread.start()
    actuator_obj._publish_status()
    subscriber_thread.join()

    assert subscriber.last_message.state == 5.0
    rclpy.shutdown()
