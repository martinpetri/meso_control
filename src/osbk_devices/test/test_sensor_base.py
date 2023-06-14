import pytest
import rclpy

from osbk_devices.sensor_base import SensorBase
from osbk_interfaces.msg import OSBKFloatValue

SENSOR_NAME = 'test_sensor'
SENSOR_VALUE = 1.2
SENSOR_UNIT = 'mV'

PUBLISH_TOPIC = 'test_sensor/value'


class MockSensor(SensorBase):
    """Subclass of SensorBase for testing."""

    def read_sensor(self) -> OSBKFloatValue:
        """Overwrite abstract read_sensor to return constant value."""
        # create OSBKFloatValue object
        msg = OSBKFloatValue()

        # with topic, unit and value
        msg.topic_name = self.publish_topic
        msg.unit = SENSOR_UNIT
        msg.data = SENSOR_VALUE

        # return it
        return msg


@pytest.fixture
def sensor_obj():
    rclpy.init()
    return MockSensor(SENSOR_NAME, 10)


def test_object_creation():
    rclpy.init()
    obj = MockSensor(SENSOR_NAME, 10)

    assert obj is not None

    rclpy.shutdown()


def test_attributes(sensor_obj):
    assert sensor_obj.get_name() == SENSOR_NAME
    assert sensor_obj.publish_topic == PUBLISH_TOPIC
    assert sensor_obj.msg_interface == OSBKFloatValue

    rclpy.shutdown()


def test_read_sensor(sensor_obj):
    reading = sensor_obj.read_sensor()

    assert reading.topic_name == PUBLISH_TOPIC
    assert reading.unit == SENSOR_UNIT
    assert reading.data == SENSOR_VALUE

    rclpy.shutdown()
