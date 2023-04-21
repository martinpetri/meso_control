import pytest
import rclpy

from osbk_devices.sensor_base import SensorBase
from awi_interfaces.msg import AWIFloatValue

SENSOR_NAME = 'test_sensor'
SENSOR_VALUE = 1.2
SENSOR_UNIT = 'mV'

PUBLISH_TOPIC = 'test_sensor/value'


class MockSensor(SensorBase):
    def read_sensor(self):
        msg = AWIFloatValue()
        msg.topic_name = self.publish_topic
        msg.unit = SENSOR_UNIT
        msg.data = SENSOR_VALUE
        return msg


@pytest.fixture
def sensor_obj():
    rclpy.init()
    return MockSensor(SENSOR_NAME)


def test_object_creation():
    rclpy.init()
    obj = MockSensor(SENSOR_NAME)
    assert obj is not None
    rclpy.shutdown()


def test_properties(sensor_obj):
    assert sensor_obj.get_name() == SENSOR_NAME
    assert sensor_obj.publish_topic == PUBLISH_TOPIC
    assert sensor_obj.msg_interface == AWIFloatValue
    rclpy.shutdown()


def test_read_sensor(sensor_obj):
    reading = sensor_obj.read_sensor()

    assert reading.topic_name == PUBLISH_TOPIC
    assert reading.unit == SENSOR_UNIT
    assert reading.data == SENSOR_VALUE
    rclpy.shutdown()
