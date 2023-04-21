import rclpy
from awi_interfaces import AWIFloatValue
from osbk_devices.sensor_base import SensorBase
import requests


class RestSensor(SensorBase):

    def __init__(self):
        super.__init__(name='RESTSensor', msg_interface=AWIFloatValue)

        self.declare_parameter('source_url')
        self.declare_parameter('path_to_value')
        self.declare_parameter('unit')

    def read_sensor(self):
        msg = AWIFloatValue()
        msg.topic_name = self.publish_topic
        msg.unit = self.get_parameter('unit')

        url = self.get_parameter('source_url')
        if type(url) == str:
            response = requests.get(self.get_parameter('source_url'))
        else:
            return None

        response = response.json

        for key in self.get_parameter('path_to_value').split('.'):
            response = response[key]

        msg.data = response

        return msg


def main(args=None):
    rclpy.init(args=args)

    sensor = RestSensor()

    rclpy.spin(sensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
