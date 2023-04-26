import rclpy
from awi_interfaces.msg import AWIFloatValue
from osbk_devices.sensor_base import SensorBase
import requests


class RestSensor(SensorBase):
    """
    Node for reading single-value-data from REST-Servers.

    Inherits from SensorBase and implements the 'read_sensor' function to get
    a single-value from a REST-server.

    :param source_url: url to the REST-server
    :type source_url: ROS-param(String)
    :param path_to_value: path to value, to extract from json response
    :type path_to_value: ROS-param(String)
    :param unit: unit of the sensor-values
    :type unit: ROS-param(String)
    """

    def __init__(self) -> None:
        """
        Construct an instance of RestSensor.

        Initializes name of node, the ROS-message-interface to use and its
        ROS-parameters.
        """
        # call constructor of SensorBase
        super().__init__(name='RESTSensor', msg_interface=AWIFloatValue)

        # initialize ROS-parameters
        self.declare_parameter('source_url')
        self.declare_parameter('path_to_value')
        self.declare_parameter('unit')

    def read_sensor(self) -> AWIFloatValue:
        """
        Read the current value from the REST-server.

        Overwrites the abstract method in SensorBase.
        Retrieves a response from the REST-server specified in the 'source_url'
        parameter and extracts the float value from the response according to
        the 'path_to_value' parameter
        """
        # create the message object
        msg = AWIFloatValue()

        # with topic name and unit
        msg.topic_name = self.publish_topic
        msg.unit = self.get_parameter('unit').value

        # send a request to the REST-server
        url = self.get_parameter('source_url').value
        if type(url) == str:
            response = requests.get(url)
        else:
            return None

        # parse json as dict
        response = response.json()

        # extract the specific value according to ros-param
        for key in self.get_parameter('path_to_value').value.split('.'):
            response = response[key]
        msg.data = response

        return msg


def main(args=None):
    # entry point for a RESTSensor-node

    rclpy.init(args=args)

    # create node
    sensor = RestSensor()

    rclpy.spin(sensor)

    sensor.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
