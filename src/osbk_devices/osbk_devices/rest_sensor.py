import rclpy
from osbk_interfaces.msg import OSBKFloatValue
from .sensor_base import SensorBase
import requests


class RestSensor(SensorBase):
    """
    Node for reading single-value-data from REST-Servers.

    Inherits from SensorBase and implements the 'read_sensor' function to get
    a single-value from a REST-server.

    :param source_url: Url to the REST-server.
    :type source_url: ROS-param(String)
    :param path_to_value: The path to the value, to extract from the json response.
    :type path_to_value: ROS-param(String)
    :param unit: Unit of the sensor-values.
    :type unit: ROS-param(String)
    """

    def __init__(self,
                 name: str = 'RESTSensor',
                 read_interval: float = 10) -> None:
        """
        Construct an instance of RestSensor.

        Initializes name of node, the ROS-message-interface to use and its
        ROS-parameters.

        :param name: The name for this node. Defaults to 'RESTSensor'.
        :type name: str
        :param read_interval: Interval in seconds to publish sensor-readings.
            Defaults to 10.
        :type read_interval: float
        """
        # call constructor of SensorBase
        super().__init__(name, read_interval, OSBKFloatValue)

        # initialize ROS-parameters
        self.declare_parameter('source_url')
        self.declare_parameter('path_to_value')
        self.declare_parameter('unit')

    def read_sensor(self) -> OSBKFloatValue:
        """
        Read the current value from the REST-server.

        Overwrites the abstract method in SensorBase.
        Retrieves a response from the REST-server specified in the 'source_url'
        parameter and extracts the float value from the response according to
        the 'path_to_value' parameter.

        :return: The current value read from the sensor.
        """
        # create the message object
        msg = OSBKFloatValue()

        # with topic name and unit
        msg.topic_name = self.publish_topic
        try:
            msg.unit = self.get_parameter('unit').value
        except AssertionError:
            # do nothing if parameter is still empty
            return None

        # send a request to the REST-server
        url = self.get_parameter('source_url').value
        if type(url) == str:
            response = requests.get(url)
        else:
            return None

        # parse json as dict
        response = response.json()
        # if response is empty do nothing
        if(response == [] or response is None):
            return None

        # extract the specific value according to ros-param
        for key in self.get_parameter('path_to_value').value.split('.'):
            try:
                # if this nested level is dict:
                response = response[key]
            except TypeError:
                # or a list:
                response = response[int(key)]
            except KeyError:
                # message is probably not what is expected
                return None

        msg.data = float(response)

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
