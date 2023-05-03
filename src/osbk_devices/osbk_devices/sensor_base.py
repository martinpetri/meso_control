from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from typing import TypeVar, List
from abc import ABC, abstractmethod

from awi_interfaces.msg import AWIFloatValue

MsgType = TypeVar('MsgType')


class SensorBase(Node, ABC):
    """
    Abstract base class for sensor implementations.

    Nodes, that implement a specific sensor directly connected to the
    controller, should inherit from this base class and overwrite the
    'read_sensor()' function. This Class also inherits from Node.

    :param publish_topic: topic name, the sensors readings are published to,
        defaults to '[node_name]/value'
    :type publish_topic: str
    :param msg_interface: the msg-interface this sensor uses to publish its
        readings
    :type msg_interface: MsgType
    :param publisher: ROS publisher for sending the readings
    :type publisher: Publisher
    :param publish_timer: ROS timer to schedule publishing of sensor-readings
    :type publish_timer: Timer
    """

    def __init__(self,
                 name: str,
                 read_interval: float,
                 msg_interface: MsgType = AWIFloatValue) -> None:
        """
        Construct instance of 'SensorBase'.

        Initializing the nodes name and its attributes for publishing sensor
        values.

        :param name: name of the node
        :type name: str
        :param read_interval: interval in seconds to publish sensor-readings
        :type read_interval: float
        :param msg_interface: ROS msg-interface to use for publishing,
            defaults to 'AWIFloatValue'
        :type msg_interface: MsgType
        """
        # call the constructor of Node
        super().__init__(name)

        # initialize topic name, interface and publisher
        self.publish_topic: str = f'{name}/value'

        self.msg_interface: MsgType = msg_interface

        self.publisher: _rclpy.Publisher = self.create_publisher(
            msg_interface,
            self.publish_topic,
            10
        )

        # create the timer to publish sensor-readings periodically
        self.publish_timer: Timer = self.create_timer(read_interval,
                                                      self.publish_reading)

        # make timer interval configurable
        self.declare_parameter('read_interval', read_interval)
        self.add_on_set_parameters_callback(self._parameter_set_callback)

    def _parameter_set_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        Callback-function for parameters being set.

        Updates the publish_timer-interval when read_interval is set.

        :param parameters: list of parameters that are being updated
        :type parameters: List[Parameter]
        :rtype: SetParametersResult
        """
        # check every updated parameter
        for param in parameters:

            if param.name == 'read_interval':
                # update the timer period
                self.publish_timer.timer_period_ns = param.value * 1e9

        result = SetParametersResult()
        result.successful = True
        return result

    def publish_reading(self) -> None:
        """
        Publish what :func: 'read_sensor()' returns.

        :rtype: None
        """
        msg = self.read_sensor()
        if(msg is not None):
            self.publisher.publish(msg)

    @abstractmethod
    def read_sensor():
        """
        Abstract method that returns a sensor reading.

        This should be overridden for specific hardware implementation.

        :return: an instance of the MsgType specified in self.msg_interface
        """
        pass
