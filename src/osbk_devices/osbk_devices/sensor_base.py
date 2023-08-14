from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from std_srvs.srv import Trigger

from typing import TypeVar, List
from abc import ABC, abstractmethod

from osbk_interfaces.msg import OSBKFloatValue

MsgType = TypeVar('MsgType')


class SensorBase(Node, ABC):
    """
    Abstract base class for sensor implementations.

    Nodes that implement a specific sensor should inherit from this base class and overwrite the
    ``read_sensor()`` function. This Class also inherits from Node.

    :param read_interval: ROS parameter to change the poll-rate for sensor readings
    :type read_interval: ROS parameter
    :param publish_topic: topic name, the sensors readings are published to,
        defaults to ``'[node_name]/value'`` test
    :type publish_topic: str
    :param publish_service_name: name of the service to publish sensor readings
    :type publish_service_name: str
    :param msg_interface: the msg-interface this sensor uses to publish its
        readings
    :type msg_interface: MsgType
    :param publisher: ROS publisher for sending the readings
    :type publisher: Publisher
    :param publish_service: the service to request a publish of sensor readings
    :type publish_service: Service
    :param publish_timer: ROS timer to schedule publishing of sensor-readings
    :type publish_timer: Timer
    """

    def __init__(self,
                 name: str,
                 read_interval: float,
                 msg_interface: MsgType = OSBKFloatValue) -> None:
        """
        Construct instance of ``'SensorBase'``.

        Initializing the nodes name and its attributes for publishing sensor
        values.

        :param name: name of the node
        :type name: str
        :param read_interval: interval in seconds to publish sensor-readings
        :type read_interval: float
        :param msg_interface: ROS msg-interface to use for publishing,
            defaults to ``'OSBKFloatValue'``
        :type msg_interface: MsgType
        """
        # call the constructor of Node
        super().__init__(name)

        # initialize topic name, ros-interface
        self.publish_topic: str = f'{self.get_name()}/value'
        self.publish_service_name: str = f'{self.get_name()}/request_publish'

        self.msg_interface: MsgType = msg_interface

        # create publisher for sensor readings
        self.publisher: _rclpy.Publisher = self.create_publisher(
            msg_interface,
            self.publish_topic,
            10
        )

        # create service for reading and publishing sensor
        self.publish_service: _rclpy.Service = self.create_service(
            Trigger,
            self.publish_service_name,
            self._publish_request_callback
        )

        # create the timer to publish sensor-readings periodically
        self.publish_timer: Timer = self.create_timer(read_interval,
                                                      self.publish_reading)

        # make timer interval configurable
        self.declare_parameter('read_interval', read_interval)
        self.add_on_set_parameters_callback(self._parameter_set_callback)

    def _publish_request_callback(self,
                                  request: Trigger.Request,
                                  response: Trigger.Response) -> Trigger.Response:
        """
        Callback-function for the publish_service.

        Calls the publish_reading method.

        :param request: service-request, should be empty
        :type request: Trigger.Request
        :param response: service-response, with a success flag and a message field
        :type response: Trigger.Response
        """
        # request is empty
        del request

        # prepare response
        response.success = True
        response.message = 'OK.'

        # TODO: probably let publish_reading return a bool, for proper error signaling
        self.publish_reading()

        return response

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
        Publish what ``read_sensor()`` returns.

        :rtype: None
        """
        msg = self.read_sensor()
        if(msg is not None):
            self.publisher.publish(msg)

    @abstractmethod
    def read_sensor() -> MsgType:
        """
        Abstract method that returns a sensor reading.

        This should be overridden for specific hardware implementation.

        :return: an instance of the MsgType specified in ``self.msg_interface``
        """
        pass
