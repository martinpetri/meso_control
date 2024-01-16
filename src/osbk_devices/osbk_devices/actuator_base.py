from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.timer import Timer
from typing import TypeVar
from abc import ABC, abstractmethod

from osbk_interfaces.srv import ContinuousActuatorControl, DiscreteActuatorControl
from osbk_interfaces.msg import ContinuousActuatorState, DiscreteActuatorState


MsgType = TypeVar('MsgType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class ActuatorBase(Node, ABC):
    """
    Abstract base class for actuator implementations.

    Nodes that implement a specific actuator should inherit from this base class and overwrite the
    ``set_actuator()`` and ``poll_status()`` function. This class also inherits from Node.

    :param service_name: The name of the service this actuator can be
        controlled with.
    :type service_name: str
    :param srv: The service registered with ROS. Its callback function calls
        ``set_actuator()``.
    :type srv: Service
    :param poll_timer: The timer to trigger publishing of the current status.
    :type poll_timer: Timer
    :param publisher: A publisher to publish current actuator status periodically.
    :type publisher: Publisher
    """

    def __init__(self,
                 name: str,
                 continuous: bool = True,
                 status_poll_interval: int = 1) -> None:
        """
        Construct instance of ``ActuatorBase``.

        Initializes a service to receive commands and a publisher to publish the
        actuators current status.

        :param name: The name for the node.
        :type name: str
        :param continuous: Wether this actuator should use float (True) or integer (False) values
            for status representation. Defaults to True.
        :type continuous: bool
        :param status_poll_interval: The interval to publish the actuators status in seconds.
        :type status_poll_interval: int
        """
        # call constructor for Node
        super().__init__(name)

        # continuous or discrete actuator states
        self._continuous: bool = continuous
        if continuous:
            self._control_interface = ContinuousActuatorControl
            self._publish_interface = ContinuousActuatorState
        else:
            self._control_interface = DiscreteActuatorControl
            self._publish_interface = DiscreteActuatorState

        # initialize the service to control this actuator with
        self.service_name: str = f"{self.get_name()}/control"
        self.srv: _rclpy.Service = self.create_service(self._control_interface,
                                                       self.service_name,
                                                       self._command_callback)

        # create a timer to periodically retrieve the current status
        self.status_poll_interval: int = status_poll_interval
        self.poll_timer: Timer = self.create_timer(self.status_poll_interval,
                                                   self._publish_status)
        self.current_status: MsgType = None

        # create publisher for the actuators current state
        self.publish_topic = f"{self.get_name()}/state"
        self.publisher: _rclpy.Publisher = self.create_publisher(
            self._publish_interface,
            self.publish_topic,
            10
        )

    def _command_callback(self,
                          request: SrvTypeRequest,
                          response: SrvTypeResponse) -> SrvTypeResponse:
        # callback function for self.srv that calls self.set_actuator
        response = self.set_actuator(request)
        return response

    @abstractmethod
    def set_actuator(self,
                     setpoint: SrvTypeRequest) -> SrvTypeResponse:
        """
        Abstract method that sets the actuator to a new setpoint.

        This method should be overridden by specific hardware implementation.

        :param setpoint: The Request sent to ``self.srv``.
        :type setpoint: SrvTypeRequest
        :return: Confirmation of the setpoint.
        :rtype: SrvTypeResponse
        """
        pass

    def _publish_status(self) -> None:
        """Publish the status returned by ``self.poll_status()`` if changed."""
        status = self.poll_status()
        # if status != self.current_status:
        status.topic_name = self.publish_topic
        self.publisher.publish(status)
        self.current_status = status

    @abstractmethod
    def poll_status(self) -> MsgType:
        """Abstract method that should retrieve the current actuator status."""
        pass
