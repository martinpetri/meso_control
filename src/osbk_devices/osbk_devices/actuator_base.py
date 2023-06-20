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

    Nodes, that implement a specific actuator directly connected to the
    controller should inherit from this base class and overwrite the
    :func:'set_actuator()' function. This class also inherits from Node.

    :param service_name: name of the service this actuator can be
        controlled with
    :type service_name: str
    :param srv: the service registered with ROS, its callback function calls
        set_actuator()
    :type srv: Service
    """

    def __init__(self,
                 name: str,
                 continuous: bool = True,
                 status_poll_interval: int = 1000) -> None:
        """
        Construct instance of :class: 'ActuatorBase'.

        Initializes a service and its name as attributes.

        :param name: name for the node
        :type name: str
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
        self.service_name: str = f"{name}/control"
        self.srv: _rclpy.Service = self.create_service(self._control_interface,
                                                       self.service_name,
                                                       self._command_callback)

        # create a timer to periodically retrieve the current status
        self.status_poll_interval: int = status_poll_interval
        self.poll_timer: Timer = self.create_timer(self.status_poll_interval,
                                                   self.poll_status)
        self.current_status: MsgType = None

        # create publisher for the actuators current state
        self.publish_topic = f"{name}/state"
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

        :param setpoint: the Request sent to self.srv
        :type setpoint: SrvTypeRequest
        :return: confirmation of the setpoint
        :rtype: SrvTypeResponse
        """
        pass

    def _publish_status(self) -> None:
        """Publish the status returned by self.poll_status() if changed."""
        status = self.poll_status()
        if status != self.current_status:
            status.topic_name = self.publish_topic
            self.publisher.publish(status)
            self.current_status = status

    @abstractmethod
    def poll_status(self) -> MsgType:
        """Abstract method that should retrieves the current actuator status."""
        pass
