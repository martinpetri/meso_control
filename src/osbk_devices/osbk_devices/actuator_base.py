from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from typing import TypeVar
from abc import ABC, abstractmethod

from awi_interfaces.srv import ActuatorControl

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

    def __init__(self, name: str) -> None:
        """
        Construct instance of :class: 'ActuatorBase'.

        Initializes a service and its name as attributes.

        :param name: name for the node
        :type name: str
        """
        # call constructor for Node
        super().__init__(name)

        # initialize the service to control this actuator with
        self.service_name: str = f'{name}/control'
        self.srv: _rclpy.Service = self.create_service(ActuatorControl,
                                                       self.service_name,
                                                       self.__command_callback)

    def __command_callback(self,
                           request: SrvTypeRequest,
                           response: SrvTypeResponse) -> SrvTypeResponse:
        # callback function for self.srv that calls self.set_actuator
        response = self.set_actuator(request)
        return response

    @abstractmethod
    def set_actuator(self, setpoint: SrvTypeRequest) -> SrvTypeResponse:
        """
        Abstract method that sets the actuator to a new setpoint.

        This method should be overriden by specific hardware implementation.

        :param setpoint: the Request sent to self.srv
        :type setpoint: SrvTypeRequest
        :return: confirmation of the setpoint
        :rtype: SrvTypeResponse
        """
        pass
