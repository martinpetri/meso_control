from rclpy.node import Node

from statemachine import StateMachine, State
from abc import ABC

class ActuatorStateMachine(Node, ABC):
    """A node that manages actuators following a finite statemachine."""
    def __init__(self, name: str):
        super().__init__(name)
