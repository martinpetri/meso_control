from rclpy.node import Node
from rclpy.client import Client

from abc import ABC
from typing import List

from .state import State
from .transition import Transition


class ActuatorStateMachine(Node, ABC):
    """A node that manages actuators following a finite statemachine."""

    def __init__(self,
                 name: str,
                 states: List[State],
                 initial_state: State,
                 transitions: List[Transition],
                 actuator_services: List[Client]):
        super().__init__(name)

        self.states: List[State] = states
        if(initial_state in states):
            self.initial_state: State = initial_state
            self.current_state = initial_state
        else:
            raise ValueError('The initial_state must be part of the list states')
        self.transitions: List[Transition] = transitions
        self.actuator_services: List[Client] = actuator_services
