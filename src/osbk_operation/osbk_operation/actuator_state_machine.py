from rclpy.node import Node
from rclpy.client import Client
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from abc import ABC
from typing import List

from .state import State
from .transition import Transition


class ActuatorEntry():
    """A type that bundles the necessary information to control an actuator."""

    def __init__(self,
                 service: Client,
                 topic: Subscription):
        self.service: Client = service
        self.topic: Subscription = topic
        self.current_state = None


class ActuatorStateMachine(Node, ABC):
    """A node that manages actuators following a finite statemachine."""

    def __init__(self,
                 name: str,
                 states: List[State],
                 initial_state: State,
                 transitions: List[Transition],
                 actuators: List[ActuatorEntry],
                 update_interval: int = 1000):
        super().__init__(name)

        # initialize the states that are part of this statemachine
        self.states: List[State] = states
        if(initial_state in states):
            self.initial_state: State = initial_state
            self.current_state = initial_state
        else:
            raise ValueError('The initial_state must be part of the list states')

        # initialize the transitions between these states
        self.transitions: List[Transition] = transitions

        # and the control-services and status-topics of each actuator

        self.actuators: List[ActuatorEntry] = actuators

        # set a timer to check the current state
        self.update_interval: int = update_interval
        self.update_timer: Timer = self.create_timer(self.update_interval,
                                                     self._update)

    def _update(self):
        """Check if actuators are in the correct state or a transition should be taken."""
        for actuator in self.actuators:
            pass
