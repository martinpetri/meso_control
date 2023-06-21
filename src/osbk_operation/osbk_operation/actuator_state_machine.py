import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from typing import List, TypeVar
from functools import partial
import warnings

from .utility import State
from .utility import Transition

SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
MsgType = TypeVar('MsgType')


class ActuatorEntry():
    """A type that bundles the necessary information to control an actuator."""

    def __init__(self,
                 name: str,
                 service_name: str,
                 service_type: SrvType,
                 topic_name: str,
                 topic_type: MsgType) -> None:
        self.name: str = name

        self.service_name: str = service_name
        self.service_type: SrvType = service_type

        self.topic_name: str = topic_name
        self.topic_type: MsgType = topic_type

        self.current_state: SrvTypeRequest = None
        self.service: Client = None
        self.topic: Subscription = None

    def handle(self, msg: MsgType) -> None:
        self.current_state = msg.state


class ActuatorStateMachine(Node):
    """A node that manages actuators following a finite statemachine."""

    def __init__(self,
                 name: str,
                 states: List[State],
                 initial_state: State,
                 transitions: List[Transition],
                 actuators: List[ActuatorEntry],
                 update_interval: int = 1000):
        super().__init__(name)

        self.active: bool = True

        # initialize the states that are part of this statemachine
        self.states: List[State] = states
        if initial_state in states:
            self.initial_state: State = initial_state
            self.current_state = initial_state
        else:
            raise ValueError('The initial_state must be part of the list states')

        # initialize the transitions between these states
        self.transitions: List[Transition] = transitions
        # and create timers for timed transitions
        self.transition_timers: List[Timer] = []
        for transition in self.transitions:
            if transition.timed and transition.time >= 0:
                callback = partial(self._timed_transition_callback, transition)
                timer = self.create_timer(transition.time, callback)
                self.transition_timers.append(timer)

        # and the control-services and status-topics of each actuator
        self.actuators: List[ActuatorEntry] = actuators
        for actuator in self.actuators:
            actuator.service = self.create_client(actuator.service_type,
                                                  actuator.service_name)

            actuator.topic = self.create_subscription(actuator.topic_type,
                                                      actuator.topic_name,
                                                      actuator.handle,
                                                      10)

        # set a timer to check the current state
        self.update_interval: int = update_interval
        self.update_timer: Timer = self.create_timer(self.update_interval,
                                                     self._update)

    def _change_state(self, next_state: State):
        """Change current_state to next_state and send new setpoints to actuators."""
        self.current_state = next_state
        self._check_current_state()

    def _check_current_state(self) -> None:
        """Check if actuators are set according to current state."""
        consistent: bool = True
        for actuator in self.actuators:
            # get the current setpoint for this actuator
            setpoint: SrvTypeRequest = self.current_state.actuator_states[actuator.name]

            # compare it to its current state
            if actuator.current_state != setpoint.new_status:
                # and send setpoint again if different
                while not actuator.service.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting...')
                self.get_logger().warn("call")
                self.future = actuator.service.call_async(setpoint)
                rclpy.spin_until_future_complete(self, self.future)
                self.get_logger().warn("called")
                consistent = False

        # terminate state machine if final state is reached and actuators are set correctly
        if self.current_state.final and consistent:
            self._terminate()

    def _check_exits(self) -> None:
        """Check if a transition should be taken and take it."""
        transition = self.current_state.check_exit_conditions()
        if transition is not None:
            self._change_state(transition.take())

    def _update(self):
        """Check if actuators are in the correct state or a transition should be taken."""
        self._check_current_state()
        self._check_exits()

    def _terminate(self):
        """Terminate state machine execution."""
        self.update_timer.cancel()
        self.active = False

    def _reset(self):
        """Reset the state machine and restart."""
        self.update_timer.reset()
        self.active = False

    def _timed_transition_callback(self, transition: Transition) -> None:
        self._change_state(transition.take())
