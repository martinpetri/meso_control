from typing import List, Callable
from rclpy.timer import Timer


class Transition:
    pass


class State:
    """
    Represents a system-state as collection of actuator states.
    
    Designed to be used in an object of the ``ActuatorStateMachine``-class.

    :param name: a name for the state
    :type name: str

    :param actuator_states: a dictionary to map the names of ``ActuatorEntry``-objects to the
        setpoint for the actuator
    :type actuator_states: dict

    :param final: wether this state should be a finite one in the statemachine
    :type final: bool

    :param possible_transitions: a list of ``Transition``-objects which have this state as starting
        state
    :type possible_transitions: List[Transition]
    """

    def __init__(self, name: str, actuator_states: dict, final: bool = False) -> None:
        self.name: str = name
        self.actuator_states: dict = actuator_states
        self.final: bool = final
        self.possible_transitions: List[Transition] = []

    def check_exit_conditions(self) -> Transition:
        """Return the first Transition with a satisfied condition."""
        for transition in self.possible_transitions:
            if transition.condition():
                return transition
        return None


def _default_condition() -> bool:
    return False


def _default_action() -> None:
    return


class Transition():
    """
    Represents a transition between two states.
    
    Designed to be used in an ``ActuatorStateMachine``-object.

    :param start: the starting state of this transition
    :type start: State

    :param end: the state this transition leads to
    :type end: State

    :param timed: wether this Transition should be taken after a given time
    :type timed: bool

    :param time: if ``timed`` is true this controls the duration in seconds after which this
        transition is triggered
    :type time: int

    :param timer: the timer to trigger this transition. it should be created by an 
        ``ActuatorStateMachine``-node
    :type timer: Timer

    :param condition: a ``Callable``-object which should return True when the transition can be
        taken
    :type condition: Callable[[], bool]

    :param action: a ``Callable``-object which gets called when this transition gets taken
    :type action: Callable[[], None]

    :param active: a boolean to activate or deactivate this transition
    :type active: bool
    """

    def __init__(self,
                 start: State,
                 end: State,
                 timed: bool = True,
                 time: int = 5000,
                 condition: Callable[[], bool] = _default_condition,
                 action: Callable[[], None] = _default_action
                 ) -> None:
        start.possible_transitions.append(self)
        self.start: State = start
        self.end: State = end
        self.timed: bool = timed
        self.time: int = time
        self.timer: Timer = None
        self.condition: Callable[[], bool] = condition
        self.action: Callable[[], None] = action
        self.active: bool = True

    def force_take(self) -> State:
        """
        Take the transition without checking its condition.

        Executes the ``action`` and returns the end-state.

        :return: the state this transition leads to
        :rtype: State
        """
        self.action()
        return self.end

    def take(self) -> State:
        """
        Take the transition if the condition is fulfilled.

        Executes the ``action`` and returns the end-state if ``condition`` returns True.

        :return: the state this transition leads to or None
        :rtype: State
        """
        if(self.condition()):
            return self.force_take()
        else:
            return None
