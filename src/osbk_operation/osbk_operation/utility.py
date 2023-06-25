from typing import List, Callable
from rclpy.timer import Timer


class Transition:
    pass


class State:
    """Represents a system-state as collection of actuator states."""

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
    """Represents a transition between two states."""

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
        self.action()
        return self.end

    def take(self) -> State:
        if(self.condition()):
            return self.force_take()
        else:
            return None
