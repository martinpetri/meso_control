from typing import Callable

from .state import State


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
        self.condition: Callable[[], bool] = condition
        self.action: Callable[[], None] = action

    def force_take(self) -> State:
        self.action()
        return self.end

    def take(self) -> State:
        if(self.condition()):
            return self.force_take
        else:
            return None
