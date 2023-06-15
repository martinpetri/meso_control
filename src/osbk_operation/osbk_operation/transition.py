from abc import ABC, abstractmethod

from .state import State


class Transition(ABC):
    """Represents a transition between two states."""

    def __init__(self, start: State, end: State):
        start.possible_transitions.append(self)
        self.start: State = start
        self.end: State = end

    def take(self) -> State:
        if(self.condition()):
            self.action()
            return self.end
        else:
            return None

    @abstractmethod
    def action():
        pass

    @abstractmethod
    def condition() -> bool:
        pass
