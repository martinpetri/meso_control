from typing import List

from .transition import Transition


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
