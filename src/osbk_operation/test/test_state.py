import pytest

from osbk_operation.utility import State, Transition


def true_condition():
    return True


def false_condition():
    return False


@pytest.fixture
def state_obj():
    return State("test_state", {})


def test_object_creation():
    obj = State("test_state", {})

    assert obj is not None


def test_check_exit_conditions(state_obj: State):
    false_transition = Transition(state_obj, state_obj, False, -1, false_condition)
    true_transition = Transition(state_obj, state_obj, False, -1, true_condition)

    assert state_obj.possible_transitions[0] is false_transition
    assert state_obj.possible_transitions[1] is true_transition
    assert state_obj.check_exit_conditions() is true_transition
