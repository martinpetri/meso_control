import pytest

from osbk_operation.utility import State, Transition

flag = False


def action():
    global flag
    flag = not flag


def true_condition():
    return True


def false_condition():
    return False


@pytest.fixture
def start():
    return State("start", {})


@pytest.fixture
def end():
    return State("end", {})


@pytest.fixture
def true_transition(start, end):
    return Transition(start, end, False, -1, true_condition, action)


@pytest.fixture
def false_transition(start, end):
    return Transition(start, end, False, -1, false_condition, action)


def test_object_creation(start, end):
    transition = Transition(start, end)

    assert transition is not None


def test_force_take(end, false_transition, true_transition):
    global flag

    flag = False
    assert false_transition.force_take() == end
    assert flag

    flag = False
    assert true_transition.force_take() == end
    assert flag
