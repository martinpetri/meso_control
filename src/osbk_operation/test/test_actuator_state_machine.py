import pytest
import rclpy

from osbk_interfaces.srv import DiscreteActuatorControl, ContinuousActuatorControl
from osbk_interfaces.msg import DiscreteActuatorState, ContinuousActuatorState
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)


flag = False


def action():
    global flag
    flag = not flag


def true_condition():
    return True


def false_condition():
    return False


@pytest.fixture
def states():
    return [
        State("start", {}),
        State("end", {}, True)
    ]


@pytest.fixture
def actuator_entries():
    return [
        ActuatorEntry("test",
                      "test/service",
                      DiscreteActuatorControl,
                      "test/topic",
                      DiscreteActuatorState),
        ActuatorEntry("test2",
                      "test2/service",
                      ContinuousActuatorControl,
                      "test2/topic",
                      ContinuousActuatorState)
    ]


def test_actuator_entry_object_creation():
    entry = ActuatorEntry("test",
                          "test/service",
                          DiscreteActuatorControl,
                          "test/topic",
                          DiscreteActuatorState)
    assert entry is not None


def test_object_creation(states, actuator_entries):
    rclpy.init()
    transitions = [
        Transition(states[0], states[1], False, -1, true_condition, action),
        Transition(states[0], states[0], False, -1, false_condition, action)
    ]

    statemachine = ActuatorStateMachine("testmachine",
                                        states,
                                        states[0],
                                        transitions,
                                        actuator_entries)

    assert statemachine is not None
    rclpy.shutdown()
