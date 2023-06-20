import pytest
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from typing import TypeVar
from threading import Thread
import warnings

from osbk_interfaces.srv import DiscreteActuatorControl, ContinuousActuatorControl
from osbk_interfaces.msg import DiscreteActuatorState, ContinuousActuatorState
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)

MsgType = TypeVar('MsgType')


flag = False

condition_0_to_1 = False
condition_0_to_2 = False
condition_1_to_2 = False
condition_2_to_1 = False
condition_1_to_3 = False
condition_2_to_3 = False


def action():
    global flag
    flag = not flag


def true_condition():
    return True


def false_condition():
    return False


class Actuator(Node):
    """Simple node to mock up an actuator with a Publisher and Service."""

    def __init__(self, name: str, srv_name: str, topic: str):
        super().__init__(name)

        self.publisher = self.create_publisher(ContinuousActuatorState, topic, 10)
        self.service = self.create_service(ContinuousActuatorControl, srv_name, self._callback)

    def _callback(self,
                  request: ContinuousActuatorControl.Request,
                  response: ContinuousActuatorControl.Response
                  ) -> ContinuousActuatorControl.Response:
        warnings.warn("request received")
        response.requested_status = request.new_status
        msg = ContinuousActuatorState()
        msg.state = request.new_status
        msg.topic_name = self.publisher.topic_name
        self.publisher.publish(msg)
        return response


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


@pytest.fixture
def machine_example():
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
        rclpy.init()

    setpoints = [ContinuousActuatorControl.Request()] * 4
    for idx, setpoint in enumerate(setpoints):
        setpoint.new_status = float(idx)

    states = [
        State(f"State_{idx}", {"test_actuator": setpoint}, idx == 3)
        for idx, setpoint in enumerate(setpoints)
    ]

    transitions = [
        Transition(
            states[0],
            states[1],
            False,
            -1,
            lambda: condition_0_to_1,
            action),
        Transition(
            states[0],
            states[2],
            False,
            -1,
            lambda: condition_0_to_2,
            action),
        Transition(
            states[1],
            states[2],
            False,
            -1,
            lambda: condition_1_to_2,
            action),
        Transition(
            states[2],
            states[1],
            False,
            -1,
            lambda: condition_2_to_1,
            action),
        Transition(
            states[1],
            states[3],
            False,
            -1,
            lambda: condition_1_to_3,
            action),
        Transition(
            states[2],
            states[3],
            False,
            -1,
            lambda: condition_2_to_3,
            action)
    ]

    actuators = [
        ActuatorEntry("test_actuator",
                      "test_actuator/control",
                      ContinuousActuatorControl,
                      "test_actuator/state",
                      ContinuousActuatorState)
    ]

    return ActuatorStateMachine("testmachine",
                                states,
                                states[0],
                                transitions,
                                actuators,
                                10000)


@pytest.fixture
def mock_actuator():
    return Actuator("test_actuator",
                    "test_actuator/control",
                    "test_actuator/state")


def test_actuator_entry_object_creation():
    entry = ActuatorEntry("test",
                          "test/service",
                          DiscreteActuatorControl,
                          "test/topic",
                          DiscreteActuatorState)
    assert entry is not None


def test_object_creation(states, actuator_entries):
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
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


def test_failed_object_creation(states, actuator_entries):
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
        rclpy.init()

    transitions = [
        Transition(states[0], states[1], True, 1000, true_condition, action),
        Transition(states[0], states[0], False, -1, false_condition, action)
    ]
    with pytest.raises(ValueError):
        _ = ActuatorStateMachine("testmachine",
                                 states,
                                 State("different", {}),
                                 transitions,
                                 actuator_entries)

    rclpy.shutdown()


def test__change_state(machine_example: ActuatorStateMachine,
                       mock_actuator: Actuator):
    
    warnings.warn("test__change_state")
    assert machine_example.current_state == machine_example.initial_state
    future = Future()
    actuator_thread = Thread(target=lambda: (rclpy.spin_until_future_complete(mock_actuator, future),
                                             warnings.warn("spin ended")))
    machine_thread = Thread(target=lambda: rclpy.spin_once(machine_example))
    warnings.warn("start actuator_thread")
    actuator_thread.start()
    machine_thread.start()
    warnings.warn("execute _change_state")
    machine_example._change_state(machine_example.states[1])
    warnings.warn("executed")
    future.set_result(True)
    machine_thread.join()
    actuator_thread.join()
    warnings.warn("thread joined")

    assert machine_example.current_state == machine_example.states[1]
    rclpy.shutdown()
