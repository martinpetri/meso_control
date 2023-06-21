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


condition_0_to_1 = False
condition_0_to_2 = False
condition_1_to_2 = False
condition_2_to_1 = False
condition_1_to_3 = False
condition_2_to_3 = False


def action():
    global flag
    flag = not flag


def create_state_machine():
    
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


def main():
    rclpy.init()

    state_machine_node = create_state_machine()
    state_machine_node._change_state(state_machine_node.states[1])
    state_machine_node.get_logger().warn(state_machine_node.current_state.name)
    state_machine_node._change_state(state_machine_node.states[2])
    state_machine_node.get_logger().warn(state_machine_node.current_state.name)
    state_machine_node._change_state(state_machine_node.states[3])
    state_machine_node.get_logger().warn(state_machine_node.current_state.name)
    # rclpy.spin(state_machine_node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
