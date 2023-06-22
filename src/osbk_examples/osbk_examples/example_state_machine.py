import rclpy

from osbk_interfaces.srv import ContinuousActuatorControl
from osbk_interfaces.msg import ContinuousActuatorState
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)


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
    print(f"Transition, flag={flag}")


def create_state_machine():
    setpoints = []
    for i in range(4):
        request = ContinuousActuatorControl.Request()
        request.new_status = float(i)
        setpoints.append(request)

    states = [
        State(f"State_{idx}", {"example_actuator": setpoint}, idx == 3)
        for idx, setpoint in enumerate(setpoints)
    ]

    transitions = [
        Transition(
            states[0],
            states[1],
            True,
            5,
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
            True,
            5,
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
            True,
            5,
            lambda: condition_2_to_3,
            action)
    ]

    actuators = [
        ActuatorEntry("example_actuator",
                      "example_actuator/control",
                      ContinuousActuatorControl,
                      "example_actuator/state",
                      ContinuousActuatorState)
    ]

    return ActuatorStateMachine("example_machine",
                                states,
                                states[0],
                                transitions,
                                actuators,
                                1)


def main():
    rclpy.init()

    state_machine_node = create_state_machine()
    rclpy.spin(state_machine_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
