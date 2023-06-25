import rclpy

import json
from functools import partial

from osbk_interfaces.srv import DiscreteActuatorControl, Modbus
from osbk_interfaces.msg import DiscreteActuatorState, OSBKStringValue
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)


REQUEST_0 = DiscreteActuatorControl.Request()
REQUEST_0.new_status = 0

REQUEST_1 = DiscreteActuatorControl.Request()
REQUEST_1.new_status = 1


STATE_NAMES = [
    "STOP",
    "PREP_TANK_A",
    "VENT_TANK_A",
    "RUN_TANK_A",
    "STOP_TANK_A",
    "DRAIN_TANK_A",
    "PREP_TANK_B",
    "VENT_TANK_B",
    "RUN_TANK_B",
    "STOP_TANK_B",
    "DRAIN_TANK_B"
]


SETPOINTS = [
    {# STOP
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_0
    },
    {# PREP_TANK_A
        "valve_tank_a": REQUEST_1,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_0
    },
    {# VENT_TANK_A
        "valve_tank_a": REQUEST_1,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_1,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_1
    },
    {# RUN_TANK_A
        "valve_tank_a": REQUEST_1,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_1
    },
    {# STOP_TANK_A
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_0
    },
    {# DRAIN_TANK_A
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_1,
        "lower_magnetic_valve": REQUEST_1,
        "pump": REQUEST_0
    },
    {# PREP_TANK_B
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_1,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_0
    },
    {# VENT_TANK_B
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_1,
        "upper_magnetic_valve": REQUEST_1,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_1
    },
    {# RUN_TANK_B
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_1,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_1
    },
    {# STOP_TANK_B
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_0,
        "lower_magnetic_valve": REQUEST_0,
        "pump": REQUEST_0
    },
    {# DRAIN_TANK_B
        "valve_tank_a": REQUEST_0,
        "valve_tank_b": REQUEST_0,
        "upper_magnetic_valve": REQUEST_1,
        "lower_magnetic_valve": REQUEST_1,
        "pump": REQUEST_0
    }
]


ACTUATOR_NODE_NAMES = [
    "valve_tank_a",
    "valve_tank_b",
    "upper_magnetic_valve",
    "lower_magnetic_valve",
    "pump"
]


STATE_TIMES = [
    5, # STOP
    5, # PREP_TANK_A
    5, # VENT_TANK_A
    10, # RUN_TANK_A
    5, # STOP_TANK_A
    5, # DRAIN_TANK_A
    5, # PREP_TANK_B
    5, # VENT_TANK_B
    10, # RUN_TANK_B
    5, # STOP_TANK_B
    5, # DRAIN_TANK_B
]


class MesoStateMachine(ActuatorStateMachine):
    def __init__(self) -> None:
        states = [
            State(STATE_NAMES[idx], setpoint, False)
            for idx, setpoint in enumerate(SETPOINTS)
        ]

        transitions = [
            Transition(
                # from state[idx] to state[idx+1]
                states[idx],
                states[idx+1],
                True,
                STATE_TIMES[idx],
                action = partial(self._set_last_step_number, idx+1))
            for idx in range(len(STATE_NAMES) - 1)
        ]
        # add transition DRAIN_TANK_B to PREP_TANK_A
        transitions.append(Transition(
                states[-1],
                states[1],
                True,
                STATE_TIMES[-1],
                action = partial(self._set_last_step_number, 1))
        )

        actuators = [
            ActuatorEntry(name,
                        f"{name}/control",
                        DiscreteActuatorControl,
                        f"{name}/state",
                        DiscreteActuatorState)
            for name in ACTUATOR_NODE_NAMES
        ]

        super().__init__("meso_state_machine",
                         states,
                         states[0],
                         transitions,
                         actuators,
                         1)
        self._terminate()
        
        self.modbus_write_client = self.create_client(Modbus, "modbus_tcp_node/write")
        self._starting_state_subscription = self.create_subscription(OSBKStringValue,
                                                                 "modbus_tcp_node/read",
                                                                 self._set_starting_state,
                                                                 10)
        self.get_logger().info("Initialized statemachine.")

    def _set_starting_state(self, msg: OSBKStringValue) -> None:
        sps_data = json.loads(msg.data)
        starting_state = self.states[int(sps_data["LAST_STEP_NUMBER"])]
        self._change_state(starting_state)
        self._starting_state_subscription.destroy()
        self.get_logger().info(f"Starting execution in state {self.current_state.name}.")

    def _set_last_step_number(self, number: int) -> None:
        request = Modbus.Request()
        request.key_name = "LAST_STEP_NUMBER"
        request.value_to_send = str(number)

        self.modbus_write_client.call_async(request)


def main():
    rclpy.init()

    state_machine_node = MesoStateMachine()
    rclpy.spin(state_machine_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()