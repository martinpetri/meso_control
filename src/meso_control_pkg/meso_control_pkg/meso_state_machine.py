import rclpy

import json
from functools import partial

from osbk_interfaces.srv import DiscreteActuatorControl, Modbus
from osbk_interfaces.msg import DiscreteActuatorState, OSBKStringValue, OSBKInt32Value
from osbk_operation.utility import State, Transition
from osbk_operation.actuator_state_machine import (
    ActuatorEntry,
    ActuatorStateMachine
)
from meso_interfaces.srv import ChangeOperatingMode


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
        "upper_magnetic_valve": REQUEST_1,
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
        "upper_magnetic_valve": REQUEST_1,
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

SECOND = 1
MINUTE = 60
STATE_TIMES = [
    5  * SECOND, # STOP
    20 * SECOND, # PREP_TANK_A
    20 * SECOND, # VENT_TANK_A
    30 * MINUTE, # RUN_TANK_A
    10 * SECOND, # STOP_TANK_A
    30 * SECOND, # DRAIN_TANK_A
    20 * SECOND, # PREP_TANK_B
    20 * SECOND, # VENT_TANK_B
    30 * MINUTE, # RUN_TANK_B
    10 * SECOND, # STOP_TANK_B
    30 * SECOND, # DRAIN_TANK_B
]


MODE_LIST = [
    "AUTOMATIC_CYCLE",
    "MEASURE_TANK_A",
    "MEASURE_TANK_B",
    "DRAIN_SENSOR",
    "STOP"
]


class MesoStateMachine(ActuatorStateMachine):
    """
    A node that implements the statemachine for controlling a mesocosm-tankpair.

    This node extends the ``ActuatorStateMachine``-node.

    :param step_publisher: Publishes the current state-number.
    :type step_publisher: Publisher

    :param modbus_write_client: Service-client to write the current state-number to the SPS via
        the ``ModbusTcpNode``.
    :type modbus_write_client: Client

    :param change_mode_service: A service to change the operating mode (automatic cylce, measure
        tank a, measure tank b, drain, stop) of this statemachine.
    :type change_mode_service: Service

    :param current_mode_publisher: Publishes the currently active operating mode.
    :type current_mode_publisher: Publisher

    :param publish_status_timer: The timer to trigger the publishing of the active operating mode.
    :type publish_status_timer: Timer

    :param current_mode: A string to represent the currently active operating mode.
    :type current_mode: str

    :param last_step_number: The index of the currently active state.
    :type last_step_number: int
    """
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
                action = partial(self._set_last_step_number, 1)
                
        ))
        # add shortcut from DRAIN_TANK_A to PREP_TANK_A for MEASURE_TANK_A mode
        transitions.append(Transition(
            states[5],
            states[1],
            False,
            condition = lambda: False,
            action = partial(self._set_last_step_number, 1)
        ))
        # add shortcut from DRAIN_TANK_B to PREP_TANK_B for MEASURE_TANK_B mode
        transitions.append(Transition(
            states[10],
            states[6],
            False,
            condition = lambda: False,
            action = partial(self._set_last_step_number, 6)
        ))

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
        
        self.step_publisher = self.create_publisher(OSBKInt32Value,
                                                    f"/{self.get_name()}/current_step",
                                                    10)
        self.modbus_write_client = self.create_client(Modbus, "/modbus_tcp_node/write")
        self._starting_state_subscription = self.create_subscription(OSBKStringValue,
                                                                 "/modbus_tcp_node/read",
                                                                 self._set_starting_state,
                                                                 10)
        self.change_mode_service = self.create_service(ChangeOperatingMode,
                                                       f"/{self.get_name()}/mode",
                                                       self._change_mode)
        
        self.current_mode_publisher = self.create_publisher(OSBKStringValue,
                                                            f"/{self.get_name()}/current_mode",
                                                            10)
        self.publish_status_timer = self.create_timer(5.0, self._publish_status)
        self.current_mode = "AUTOMATIC_CYCLE"
        self.last_step_number = 0

        self.get_logger().info("Initialized statemachine.")

    def _set_starting_state(self, msg: OSBKStringValue) -> None:
        sps_data = json.loads(msg.data)
        if "LAST_STEP_NUMBER" in sps_data:
            self.last_step_number = int(sps_data["LAST_STEP_NUMBER"])
            starting_state = self.states[self.last_step_number]
            self._change_state(starting_state)
            self._starting_state_subscription.destroy()
            self._reset()
            self.get_logger().info(f"Starting execution in state {self.current_state.name}.")

    def _set_last_step_number(self, number: int) -> None:
        request = Modbus.Request()
        request.key_name = "LAST_STEP_NUMBER"
        request.value_to_send = str(number)
        self.last_step_number = number

        self.modbus_write_client.call_async(request)
    
    def _change_mode(self,
                     request: ChangeOperatingMode.Request,
                     response: ChangeOperatingMode.Response
                     ) -> ChangeOperatingMode.Response:
        response.requested_mode = request.mode
        if request.mode in MODE_LIST:
            self.current_mode = request.mode
            if request.mode == "AUTOMATIC_CYCLE":
                # remove shortcuts
                self.transitions[3].condition = lambda: False
                self.transitions[8].condition = lambda: False
                self.transitions[11].condition = lambda: False
                self.transitions[12].condition = lambda: False
                # activate all transitions
                for idx, transition in enumerate(self.transitions):
                    transition.active = True
            elif request.mode == "MEASURE_TANK_A":
                # shortcut for transition out of RUN_TANK_B
                self.transitions[8].condition = lambda: True
                # shortcut from DRAIN_TANK_A to PREP_TANK_A
                self.transitions[11].condition = lambda: True
                # remove other shortcuts
                self.transitions[3].condition = lambda: False
                self.transitions[12].condition = lambda: False
                # deactivate transition out of RUN_TANK_A
                # activate other transitions
                for idx, transition in enumerate(self.transitions):
                    transition.active = idx != 3
            elif request.mode == "MEASURE_TANK_B":
                # shortcut for transition out of RUN_TANK_A
                self.transitions[3].condition = lambda: True
                # shortcut from DRAIN_TANK_B to PREP_TANK_B
                self.transitions[12].condition = lambda: True
                # remove other shortcuts
                self.transitions[8].condition = lambda: False
                self.transitions[11].condition = lambda: False
                # deactivate transition out of RUN_TANK_B
                # activate other transitions
                for idx, transition in enumerate(self.transitions):
                    transition.active = idx != 8
            elif request.mode == "DRAIN_SENSOR":
                # shortcuts for transitions out of RUN_TANK_A, RUN_TANK_B
                self.transitions[3].condition = lambda: True
                self.transitions[8].condition = lambda: True
                # deactivate other shortcuts out of DRAIN_SENSOR
                self.transitions[11].condition = lambda: False
                self.transitions[12].condition = lambda: False
                # deactivate transition out of DRAIN_SENSOR_A or DRAIN_SENSOR_B
                # activate other transitions
                for idx, transition in enumerate(self.transitions):
                    transition.active = (idx != 5 and idx != 10)
            elif request.mode == "STOP":
                # shortcuts for transitions out of RUN_TANK_A, RUN_TANK_B
                self.transitions[3].condition = lambda: True
                self.transitions[8].condition = lambda: True
                # deactivate other shortcuts out of DRAIN_SENSOR
                self.transitions[11].condition = lambda: False
                self.transitions[12].condition = lambda: False
                # deactivate transition out of STOP_SENSOR_A or STOP_SENSOR_B
                # activate other transitions
                for idx, transition in enumerate(self.transitions):
                    transition.active = (idx != 4 and idx != 9)
        return response

    def _publish_status(self):
        msg = OSBKStringValue()
        msg.topic_name = self.current_mode_publisher.topic
        msg.data = self.current_mode
        msg.unit = "Operating mode"
        self.current_mode_publisher.publish(msg)

        msg = OSBKInt32Value()
        msg.topic_name = self.step_publisher.topic
        msg.data = self.last_step_number
        msg.unit = "Stepnumber"
        self.step_publisher.publish(msg)

        self.publish_status_timer.reset()


def main():
    rclpy.init()

    state_machine_node = MesoStateMachine()
    try:
        rclpy.spin(state_machine_node)
    except(KeyboardInterrupt):
        state_machine_node.get_logger().info("Shutting down.")

    state_machine_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
