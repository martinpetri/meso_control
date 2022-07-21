# meso-control launch file

from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC_MODBUS_VALUES = "/meso/json_modbus_values"
MODBUS_SERVICE_NAME = "/meso/modbus_tcp_node"

JSON_MODBUS_WRITE_REGISTERS =   '{      "heartbeat": 32016, \
                                        "temp_control_A": 32017, "temp_setpoint_A": 32008, "temp_hysteresis_A": 32010, \
                                        "temp_control_B": 32018, "temp_setpoint_B": 32009, "temp_hysteresis_B": 32011, \
                                        "v1":32000, "v2":32001, "v3":32002, "v4":32003, "v5":32004, "v6":32005, "v7":32006, "p1":32007 }'


# JSON_MODBUS_FEEDBACK_REGISTERS = '{     "temp_control_A": 32017, "temp_setpoint_A": 32008, "temp_hysteresis_A": 32010, \
#                                         "temp_control_B": 32018, "temp_setpoint_B": 32009, "temp_hysteresis_B": 32011, \
#                                         "v1":32000, "v2":32001, "v3":32002, "v4":32003, "v5":32004, "v6":32005, "v7":32006, "p1":32007, \
#                                         "mps_temp": 7, "mps_ph": 8, "mps_cond": 9, "mps_o2": 10, \
#                                         "status": 12, "status_timer": 13, "last_tank": 18, \
#                                         "cooler_tank_A": 14, "heater_tank_A": 15, \
#                                         "cooler_tank_B": 16, "heater_tank_B": 17 }'

JSON_MODBUS_FEEDBACK_REGISTERS = '{     "temp_control_A": 32017, "temp_setpoint_A": 32008, "temp_hysteresis_A": 32010, \
                                        "temp_control_B": 32018, "temp_setpoint_B": 32009, "temp_hysteresis_B": 32011, \
                                        "v1":1, "v2":2, "v3":3, "v4":4, "v5":32004, "v6":32005, "v7":32006,  "p1":32007, \
                                        "mps_temp": 7, "mps_ph": 8, "mps_cond": 9, "mps_o2": 10, \
                                        "status": 12, "status_timer": 13, "last_tank": 18, \
                                        "cooler_tank_A": 14, "heater_tank_A": 15, \
                                        "cooler_tank_B": 16, "heater_tank_B": 17 }'

def generate_launch_description():
    return LaunchDescription([
        Node(
                executable='simple_control_node',
                respawn='false',
                name='simple_control_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name":"control"},
                        {"temp_setpoint_A": 20.0},
                        {"temp_hysteresis_A": 0.1},
                        {"temp_setpoint_B": 20.0},
                        {"temp_hysteresis_B": 0.1},
                        {"temp_check_interval": 10},
                        {"tank_switch_interval": 1800},
                        {"empty_duration": 20},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES}
                ]                  
        ),
        Node(
                executable='modbus_tcp_node',
                respawn='false',
                name='modbus_tcp_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name": 'modbus_tcp_node'},
                        {"topic_name": 'json_modbus_values'},
                        {"modbus_host_ip": '192.168.51.102'},
                        {"modbus_host_port": 502},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"read_modbus_interval": 1},
                        {"json_modbus_write_registers": JSON_MODBUS_WRITE_REGISTERS},
                        {"json_modbus_feedback_registers": JSON_MODBUS_FEEDBACK_REGISTERS}
                ]
        ),
    ])