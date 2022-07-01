# meso-control launch file

from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC_MODBUS_VALUES = "/meso/json_modbus_values"
MODBUS_SERVICE_REGISTER_NAME = "/meso/modbus_register_json"
MODBUS_SERVICE_JSON_NAME = "/meso/modbus_service_json"
JSON_MODBUS_WRITE_REGISTERS = '{"v1":"32000", "v2":"32001", "v3":"32002", "v4":"32003", "v5":"32004", "v6":"32005", "v7":"32006", "p1":"32007"}'
JSON_MODBUS_FEEDBACK_REGISTERS = '{"v1":"1", "v2":"2", "v3":"3", "v4":"4", "v5":"32004", "v6":"32005", "v7":"32006", "p1":"32007"}'

def generate_launch_description():
    return LaunchDescription([
        Node(
                executable='small_control_node',
                respawn='false',
                name='small_control_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name":"small_control"},
                        {"temp_setpoint": 12.0},
                        {"temp_hysteresis": 1},
                        {"temp_check_interval": 2},
                        {"tank_switch_interval": 15},
                        {"flush_duration": 5},
                        {"modbus_service_register_name": MODBUS_SERVICE_REGISTER_NAME},
                        {"modbus_service_json_name": MODBUS_SERVICE_JSON_NAME},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"json_modbus_write_registers": JSON_MODBUS_WRITE_REGISTERS}
                ]                  
        ),
        Node(
                executable='modbus_tcp_node',
                respawn='true',
                name='modbus_tcp_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name": 'modbus_tcp_node'},
                        {"topic_name": 'json_modbus_values'},
                        {"modbus_host_ip": '192.168.178.51'},
                        {"modbus_host_port": 502},
                        {"modbus_service_register_name": MODBUS_SERVICE_REGISTER_NAME},
                        {"modbus_service_json_name": MODBUS_SERVICE_JSON_NAME},
                        {"read_modbus_interval": 1},
                        {"json_modbus_write_registers": JSON_MODBUS_WRITE_REGISTERS},
                        {"json_modbus_feedback_registers": JSON_MODBUS_FEEDBACK_REGISTERS}
                ]
        ),
    ])