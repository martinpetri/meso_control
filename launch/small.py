# meso-control launch file

from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC_MODBUS_VALUES = "/meso/json_modbus_values"
MODBUS_SERVICE_NAME = "/meso/modbus_tcp_node"
JSON_MODBUS_REGISTERS = '{"v1":"32000", "v2":"32000", "v3":"32000", "v4":"32000", "v5":"32000", "v6":"32000", "p1":"32000"}'

def generate_launch_description():
    return LaunchDescription([
        Node(
                executable='small_control_node',
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
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"json_modbus_registers": JSON_MODBUS_REGISTERS}
                ]                  
        ),
        Node(
                executable='modbus_tcp_node',
                name='modbus_tcp_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name": 'modbus_tcp_node'},
                        {"topic_name": 'json_modbus_values'},
                        {"modbus_host_ip": '192.168.178.51'},
                        {"modbus_host_port": 502},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"read_modbus_interval": 1},
                        {"json_modbus_registers": JSON_MODBUS_REGISTERS}
                ]
        ),
    ])