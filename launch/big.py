# meso-control launch file

from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC_MODBUS_VALUES = "/meso/json_modbus_values"
MODBUS_SERVICE_NAME = "/meso/modbus_tcp_node"
JSON_MODBUS_REGISTERS = '{"v1":"32000", "v2":"32001", "v3":"32002", "v4":"32003", "v5":"32004", "v6":"32005", "p1":"32007"}'

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='meso_control_pkg',
        #     namespace='meso',
        #     executable='web_node',
        #     name='web',
        #     parameters=[
        #         {"value_store_url_values": "http://134.1.4.32/api/v1/values"},
        #         {"value_store_url_topics": "http://134.1.4.32/api/v1/topics"},
        #         {"topics": ["/rsv/flow_average", "/rsv/valve_position"]}
        #     ]
        # ),
        Node(
                executable='sensor_string_val_node', name='multi_param', package='meso_control_pkg', namespace='meso',
                parameters=[
                        {"name":"multi_param"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "multiparams"},
                ]
        ),
        Node(   executable='actuator_node', name='v1', package='meso_control_pkg', namespace='meso',
                parameters=[
                        {"name":"v1"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v1"},
                ]
        ),
        Node(   executable='actuator_node', name='v2', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"v2"},
                        {"poll_interval": 5},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v2"},
                ]
        ),
        Node(   executable='actuator_node', name='v3', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"v3"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v3"},
                ]

        ),
        Node(   executable='actuator_node', name='v4', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"v4"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v4"},
                ]
        ),
        Node(   executable='actuator_node', name='v5', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"v5"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v5"},
                ]
        ),
        Node(   executable='actuator_node', name='v6', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"v6"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "v6"},
                ]
        ),
        Node(   executable='actuator_node', name='p1', package='meso_control_pkg', namespace='meso',
                parameters=[{"name":"p1"},
                        {"poll_interval": 5},
                        {"publish_interval": 5},
                        {"topic_modbus_values": TOPIC_MODBUS_VALUES},
                        {"modbus_service_name": MODBUS_SERVICE_NAME},
                        {"modbus_register_key": "p1"},
                ]
        ),
        Node(
                executable='big_control_node',
                name='big_control_node',
                package='meso_control_pkg',
                namespace='meso',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name":"big_control"},
                        {"temp_setpoint": 12.0},
                        {"temp_hysteresis": 1},
                        {"temp_check_interval": 2},
                        {"tank_switch_interval": 15},
                        {"flush_duration": 5},
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