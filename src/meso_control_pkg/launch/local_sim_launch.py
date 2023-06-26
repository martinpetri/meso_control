from launch import LaunchDescription
from launch_ros.actions import Node

MODBUS_READ_TOPIC = "modbus_tcp_node/read"
MODBUS_WRITE_SERVICE = "modbus_tcp_node/write"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='meso_control_pkg',
            executable='mock_publisher'
        ),
        Node(
            package='meso_control_pkg',
            executable='water_sensor_node'
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_binary_actuator_node',
            name="valve_tank_a",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "V1"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "V1"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_binary_actuator_node',
            name="valve_tank_b",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "V3"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "V3"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_binary_actuator_node',
            name="upper_magnetic_valve",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "UPPER_MAGNETIC_VALVE"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "UPPER_MAGNETIC_VALVE"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_binary_actuator_node',
            name="lower_magnetic_valve",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "LOWER_MAGNETIC_VALVE"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "LOWER_MAGNETIC_VALVE"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_binary_actuator_node',
            name="pump",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "PUMP"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "PUMP"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_continuous_actuator_node',
            name="temp_control_a",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "TEMP_SETPOINT_A"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "TEMP_SETPOINT_A"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_continuous_actuator_node',
            name="temp_control_b",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "TEMP_SETPOINT_B"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "TEMP_SETPOINT_B"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_continuous_actuator_node',
            name="tide_control_a",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "TIDE_A"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "TIDE_A"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='sps_continuous_actuator_node',
            name="tide_control_b",
            parameters=[
                        {"modbus_read_topic":MODBUS_READ_TOPIC},
                        {"read_key": "TIDE_B"},
                        {"modbus_write_service": MODBUS_WRITE_SERVICE},
                        {"write_key": "TIDE_B"}
            ]
        ),
        Node(
            package='meso_control_pkg',
            executable='meso_state_machine'
        )
    ])