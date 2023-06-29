from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import FrontendLaunchDescriptionSource

import json

MODBUS_READ_TOPIC = "modbus_tcp_node/read"
MODBUS_WRITE_SERVICE = "modbus_tcp_node/write"

JSON_MODBUS_WRITE_REGISTERS = json.dumps({
    "V1":32001,
    "V2":32002,
    "V3":32003,
    "V4":32004,
    "PUMP":32005,
    "UPPER_MAGNETIC_VALVE":32006,
    "LOWER_MAGNETIC_VALVE":32007,
    "HEAT_A":32008,
    "HEAT_B":32009,
    "COOL_A":32010,
    "COOL_B":32011,
    "TIDE_A":32012,
    "TIDE_B":32013,
    "TEMP_SETPOINT_A":32014,
    "TEMP_SETPOINT_B":32015,
    "SPARE_16":32016,
    "SPARE_17":32017,
    "SPARE_18":32018,
    "SPARE_19":32019,
    "SPARE_20":32020,
    "SPARE_21":32021,
    "SPARE_22":32022,
    "SPARE_23":32023,
    "SPARE_24":32024,
    "SPARE_25":32025,
    "SPARE_26":32026,
    "SPARE_27":32027,
    "SPARE_28":32028,
    "SPARE_29":32029,
    "HEARTBEAT":32030,
    "LAST_STEP_NUMBER":32031,
    "MANUAL_CONTROL":32032
})

JSON_MODBUS_FEEDBACK_REGISTERS = json.dumps({
    "V1":32033,
    "V2":32034,
    "V3":32035,
    "V4":32036,
    "PUMP":32037,
    "UPPER_MAGNETIC_VALVE":32038,
    "LOWER_MAGNETIC_VALVE":32039,
    "HEAT_A":32040,
    "HEAT_B":32041,
    "COOL_A":32042,
    "COOL_B":32043,
    "TIDE_A":32044,
    "TIDE_B":32045,
    "TEMP_SETPOINT_A":32046,
    "TEMP_SETPOINT_B":32047,
    "SPARE_16":32048,
    "SPARE_17":32049,
    "SPARE_18":32050,
    "LAST_STEP_NUMBER":32051,
    "MANUAL_CONTROL":32052,
    "RO_TEMP":32053,
    "RO_PH":32054,
    "RO_COND":32055,
    "RO_OXYGEN":32056,
    "RO_SPARE_25":32057,
    "RO_SPARE_26":32058,
    "RO_SPARE_27":32059,
    "RO_SPARE_28":32060,
    "RO_SPARE_29":32061,
    "RO_SPARE_30":32062,
    "RO_SPARE_31":32063,
    "RO_SPARE_32":32064
})

def generate_launch_description():
    return LaunchDescription([
        Node(
                executable='modbus_tcp_node',
                respawn='true',
                name='modbus_tcp_node',
                package='meso_control_pkg',
                arguments=[('--ros-args --log-level info')],
                parameters=[
                        {"name": 'modbus_tcp_node'},
                        {"topic_name": MODBUS_READ_TOPIC},
                        {"modbus_host_ip": 'localhost'},
                        {"modbus_host_port": 5020},
                        {"modbus_service_name": MODBUS_WRITE_SERVICE},
                        {"read_modbus_interval": 1},
                        {"json_modbus_write_registers": JSON_MODBUS_WRITE_REGISTERS},
                        {"json_modbus_feedback_registers": JSON_MODBUS_FEEDBACK_REGISTERS}
                ]
        ),
        # Node(
        #     package='meso_control_pkg',
        #     executable='mock_publisher'
        # ),
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
        ),
        Node(
            package='meso_control_pkg',
            executable='tide_sim'
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource([
                FindPackageShare("rosbridge_server"), '/launch', '/rosbridge_websocket_launch.xml'
            ])
        )
    ])