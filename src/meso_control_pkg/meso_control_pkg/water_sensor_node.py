import rclpy

import json
from typing import List, Dict

from osbk_devices.sensor_base import SensorBase
from osbk_interfaces.msg import OSBKStringValue
from meso_interfaces.msg import WaterData


class WaterSensor(SensorBase):
    """Node for collecting data from the water-sensor."""

    def __init__(self):
        super().__init__("water_sensor", 1, WaterData)

        self.last_reading: List[float] = [0.0, 0.0, 0.0, 0.0]

        self.modbus_subscriber = self.create_subscription(OSBKStringValue,
                                                          "modbus_tcp_node/json_modbus_values",
                                                          self.modbus_handle,
                                                          10)
    
    def read_sensor(self):
        reading = WaterData()
        reading.topic_name = self.publish_topic
        reading.temperature = self.last_reading[0]
        reading.temperature_unit = u"\N{DEGREE SIGN}C"
        reading.salinity = self.last_reading[1]
        reading.salinity_unit = "%"
        reading.o2_concentration = self.last_reading[2]
        reading.o2_concentration_unit = "%"
        reading.acidity = self.last_reading[3]
        reading.acidity_unit = "pH"
        return reading
    
    def modbus_handle(self, msg: OSBKStringValue):
        modbus_msg: Dict = json.loads(msg.data)
        self.last_reading = [
            float(modbus_msg["RO_TEMP"]) / 100.0,
            float(modbus_msg["RO_COND"]) / 100.0,
            float(modbus_msg["RO_OXYGEN"]) / 100.0,
            float(modbus_msg["RO_PH"]) / 100.0
        ]


def main():
    rclpy.init()
    water_sensor_node = WaterSensor()
    rclpy.spin(water_sensor_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
