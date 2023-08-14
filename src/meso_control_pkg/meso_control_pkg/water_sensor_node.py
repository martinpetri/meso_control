import rclpy

import json
from typing import List, Dict

from osbk_devices.sensor_base import SensorBase
from osbk_interfaces.msg import OSBKStringValue
from meso_interfaces.msg import WaterData


class WaterSensor(SensorBase):
    """
    Node for collecting and publishing the data from the water-sensor.
    
    This node extends the ``SensorBase``-node nad uses the ``ModbusTcpNode``.

    :param last_reading: last transmitted sensor-readings from the SPS.
    :type last_reading: List[float]

    :param modbus_subscriber: subscribes to the topic where the content of the SPS-feedback-values
        are published
    :type modbus_subscriber: Subscription
    """

    def __init__(self):
        super().__init__("water_sensor", 1, WaterData)

        self.last_reading: List[float] = [0.0, 0.0, 0.0, 0.0]

        self.modbus_subscriber = self.create_subscription(OSBKStringValue,
                                                          "modbus_tcp_node/read",
                                                          self.modbus_handle,
                                                          10)
    
    def read_sensor(self):
        """
        Return the last sensor-reading as ROS-message to be published.
        """
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
        """
        Subscription-callback to store the transmitted sensor-values.
        """
        modbus_msg: Dict = json.loads(msg.data)
        try:
            self.last_reading = [
                float(modbus_msg["RO_TEMP"]) / 100.0,
                float(modbus_msg["RO_COND"]) / 100.0,
                float(modbus_msg["RO_OXYGEN"]) / 100.0,
                float(modbus_msg["RO_PH"]) / 100.0
            ]
        except(KeyError):
            pass


def main():
    rclpy.init()
    water_sensor_node = WaterSensor()
    try:
        rclpy.spin(water_sensor_node)
    except(KeyboardInterrupt):
        water_sensor_node.get_logger().info("Shutting down.")

    water_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
