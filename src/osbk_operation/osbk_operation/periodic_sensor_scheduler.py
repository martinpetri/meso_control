from rclpy.node import Node
from rclpy.timer import Timer
from osbk_devices.sensor_base import SensorBase


class SensorSchedEntry:
    """
    Holds a sensor-object and its corresponding timer for scheduling.

    :param sensor: the sensor-node of this entry
    :type sensor: osbk_devices.SensorBase
    :param timer: the corresponding ROS-timer
    :type timer: Timer
    """

    def __init__(self, sensor: SensorBase, timer: Timer) -> None:
        """
        Construct an Entry for the sensor-scheduler.

        :param sensor: the sensor-object
        :type sensor: osbk_devices.SensorBase
        :param timer: the ROS-timer-object
        :type timer: Timer
        """
        self.sensor = sensor
        self.timer = timer


class PeriodicSensorScheduler(Node):
    """
    Handles periodic reading and publishing of Sensors.

    :param sensor_entries: list of sensors to schedule
    :type sensor_entries: list[SensorSchedEntry]
    """

    def __init__(self) -> None:
        """Construct instance of PeriodicSensorScheduler."""
        # call the constructor of Node
        super().__init__('sensor_scheduler')

        # initalize list for sensors
        self.sensor_entries: list[SensorSchedEntry] = []

    def add_sensor(self,
                   sensor_to_add: SensorBase,
                   read_interval: float) -> None:
        """
        Add a sensor to the scheduler to publish its readings periodically.

        Creates a timer with the given interval and the sensors
        'publish_reading()' method as its callback-function. Adds the
        sensor-node and its timer to the 'sensor_entries' list.

        :param sensor_to_add: sensor-node to read periodically.
        :type sensor_to_add: osbk_devices.SensorBase
        :param read_interval: time-interval in seconds for reading the sensor
        :type read_interval: float
        """
        # create timer with sensor.publish_reading() as callback
        timer = self.create_timer(read_interval, sensor_to_add.publish_reading)

        # create entry-object for scheduler
        entry = SensorSchedEntry(sensor_to_add, timer)

        # add it to the list of sensors to schedule
        self.sensor_entries.append(entry)
