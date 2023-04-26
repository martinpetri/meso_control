from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from typing import TypeVar
from abc import ABC, abstractmethod

from awi_interfaces.msg import AWIFloatValue

MsgType = TypeVar('MsgType')


class SensorBase(Node, ABC):
    """
    Abstract base class for sensor implementations.

    Nodes, that implement a specific sensor directly connected to the
    controller should inherit from this base class and overwrite the
    :func:'read_sensor()' function.

    :param publish_topic: topic name, the sensors readings are published to,
        defaults to "[node_name]/value"
    :type publish_topic: str
    :param msg_interface: the msg-interface this sensor uses to publish its
        readings
    :type msg_interface: MsgType
    :param publisher: ROS publisher for sending the readings
    :type publisher: Publisher

    """

    def __init__(self,
                 name: str,
                 msg_interface: MsgType = AWIFloatValue) -> None:
        """
        Construct instance of :class:'SensorBase'.

        Initializing the nodes name and its attributes for publishing sensor
        values.
        :param name: name of the node
        :type name: str
        :param msg_interface: ROS msg-interface to use for publishing,
            defaults to 'AWIFloatValue'
        :type msg_interface: MsgType
        """
        # call the constructor of Node
        super().__init__(name)

        # initialize topic name, interface and publisher
        self.publish_topic: str = f"{name}/value"
        self.msg_interface: MsgType = msg_interface
        self.publisher: _rclpy.Publisher = self.create_publisher(
            msg_interface,
            self.publish_topic,
            10
        )

    def publish_reading(self) -> None:
        """
        Publish what :func: 'read_sensor()' returns.

        :rtype: None
        """
        msg = self.read_sensor()
        self.publisher.publish(msg)

    @abstractmethod
    def read_sensor():
        """
        Abstract method that returns a sensor reading.

        This should be overridden for specific hardware implementation.
        :return: an instance of the MsgType specified in self.msg_interface
        """
        pass


# def main(args=None):
#     rclpy.init(args=args)

#     sensor = SensorBase()

#     rclpy.spin(sensor)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     sensor.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
