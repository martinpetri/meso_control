from rclpy.node import Node
from awi_interfaces import AWIFloatValue
from abc import ABC, abstractmethod


class SensorBase(Node, ABC):

    def __init__(self, name, msg_interface=AWIFloatValue):
        super().__init__(name)

        self.publish_topic = f'{name}/value'
        self.msg_interface = msg_interface
        self.publisher = self.create_publisher(msg_interface,
                                               self.publish_topic,
                                               10)

    def publish_reading(self):
        msg = self.read_sensor()
        self.publisher.publish(msg)

    @abstractmethod
    def read_sensor():
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
