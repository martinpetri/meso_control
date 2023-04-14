import rclpy
from rclpy.node import Node
from awi_interfaces import AWIFloatValue


class SensorBase(Node):

    def __init__(self, name):
        super().__init__(name)

        self.publish_topic = f'{name}/value'
        self.publisher = self.create_publisher(String, self.publish_topic, 10)

    def __init__(self):
        self.__init__('sensor')
        


    def publish_reading(self):
        msg = AWIFloatValue()
        msg.topic_name = self.publish_topic
        msg.data = self.read_sensor()
        msg.unit = ''
        self.publisher.publish(msg)
    
    def read_sensor():
        return 0.0


def main(args=None):
    rclpy.init(args=args)

    sensor = SensorBase()

    rclpy.spin(sensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()