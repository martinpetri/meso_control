import rclpy
from rclpy.node import Node
from awi_interfaces.srv import ActuatorControl


class ActuatorBase(Node):

    def __init__(self, name='actuator'):
        super().__init__(name)

        self.service_name = f'{name}/control'
        self.srv = self.create_service(ActuatorControl,
                                       self.service_name,
                                       self.command_callback)

    def command_callback(self, request, response):
        response = self.set_actuator(request)
        return response

    def set_actuator(self, setpoint):
        return None


def main(args=None):
    rclpy.init(args=args)

    actuator = ActuatorBase()

    rclpy.spin(actuator)

    actuator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
