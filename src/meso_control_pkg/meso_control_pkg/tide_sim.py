import rclpy
from rclpy.node import Node

import requests

from osbk_interfaces.srv import ContinuousActuatorControl, DiscreteActuatorControl
from osbk_interfaces.msg import OSBKInt32Value


REAL_MAX = 589.0
REAL_MIN = 409.0

MESO_MAX = 7.80
MESO_MIN = 0


class TideSim(Node):
    """Node for controlling the tide platform based on real tide levels."""

    def __init__(self):
        super().__init__("tide_sim")

        self.declare_parameter("update_interval", 60)
        self.declare_parameter(
            "request_url",
            "https://www.pegelonline.wsv.de/webservices/rest-api/v2/stations/LIST%20AUF%20SYLT/" +
            "W/measurements.json?start=P0DT0H30M"
        )
        
        self.tide_a_client = self.create_client(ContinuousActuatorControl,
                                                "tide_control_a/control")
        self.tide_b_client = self.create_client(ContinuousActuatorControl,
                                                "tide_control_b/control")
        
        self.mode_publisher_a = self.create_publisher(OSBKInt32Value,
                                                    f"/{self.get_name()}/auto_tide_current_a",
                                                    10)
        
        self.mode_service_a = self.create_service(DiscreteActuatorControl,
                                                f"/{self.get_name()}/set_auto_tide_a",
                                                self.set_auto_tide_a)
        self.auto_tide_setting_a: bool = True

        self.mode_publisher_b = self.create_publisher(OSBKInt32Value,
                                                    f"/{self.get_name()}/auto_tide_current_b",
                                                    10)
        
        self.mode_service_b = self.create_service(DiscreteActuatorControl,
                                                f"/{self.get_name()}/set_auto_tide_b",
                                                self.set_auto_tide_b)
        self.auto_tide_setting_b: bool = True
        
        self.update_timer = self.create_timer(self.get_parameter("update_interval").value,
                                              self.update)
        
        self.mode_publish_timer = self.create_timer(5.0, self.publish_mode)

        self.update()

    def update(self):
        if self.auto_tide_setting_a or self.auto_tide_setting_b:
            response = requests.get(self.get_parameter("request_url").value)
            response = response.json()
            if len(response) != 0:
                real_val = float(response[-1]["value"])

                meso_val = (real_val - REAL_MIN) / 2.3;
                meso_val = meso_val / 10;
                meso_val = round(meso_val, 2);

                if meso_val > MESO_MAX:
                    meso_val = MESO_MAX;
                elif meso_val < MESO_MIN:
                    meso_val = MESO_MIN;

                request = ContinuousActuatorControl.Request()
                request.new_status = meso_val
                if self.auto_tide_setting_a:
                    while not self.tide_a_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting...')
                    self.tide_a_client.call_async(request)

                if self.auto_tide_setting_b:
                    while not self.tide_b_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting...')
                    self.tide_b_client.call_async(request)

        self.update_timer.reset()
    
    def set_auto_tide_a(self,
                      request: DiscreteActuatorControl.Request,
                      response: DiscreteActuatorControl.Response
                      ) -> DiscreteActuatorControl.Response:
        self.auto_tide_setting_a = request.new_status > 0
        response.requested_status = int(self.auto_tide_setting_a)
        self.update()
        return response
    
    def set_auto_tide_b(self,
                      request: DiscreteActuatorControl.Request,
                      response: DiscreteActuatorControl.Response
                      ) -> DiscreteActuatorControl.Response:
        self.auto_tide_setting_b = request.new_status > 0
        response.requested_status = int(self.auto_tide_setting_b)
        self.update()
        return response
    
    def publish_mode(self):
        current_state_a = OSBKInt32Value()
        current_state_a.topic_name = self.mode_publisher_a.topic
        current_state_a.data = int(self.auto_tide_setting_a)
        current_state_a.unit = "Auto-Tide-Mode"

        current_state_b = OSBKInt32Value()
        current_state_b.topic_name = self.mode_publisher_b.topic
        current_state_b.data = int(self.auto_tide_setting_b)
        current_state_b.unit = "Auto-Tide-Mode"

        self.mode_publisher_a.publish(current_state_a)
        self.mode_publisher_b.publish(current_state_b)


def main():
    rclpy.init()
    tide_sim_node = TideSim()
    try:
        rclpy.spin(tide_sim_node)
    except(KeyboardInterrupt):
        tide_sim_node.get_logger().info("Shutting down.")

    tide_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
