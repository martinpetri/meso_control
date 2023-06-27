import rclpy
from rclpy.node import Node

import requests
import json

from osbk_interfaces.srv import ContinuousActuatorControl


REAL_MAX = 589.0
REAL_MIN = 409.0

MESO_MAX = 7.80
MESO_MIN = 0


class TideControl(Node):
    """Node for controlling the tide platform based on real tide levels."""

    def __init__(self):
        super().__init__("tide_control")

        self.declare_parameter("update_interval", 300)
        self.declare_parameter(
            "request_url",
            "https://www.pegelonline.wsv.de/webservices/rest-api/v2/stations/LIST%20AUF%20SYLT/" +
            "W/measurements.json?start=P0DT0H30M"
        )
        
        self.tide_a_client = self.create_client(ContinuousActuatorControl,
                                                "tide_control_a/control")
        self.tide_b_client = self.create_client(ContinuousActuatorControl,
                                                "tide_control_b/control")
        
        self.update_timer = self.create_timer(self.get_parameter("update_interval").value,
                                              self.update)
        self.update()

    def update(self):
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
            while not self.tide_a_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting...')
            self.tide_a_client.call_async(request)
            while not self.tide_b_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting...')
            self.tide_b_client.call_async(request)
        self.update_timer.reset()


def main():
    rclpy.init()
    tide_control_node = TideControl()
    try:
        rclpy.spin(tide_control_node)
    except(KeyboardInterrupt):
        tide_control_node.get_logger().info("Shutting down.")

    tide_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
