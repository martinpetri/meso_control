import rclpy
from rclpy.node import Node

import requests
from requests.exceptions import ConnectionError, Timeout

from osbk_interfaces.srv import DiscreteActuatorControl
from osbk_interfaces.msg import OSBKInt32Value


REAL_MAX = 589.0
REAL_MIN = 409.0

MESO_MAX = 780
MESO_MIN = 100


class TideSim(Node):
    """
    A node for controlling the tide platform based on real tide levels.
    
    This node uses pegelonline.wsv.de to retrieve current tide levels at List (Sylt)
    and sends a proportional height to the stepper-motor-control on the SPS.

    :param tide_a_client: Interacts with the node controlling the tide-platform in tank A.
    :type tide_a_client: Client

    :param tide_b_client: Interacts with the node controlling the tide-platform in tank B.
    :type tide_b_client: Client

    :param mode_publisher_a: Publishes the currently active mode for tide control in.
        tank A (automatic or manual)
    :type mode_publisher_a: Publisher

    :param mode_service_a: can be used to set the mode for tide control in tank A
    :type mode_service_a: Service

    :param auto_tide_setting_a: Wether tide level in tank A is controlled automatically.
    :type auto_tide_setting_a: bool

    :param mode_publisher_b: Publishes the currently active mode for tide control in.
        tank B
    :type mode_publisher_b: Publisher

    :param mode_service_b: Can be used to set the mode for tide control in tank B.
    :type mode_service_b: Service

    :param auto_tide_setting_b: Wether tide level in tank B is controlled automatically.
    :type auto_tide_setting_b: bool

    :param update_timer: Triggers the update of the automatic tide control.
    :type update_timer: Timer

    :param mode_publish_timer: Triggers the publishing of the currently active modes.
    :type mode_publish_timer: Timer
    """

    def __init__(self):
        super().__init__("tide_sim")

        self.declare_parameter("update_interval", 60)
        self.declare_parameter(
            "request_url",
            "https://www.pegelonline.wsv.de/webservices/rest-api/v2/stations/LIST%20AUF%20SYLT/" +
            "W/measurements.json?start=P0DT0H30M"
        )
        
        self.tide_a_client = self.create_client(DiscreteActuatorControl,
                                                "tide_control_a/control")
        self.tide_b_client = self.create_client(DiscreteActuatorControl,
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
        """
        Update the tide levels of the tanks according to real values.
        """
        if self.auto_tide_setting_a or self.auto_tide_setting_b:
            try:
                response = requests.get(self.get_parameter("request_url").value, timeout=60.0)
            except(ConnectionError):
                self.get_logger().info(f"Connection to {self.get_parameter('request_url').value} failed, trying again next time.")
                self.update_timer.reset()
                return
            except(Timeout):
                self.get_logger().info(f"Request from {self.get_parameter('request_url').value} timed out, trying again next time.")
                self.update_timer.reset()
                return
            response = response.json()
            if len(response) != 0:
                real_val = float(response[-1]["value"])

                meso_val = (real_val - REAL_MIN) / 2.3;
                meso_val = meso_val * 10;
                meso_val = round(meso_val);

                if meso_val > MESO_MAX:
                    meso_val = MESO_MAX;
                elif meso_val < MESO_MIN:
                    meso_val = MESO_MIN;

                request = DiscreteActuatorControl.Request()
                request.new_status = meso_val
                if self.auto_tide_setting_a:
                    while not self.tide_a_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('service not available, waiting...')
                    self.tide_a_client.call_async(request)

                # if self.auto_tide_setting_b:
                #     while not self.tide_b_client.wait_for_service(timeout_sec=1.0):
                #             self.get_logger().info('service not available, waiting...')
                #     self.tide_b_client.call_async(request)

        self.update_timer.reset()
    
    def set_auto_tide_a(self,
                      request: DiscreteActuatorControl.Request,
                      response: DiscreteActuatorControl.Response
                      ) -> DiscreteActuatorControl.Response:
        """Service callback to set the mode for tide-control in tank A."""
        self.auto_tide_setting_a = request.new_status > 0
        response.requested_status = int(self.auto_tide_setting_a)
        self.update()
        return response
    
    def set_auto_tide_b(self,
                      request: DiscreteActuatorControl.Request,
                      response: DiscreteActuatorControl.Response
                      ) -> DiscreteActuatorControl.Response:
        """Service callback to set the mode for tide-control in tank A."""
        self.auto_tide_setting_b = request.new_status > 0
        response.requested_status = int(self.auto_tide_setting_b)
        self.update()
        return response
    
    def publish_mode(self):
        """Publish the current modes of tide-control in tank A and B."""
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
