import pytest
import rclpy
import json
from threading import Thread

from osbk_devices.rest_sensor import RestSensor
from rclpy.parameter import Parameter
from http.server import HTTPServer, BaseHTTPRequestHandler


class HTTPHandler(BaseHTTPRequestHandler):
    """HTTPRequestHandler to deliver values to RESTSensor."""

    def do_GET(self):
        """Send response to GET-request."""
        # the response json
        response = json.dumps({
            'path': {
                'to': {
                    'json': 22.5,
                    'not-json': 22.6
                },
                'not-to': -1
            }
        })

        # Code 200(OK)
        self.send_response(200)
        self.end_headers()

        # send response
        self.wfile.write(response.encode('utf-8'))


@pytest.fixture
def sensor_obj():
    try:
        rclpy.init()
    except(RuntimeError):
        rclpy.shutdown()
        rclpy.init()
    return RestSensor()


@pytest.fixture
def server():
    addr = ('localhost', 55555)
    return HTTPServer(addr, HTTPHandler)


def test_object_creation():
    rclpy.init()
    obj = RestSensor()

    assert obj is not None
    assert obj.has_parameter('source_url')
    assert obj.has_parameter('path_to_value')
    assert obj.has_parameter('unit')

    rclpy.shutdown()


def test_read_sensor(sensor_obj, server):
    # define ROS-parameters for RestServer-node
    param_list = [
        Parameter('source_url', Parameter.Type.STRING,
                  'http://localhost:55555'),
        Parameter('path_to_value', Parameter.Type.STRING, 'path.to.json'),
        Parameter('unit', Parameter.Type.STRING, 'degC')
    ]
    sensor_obj.set_parameters(param_list)

    # start separate thread with http-server
    http_thread = Thread(target=server.handle_request)
    http_thread.start()
    # call read_sensor
    reading = sensor_obj.read_sensor()
    # wait for http server to shutdown after one request
    http_thread.join()

    assert reading.topic_name == 'RESTSensor/value'
    assert reading.data == 22.5
    assert reading.unit == 'degC'

    rclpy.shutdown()
