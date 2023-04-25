import pytest
import rclpy
import json
from threading import Thread

from osbk_devices.rest_sensor import RestSensor
from rclpy.parameter import Parameter
from http.server import HTTPServer, BaseHTTPRequestHandler


@pytest.fixture
def sensor_obj():
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


class HTTPHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        response = json.dumps({
            'path': {
                'to': {
                    'json': 22.5,
                    'not-json': 22.6
                },
                'not-to': -1
            }
        })

        self.send_response(200)
        self.end_headers()
        self.wfile.write(response.encode('utf-8'))


def test_read_sensor(sensor_obj, server):
    # param_list = [
    #     Parameter('source_url', Parameter.Type.STRING,
    #               'http://echo.jsontest.com/test/22.5/one/two'),
    #     Parameter('path_to_value', Parameter.Type.STRING,
    #               'test'),
    #     Parameter('unit', Parameter.Type.STRING, '°C')
    # ]
    param_list = [
        Parameter('source_url', Parameter.Type.STRING, 'http://localhost:55555'),
        Parameter('path_to_value', Parameter.Type.STRING, 'path.to.json'),
        Parameter('unit', Parameter.Type.STRING, '°C')
    ]
    sensor_obj.set_parameters(param_list)

    http_thread = Thread(target=server.handle_request)
    print("start")
    http_thread.start()
    print("read")
    reading = sensor_obj.read_sensor()
    print("join")
    http_thread.join()

    print(f'this: {reading}')

    assert reading.topic_name == 'RESTSensor/value'
    assert reading.data == 22.5
    assert reading.unit == '°C'
    rclpy.shutdown()
