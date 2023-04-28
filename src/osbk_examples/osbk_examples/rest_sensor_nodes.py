import rclpy
from rclpy.parameter import Parameter

from osbk_devices.rest_sensor import RestSensor
from osbk_operation.periodic_sensor_scheduler import PeriodicSensorScheduler


def main():
    rclpy.init()

    pegelonline = RestSensor('Pegelonline')
    pegelonline.set_parameters([
        Parameter('source_url', Parameter.Type.STRING,
                  'https://www.pegelonline.wsv.de/webservices/rest-api/v2/st' +
                  'ations/LIST AUF SYLT/W/measurements.json?start=P0DT0H00M'),
        Parameter('path_to_value', Parameter.Type.STRING, '0.value'),
        Parameter('unit', Parameter.Type.STRING, 'cm')
    ])

    jsontest = RestSensor('JSONTest')
    jsontest.set_parameters([
        Parameter('source_url', Parameter.Type.STRING,
                  'http://echo.jsontest.com/key/value/temp/22.5'),
        Parameter('path_to_value', Parameter.Type.STRING, 'temp'),
        Parameter('unit', Parameter.Type.STRING, 'degC')
    ])

    scheduler = PeriodicSensorScheduler()
    scheduler.add_sensor(pegelonline, 5)
    scheduler.add_sensor(jsontest, 10)

    try:
        rclpy.spin(scheduler)
    except KeyboardInterrupt:
        print('Shutdown...')

    pegelonline.destroy_node()
    jsontest.destroy_node()
    scheduler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
