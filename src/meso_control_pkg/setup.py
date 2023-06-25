import os
from glob import glob
from setuptools import setup

package_name = 'meso_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch',
                                                                           '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Petri',
    maintainer_email='martin.petri@awi.de',
    description='Controlsoftware for a mesocosm tank-pair.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mock_publisher = meso_control_pkg.mock_publisher:main",
            "water_sensor_node = meso_control_pkg.water_sensor_node:main",
            "modbus_tcp_node = meso_control_pkg.modbus_tcp_node:main",
            "sps_binary_actuator_node = meso_control_pkg.sps_binary_actuator_node:main",
            "sps_continuous_actuator_node = meso_control_pkg.sps_continuous_actuator_node:main",
            "meso_state_machine = meso_control_pkg.meso_state_machine:main"
        ],
    },
)