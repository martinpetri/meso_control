from setuptools import setup

package_name = 'old_meso_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Petri',
    maintainer_email='martin.petri@awi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_string_val_node = old_meso_control_pkg.sensor_string_val_node:main",
            "small_control_node = old_meso_control_pkg.small_control_node:main",
            "simple_control_node = old_meso_control_pkg.simple_control_node:main",
            "modbus_tcp_node = old_meso_control_pkg.modbus_tcp_node:main",
            "test_node = old_meso_control_pkg.test_node:main",
        ],
    },
)