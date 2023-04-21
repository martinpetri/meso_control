from setuptools import setup

package_name = 'osbk_devices'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Galdiga',
    maintainer_email='t.galdiga@uni-bremen.de',
    description='Provides basic hardware nodes to inherit from',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rest_sensor = osbk_devices.rest_sensor:main',
            'actuator_base = osbk_devices.actuator_base:main',
        ],
    },
)
