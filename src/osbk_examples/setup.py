from setuptools import setup

package_name = 'osbk_examples'

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
    description='Examples for learning and testing the osbk-library.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rest_sensor_nodes = osbk_examples.rest_sensor_nodes:main',
            'example_state_machine = osbk_examples.example_state_machine:main',
            'example_actuator = osbk_examples.example_actuator:main'
        ],
    },
)
