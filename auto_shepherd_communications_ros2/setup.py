from setuptools import setup
from glob import glob
import os

package_name = 'auto_shepherd_communications_ros2'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='28260769@Students.lincoln.ac.uk',
    description='ROS2 Communications for the Auto Shepard Summer School Work',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'serial_node.py = auto_shepherd_communications_ros2.serial_node:main',
            'serial_node_receiver.py = auto_shepherd_communications_ros2.serial_node_receiver:main',
            'bridge_interface.py = auto_shepherd_communications_ros2.bridge_interface:main',
            'data_loader_node.py = auto_shepherd_communications_ros2.data_loader_node:main'
        ],
    },
)
