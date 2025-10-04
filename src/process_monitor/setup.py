import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'process_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jetson User',
    maintainer_email='jetson@example.com',
    description='ROS 2 node that publishes per-process CPU and memory usage for Foxglove table view.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_monitor = process_monitor.process_monitor_node:main',
        ],
    },
)
