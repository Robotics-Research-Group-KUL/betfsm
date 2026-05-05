from setuptools import setup
import os
from glob import glob

package_name = 'betfsm_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include your html file in the share directory
        ('share/' + package_name, glob('betfsm_ros/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erwin',
    description='ROS 2 wrapper for BeTFSM',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'betfsm_node = betfsm_ros.betfsm_node:main',
            'betfsm_action_server = betfsm_ros.betfsm_action_server:main',
        ],
    },
)
