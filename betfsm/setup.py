from setuptools import setup
import os
from glob import glob

package_name = 'betfsm'

setup(
    name=package_name,
    # This remains empty because pyproject.toml handles the rest
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
