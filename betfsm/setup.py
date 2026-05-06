from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'betfsm'

setup(
    name=package_name,
    # This remains empty because pyproject.toml handles the rest
    packages=find_packages(include=['betfsm', 'betfsm.*', 'betfsm_examples', 'betfsm_examples.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            "example_concurrentseq          = betfsm_examples.example_concurrentseq:main",
            "example_sequence1              = betfsm_examples.example_sequence1:main",  
            "example_sequence2              = betfsm_examples.example_sequence2:main",  
            "example_sequence3              = betfsm_examples.example_sequence3:main",  
            "example_statemachine           = betfsm_examples.example_statemachine:main",  
            "gui_example_simple1            = betfsm_examples.guiexample_simple1:main", 
            "gui_example_simple2            = betfsm_examples.guiexample_simple2:main", 
            "gui_example_large              = betfsm_examples.guiexample_large:main", 
            "example_common_nodes           = betfsm_examples.example_common_nodes:main",
            "example_logging                = betfsm_examples.example_logging:main"  ,   
            "example_runner                 = betfsm_examples.example_runner:main"
        ]
    },   
)
