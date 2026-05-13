from setuptools import find_packages, setup

import glob

package_name = 'betfsm_demos'

import os

# def package_files(install_base, source_base):
#     """installs to **content** of source_base in the directory install_base (recursively)"""
#     data = []
#     for (path, directories, filenames) in os.walk(source_base):
#         for filename in filenames:
#             data.append((  os.path.normpath(os.path.join(install_base,path,"..")), os.path.normpath(os.path.join(path,filename)) ) )
#     return data

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/tasks", ['./tasks/my_tasks.json'])
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erwin Aertbeliën',
    maintainer_email='Erwin.Aertbelien@kuleuven.be',
    description='tutorial scripts for BeTFSM',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "skill_example_1  = betfsm_demos.skill_example_1:main",
            "skill_example_2  = betfsm_demos.skill_example_2:main",
            "skill_example_3  = betfsm_demos.skill_example_3:main",
            "skill_example_4  = betfsm_demos.skill_example_4:main"
            #'test_ea = ' + package_name + ".test_ea:main",
            #'example_graphviz = ' +package_name + ".example_graphviz:main",
            #'example_action_server = ' +package_name + ".example_action_server:main",            
            #'example_action_server2 = ' +package_name + ".example_action_server2:main"            
        ],
    },
)
