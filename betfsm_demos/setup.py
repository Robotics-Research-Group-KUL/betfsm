from setuptools import find_packages, setup
import glob

package_name = 'betfsm_demos'

import os



# Function to dynamically build the data_files list for a whole tree
def generate_data_files(source_dir, target_base):
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            # Determine the relative path to maintain the nested structure
            rel_path = os.path.relpath(root, source_dir)
            # Define the target installation directory in 'share'
            if rel_path == '.':
                target_dir = os.path.join(target_base, source_dir)
            else:
                target_dir = os.path.join(target_base, source_dir, rel_path)
            
            # Get full paths of all files in the current directory
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, file_paths))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]
data_files.extend(generate_data_files('tasks', os.path.join('share', package_name)))



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
