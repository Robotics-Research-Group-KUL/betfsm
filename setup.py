from setuptools import find_packages, setup, glob

package_name = 'yasmin_etasl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/tasks", ['./tasks/my_tasks.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='Erwin.Aertbelien@kuleuven.be',
    description='TODO: Package description',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_ea = ' + package_name + ".test_ea:main",
            'example_graphviz = ' +package_name + ".example_graphviz:main"
        ],
    }
)
