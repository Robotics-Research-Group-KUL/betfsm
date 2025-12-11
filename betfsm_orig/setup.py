from setuptools import find_packages, setup, glob

package_name = 'betfsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
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
            'web_server = ' + package_name + ".web_server:main",
        ],
    }
)
