from setuptools import find_packages, setup

package_name = 'yasmin_etasl_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erwin Aertbeliën',
    maintainer_email='Erwin.Aertbelien@kuleuven.be',
    description='tutorial scripts for yasmin_etasl',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
