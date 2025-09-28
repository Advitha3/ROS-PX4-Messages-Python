from setuptools import find_packages, setup

package_name = 'sensor_combined'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='advitha',
    maintainer_email='advitha@example.com',
    description='PX4 Figure-Eight Mission node for VTOL with arming, takeoff, transition, and trajectory execution.',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'figure_eight_node = sensor_combined.waypoint:main',
            'log_messages  = sensor_combined.log_messages:main'
        ],
    },
)
