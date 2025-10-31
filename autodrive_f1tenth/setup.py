from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_f1tenth'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak, Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu, tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem ROS 2 Package for F1TENTH',
    license='BSD',
    entry_points={
        'console_scripts': [
            'autodrive_bridge = autodrive_f1tenth.autodrive_bridge:main',
            'teleop_keyboard = autodrive_f1tenth.teleop_keyboard:main',
        ],
    },
)
