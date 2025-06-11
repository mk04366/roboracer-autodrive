import os
from glob import glob
from setuptools import setup

package_name = 'system_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/system_launch']),
        ('share/system_launch', ['package.xml']),
        ('share/system_launch/launch', ['launch/full_system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ammar',
    maintainer_email='mammark14@gmail.com',
    description='Central launch package for entire system',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
