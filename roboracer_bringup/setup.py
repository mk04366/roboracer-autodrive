from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyh',
    maintainer_email='93693983+Lyhoh@users.noreply.github.com',
    description='System bringup package for launching the full roboracer stack.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
