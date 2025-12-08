from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/amcl_launch.py',
        'launch/slam_launch.py',
        'launch/cartographer_launch.py',
        'launch/hector_launch.py',
        'launch/rtabmap_launch.py'
    ]),
    ('share/' + package_name + '/config', [
        'config/amcl_config.yaml',
        'config/slam_config.yaml',
        'config/cartographer_config.lua',
        'config/hector_config.yaml',
        'config/rtabmap_config.yaml'
    ]),
    ('share/' + package_name + '/maps', [
        'maps/map5.yaml',
        'maps/map5.pgm'
    ]),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='localhost',
    maintainer_email='localhost@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
