from setuptools import find_packages, setup

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/maps', [
        'maps/map5.yaml',
        'maps/map5.pgm'
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bl',
    maintainer_email='bo.ryanli0420@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lane_generator = planning.lane_generator:main",
        ],
    },
)
