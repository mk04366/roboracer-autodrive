from setuptools import setup

package_name = 'rl_training'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autroboracer',
    maintainer_email='user@example.com',
    description='RL SAC environment for RoboRacer AutoDRIVE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_sac = rl_training.train_sac:main'
        ],
    },
)