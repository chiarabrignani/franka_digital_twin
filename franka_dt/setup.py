import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'franka_dt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chiara',
    maintainer_email='chiara.brignani@edu.unife.it',
    description='Digital Twin bidirezionale del robot Franka FR3',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_manager = franka_dt.state_manager:main',
            'command_manager = franka_dt.command_manager:main',
            'tracking_test = franka_dt.tracking_test:main',
            'full_sequence_test = franka_dt.full_sequence_test:main',
            'odte_monitor = franka_dt.odte_monitor:main',
            'digital_model = franka_dt.digital_model:main',
        ],
    },
)
