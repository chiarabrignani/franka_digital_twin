import os
from glob import glob
from setuptools import setup

package_name = 'franka_pt_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
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
            os.path.join(
                'share', package_name,
                'meshes', 'robots', 'fr3', 'visual'
            ),
            glob('meshes/robots/fr3/visual/*.dae')
        ),
        (
            os.path.join(
                'share', package_name,
                'meshes', 'robot_ee', 'franka_hand_white', 'visual'
            ),
            glob('meshes/robot_ee/franka_hand_white/visual/*.dae')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chiara',
    maintainer_email='chiara.brignani@edu.unife.it',
    description='Mesh verdi per la visualizzazione del Physical Twin Franka FR3',
    license='MIT',
)
