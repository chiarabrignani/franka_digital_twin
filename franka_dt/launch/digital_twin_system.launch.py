import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Directory dei launch del Digital Twin
    dt_launch_dir = os.path.join(
        get_package_share_directory('franka_dt'),
        'launch'
    )

    # Launch della simulazione Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_gazebo_bringup'),
                'launch',
                'gazebo_franka_arm_example_controller.launch.py'
            ])
        ),
        launch_arguments={
            'load_gripper': 'true',
            'franka_hand': 'franka_hand',
            'robot_type': 'fr3',
            'controller': 'fr3_arm_controller',
            'rviz': 'true',
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # Nodi principali del Digital Twin
    digital_twin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                dt_launch_dir,
                'digital_twin.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # Visualizzazione PT verde e DT arancione
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                dt_launch_dir,
                'visualization.launch.py'
            )
        )
    )

    # Generatore dei comandi
    command_generator = Node(
        package='franka_dt',
        executable='digital_command_generator',
        name='digital_command_generator',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([

        # 1. Avvio della simulazione
        gazebo_launch,

        # 2. Avvio dei nodi del Digital Twin
        digital_twin_launch,

        # 3. Avvio della visualizzazione
        visualization_launch,

        # 4. Avvio del generatore dei comandi
        command_generator,

    ])
