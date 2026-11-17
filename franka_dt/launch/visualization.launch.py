import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_dir = get_package_share_directory(
        'franka_description'
    )

    xacro_file = os.path.join(
        description_dir,
        'robots',
        'fr3',
        'fr3.urdf.xacro'
    )

    # Physical Twin: mesh verdi
    pt_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' hand:=true description_pkg:=franka_pt_visual'
        ]),
        value_type=str
    )

    # Digital Twin: mesh arancioni
    dt_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' hand:=true arm_prefix:=dt description_pkg:=franka_dt'
        ]),
        value_type=str
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('franka_dt'),
        'rviz',
        'digital_twin.rviz'
    )

    return LaunchDescription([

        # Robot State Publisher del Physical Twin
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='physical_twin',
            name='pt_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': pt_description,
                'frame_prefix': 'pt/',
                'use_sim_time': True
            }],
            remappings=[
                ('joint_states', '/joint_states'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),

        # Posizionamento del Physical Twin
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pt_position',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'pt/base'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Robot State Publisher del Digital Twin
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='digital_twin',
            name='dt_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': dt_description,
                'frame_prefix': 'dt/',
                'use_sim_time': True
            }],
            remappings=[
                ('joint_states',
                 '/digital_twin/visualization_joint_states'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),

        # Posizionamento del Digital Twin
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dt_position',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'dt/base'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='digital_twin_visualization',
            arguments=[
                '-d',
                rviz_config_file
            ],
            output='screen'
        )

    ])
