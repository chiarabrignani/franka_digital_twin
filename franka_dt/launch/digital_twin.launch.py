from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Utilizza il clock simulato di Gazebo'
        ),

        Node(
            package='franka_dt',
            executable='state_manager',
            name='state_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='franka_dt',
            executable='command_manager',
            name='command_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='franka_dt',
            executable='digital_model',
            name='digital_model',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='franka_dt',
            executable='odte_monitor',
            name='odte_monitor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

    ])
