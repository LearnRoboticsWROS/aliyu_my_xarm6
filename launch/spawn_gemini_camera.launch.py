#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    # Costruisci il path in modo sicuro
    camera_urdf_path = PathJoinSubstitution([
        FindPackageShare('my_xarm6'),
        'urdf',
        'gemini_335_sim.urdf.xacro'
    ]).perform(context)

    # Leggi il file una volta risolta la substitution
    with open(camera_urdf_path, 'r') as f:
        robot_description_content = f.read()

    # Pubblica la descrizione URDF (include il plugin Gazebo)
    camera_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gemini_camera_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content,
        }],
    )

    return [camera_state_pub]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
