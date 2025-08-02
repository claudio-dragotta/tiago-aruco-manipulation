from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_gazebo'),
                'launch',
                'tiago_gazebo.launch.py'
            )
        ),
        launch_arguments={'is_public_sim': 'True'}.items()
    )

    macchina_node = TimerAction(
        period=15.0,  # aspetta 10 secondi
        actions=[
            Node(
                package='robot_nodes',
                executable='macchina_a_stati',
                name='macchina_a_stati',
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )

    testa_node = TimerAction(
        period=20.0,  # aspetta 15 secondi
        actions=[
            Node(
                package='robot_nodes',
                executable='testa_e_aruco',
                name='testa_e_aruco',
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        macchina_node,
        testa_node
    ])
