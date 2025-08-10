#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """Launch system principale - Solo nodi essenziali"""
    
    nodes = [
        Node(
            package='robot_nodes',
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),
        Node(
            package='robot_nodes', 
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen'
        ),
        Node(
            package='robot_nodes',
            executable='head_movement_action_node', 
            name='head_movement_action_node',
            output='screen'
        ),
        Node(
            package='robot_nodes',
            executable='ik',
            name='ik_node',
            output='screen'
        )
    ]
    
    welcome = LogInfo(msg="=== SISTEMA NODI AVVIATO ===")
    
    return LaunchDescription([welcome] + nodes)
