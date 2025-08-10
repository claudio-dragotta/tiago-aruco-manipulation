#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """Launch debug - Con output verbose"""
    
    nodes = [
        Node(
            package='robot_nodes',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_nodes', 
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_nodes',
            executable='head_movement_action_node', 
            name='head_movement_action_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_nodes',
            executable='ik',
            name='ik_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ]
    
    welcome = LogInfo(msg="=== DEBUG SYSTEM ATTIVO ===")
    
    return LaunchDescription([welcome] + nodes)