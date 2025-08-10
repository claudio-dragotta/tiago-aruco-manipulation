#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file sistema completo - Nodi ArUco per sistema con Gazebo già avviato
    """

    # Messaggio di avvio
    welcome_msg = LogInfo(
        msg=[
            "=== SISTEMA ARUCO MANIPULATION ATTIVO ===\n",
            "Gazebo e RViz dovrebbero essere già avviati.\n",
            "Avvio nodi ArUco per manipolazione TiAGO...\n",
            "Posizionare 4 marker ArUco davanti alla camera.\n",
            "==========================================="
        ]
    )

    # Nodi principali con timing coordinato
    state_machine_node = TimerAction(
        period=5.0,  
        actions=[
            Node(
                package='robot_nodes',
                executable='state_machine',
                name='robot_state_machine',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ik_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='ik',
                name='kinematic_planner',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    aruco_detector_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    head_movement_action_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='head_movement_action_node',
                name='head_movement_action_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ready_msg = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg=[
                "=== SISTEMA COMPLETO PRONTO ===\n",
                "• Gazebo: ATTIVO\n",
                "• RViz: ATTIVO\n", 
                "• TiAGO: Caricato\n",
                "• ArUco System: ATTIVO\n",
                "\nPosizionare 4 marker ArUco (ID: 1,2,3,4) davanti alla camera TiAGO\n",
                "Monitor: ros2 topic echo /all_markers_found\n",
                "============================"
            ])
        ]
    )

    return LaunchDescription([
        welcome_msg,
        state_machine_node,     # t=5s
        ik_node,               # t=7s
        aruco_detector_node,   # t=9s
        head_movement_action_node, # t=10s
        ready_msg             # t=15s
    ])
