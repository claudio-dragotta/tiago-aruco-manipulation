#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file sistema completo - Avvia tutto automaticamente in sequenza
    """

    # Messaggio di avvio
    welcome_msg = LogInfo(
        msg=[
            "=== SISTEMA COMPLETO TIAGO ARUCO MANIPULATION ===\n",
            "Avvio automatico di:\n",
            "   - Gazebo con TIAGo\n",
            "   - RViz per visualizzazione\n", 
            "   - Tutti i nodi di manipolazione\n",
            "Attendere 30 secondi per caricamento completo...\n",
            "Posizionare 4 marker ArUco (ID: 1,2,3,4) davanti alla camera\n",
            "================================================="
        ]
    )

    # 1. Avvio TIAGo dalla directory locale
    gazebo_launch = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg=[
                "Avvio TIAGo da installazione locale...\n",
                "Directory: /home/claudio/tiago_public_ws\n",
                "Attendere caricamento Gazebo...\n",
                "================================="
            ]),
            ExecuteProcess(
                cmd=['bash', '-c', 
                     '. /opt/ros/humble/setup.bash && cd /home/claudio/tiago_public_ws && . install/setup.bash && ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True'],
                output='screen'
            )
        ]
    )

    # RViz viene già avviato automaticamente da TIAGo gazebo launch

    # 3. Nodi principali con timing coordinato (dopo caricamento Gazebo)
    # PRIMO: State Machine - coordina tutto il processo
    state_machine_node = TimerAction(
        period=18.0,  
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

    # SECONDO: ArUco Detector - rileva i marker
    aruco_detector_node = TimerAction(
        period=20.0,
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

    # TERZO: Head Movement - scansione da sinistra a destra
    head_movement_action_node = TimerAction(
        period=22.0,
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
    
    # QUARTO: IK Node - manipolazione oggetti
    ik_node = TimerAction(
        period=24.0,
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

    ready_msg = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg=[
                "=== SISTEMA COMPLETO PRONTO ===\n",
                "Gazebo: ATTIVO con TIAGo caricato\n",
                "RViz: ATTIVO per visualizzazione\n", 
                "TiAGO: Configurazione completata\n",
                "ArUco System: ATTIVO e in scansione\n",
                "State Machine: ATTIVA\n",
                "\nISTRUZIONI:\n",
                "1. Posizionare 4 marker ArUco (ID: 1,2,3,4) davanti alla camera TiAGo\n",
                "2. Il robot inizierà automaticamente la scansione\n",
                "3. Dopo rilevamento marker -> manipolazione automatica\n",
                "\nMONITORING:\n",
                "   ros2 topic echo /all_markers_found\n",
                "   ros2 topic echo /marker_discovered\n",
                "=================================="
            ])
        ]
    )

    return LaunchDescription([
        welcome_msg,           # t=0s - Messaggio iniziale
        gazebo_launch,         # t=2s - Avvia TIAGo (include già RViz)
        state_machine_node,    # t=18s - State Machine (PRIMO - coordina tutto)
        aruco_detector_node,   # t=20s - ArUco Detector (SECONDO - rileva marker)
        head_movement_action_node, # t=22s - Head Movement (TERZO - scansione)
        ik_node,              # t=24s - IK Node (QUARTO - manipolazione)
        ready_msg             # t=30s - Sistema pronto!
    ])
