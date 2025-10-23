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

    # 2. Avvio RViz con configurazione TIAGo (separato per maggiore controllo)
    rviz_launch = TimerAction(
        period=15.0,  # Avvia dopo che Gazebo è caricato
        actions=[
            LogInfo(msg=[
                "Avvio RViz con configurazione TIAGo...\n",
                "===================================="
            ]),
            ExecuteProcess(
                cmd=['bash', '-c', 
                     '. /opt/ros/humble/setup.bash && cd /home/claudio/tiago_public_ws && . install/setup.bash && ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tiago_2dnav)/share/tiago_2dnav/config/rviz/navigation.rviz'],
                output='screen'
            )
        ]
    )

    # 3. Nodi principali con timing coordinato (dopo caricamento Gazebo)
    # PRIMO: ArUco Scan Publisher - rileva i marker dalla fotocamera
    aruco_scan_publisher = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='aruco_scan_publisher',
                name='aruco_scan_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # SECONDO: ArUco Coordinate Transformation - trasforma le coordinate rispetto a base_footprint
    aruco_coord_transform = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='aruco_coord_transformation',
                name='aruco_coord_transformation',
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

    # QUARTO: Motion Planner Node - inversione cinematica per il braccio
    motion_planner = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='motion_planner_node',
                name='motion_planner_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # QUINTO: State Machine - coordina tutto il processo di manipolazione
    state_machine_node = TimerAction(
        period=26.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='state_machine',
                name='robot_state_machine_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ready_msg = TimerAction(
        period=32.0,
        actions=[
            LogInfo(msg=[
                "=== SISTEMA COMPLETO PRONTO ===\n",
                "Gazebo: ATTIVO con TIAGo caricato (t=2s)\n",
                "RViz: ATTIVO per visualizzazione (t=15s)\n",
                "ArUco Scan Publisher: ATTIVO (t=18s)\n",
                "ArUco Coord Transform: ATTIVO (t=20s)\n",
                "Head Movement: ATTIVO - scansione in corso (t=22s)\n",
                "Motion Planner: ATTIVO - inversione cinematica pronta (t=24s)\n",
                "State Machine: ATTIVA - attesa marker (t=26s)\n",
                "\nARCHITETTURA:\n",
                "Pipeline basata su architettura del collega (FUNZIONANTE)\n",
                "1. ArUco Scan Publisher: rileva marker dalla fotocamera\n",
                "2. ArUco Coord Transform: trasforma coordinate in base_footprint\n",
                "3. Head Movement: scansione dell'ambiente\n",
                "4. Motion Planner: inversione cinematica del braccio\n",
                "5. State Machine: coordina sequenza manipolazione\n",
                "\nISTRUZIONI:\n",
                "1. Posizionare 4 marker ArUco (ID: 1,2,3,4) davanti alla camera TiAGo\n",
                "2. Il robot inizierà automaticamente la ricerca\n",
                "3. Dopo rilevamento marker -> manipolazione automatica\n",
                "\nDEBUG:\n",
                "   ros2 topic list | grep aruco\n",
                "   ros2 topic echo /aruco_poses\n",
                "   ros2 topic echo /aruco_poses_transformed\n",
                "=================================="
            ])
        ]
    )

    return LaunchDescription([
        welcome_msg,                # t=0s - Messaggio iniziale
        gazebo_launch,              # t=2s - Avvia TIAGo
        rviz_launch,                # t=15s - Avvia RViz separatamente
        aruco_scan_publisher,       # t=18s - ArUco Scan Publisher (PRIMO - rileva marker)
        aruco_coord_transform,      # t=20s - ArUco Coordinate Transformation (SECONDO)
        head_movement_action_node,  # t=22s - Head Movement (TERZO - scansione)
        motion_planner,             # t=24s - Motion Planner (QUARTO - inversione cinematica)
        state_machine_node,         # t=26s - State Machine (QUINTO - coordina tutto)
        ready_msg                   # t=30s - Sistema pronto!
    ])
