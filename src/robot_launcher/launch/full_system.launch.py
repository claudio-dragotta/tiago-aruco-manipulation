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

    # Variabili d'ambiente per rendering WSL2 e display OpenCV
    display_env = {
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'MESA_GL_VERSION_OVERRIDE': '3.3',
        'DISPLAY': os.environ.get('DISPLAY', ':0'),
    }

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
                output='screen',
                additional_env=display_env
            )
        ]
    )

    # 3. Nodi principali con timing coordinato (dopo caricamento Gazebo)
    # PRIMO: ArUco Detector - rileva i marker e pubblica pose ottimizzate + /all_markers_found
    aruco_detector = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=display_env
            )
        ]
    )

    # SECONDO: Head Movement - scansione da sinistra a destra
    head_movement_action_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='head_movement_action_node',
                name='head_movement_action_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=display_env
            )
        ]
    )

    # TERZO: IK Node - inversione cinematica, ascolta command_topic (Int32) e aruco_base_pose_X
    ik_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='ik',
                name='kinematic_planner',
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=display_env
            )
        ]
    )

    # QUARTO: State Machine - coordina tutto il processo di manipolazione
    state_machine_node = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='robot_nodes',
                executable='state_machine',
                name='robot_state_machine_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                additional_env=display_env
            )
        ]
    )

    ready_msg = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg=[
                "=== SISTEMA COMPLETO PRONTO ===\n",
                "Gazebo: ATTIVO con TIAGo caricato (t=2s)\n",
                "RViz: ATTIVO per visualizzazione (t=15s)\n",
                "ArUco Detector: ATTIVO (t=18s)\n",
                "Head Movement: ATTIVO - scansione in corso (t=20s)\n",
                "IK Node: ATTIVO - inversione cinematica pronta (t=22s)\n",
                "State Machine: ATTIVA - attesa marker (t=24s)\n",
                "\nARCHITETTURA:\n",
                "1. ArUco Detector: rileva marker, pubblica /aruco_base_pose_X e /all_markers_found\n",
                "2. Head Movement: scansione dell'ambiente\n",
                "3. IK Node: ascolta command_topic (Int32), calcola traiettorie\n",
                "4. State Machine: coordina sequenza manipolazione\n",
                "\nDEBUG:\n",
                "   ros2 topic list | grep aruco\n",
                "   ros2 topic echo /all_markers_found\n",
                "   ros2 topic echo /aruco_base_pose_1\n",
                "   ros2 topic echo /command_topic\n",
                "=================================="
            ])
        ]
    )

    return LaunchDescription([
        welcome_msg,                # t=0s  - Messaggio iniziale
        gazebo_launch,              # t=2s  - Avvia TIAGo (include RViz automaticamente)
        aruco_detector,             # t=18s - ArUco Detector (rileva marker)
        head_movement_action_node,  # t=20s - Head Movement (scansione)
        ik_node,                    # t=22s - IK Node (inversione cinematica)
        state_machine_node,         # t=24s - State Machine (coordina tutto)
        ready_msg                   # t=30s - Sistema pronto!
    ])
