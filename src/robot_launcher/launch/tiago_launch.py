from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():



    # 1. State Machine - COORDINATORE CENTRALE che gestisce tutto
    state_machine_node = TimerAction(
        period=8.0,  # Inizia per PRIMA come coordinatore
        actions=[
            Node(
                package='robot_nodes',
                executable='state_machine',
                name='robot_state_machine',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 2. IK Node - servizio per la cinematica, deve essere disponibile subito
    ik_node = TimerAction(
        period=10.0,  # Subito dopo state machine
        actions=[
            Node(
                package='robot_nodes',
                executable='ik',
                name='kinematic_planner',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 3. ArUco Detector - deve partire PRIMA del movimento testa
    aruco_detector_node = TimerAction(
        period=12.0,  # Parte prima per essere pronto a rilevare
        actions=[
            Node(
                package='robot_nodes',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 4. Head Movement - controllato dalla state machine, parte dopo ArUco detector
    head_movement_action_node = TimerAction(
        period=13.0,  # Parte dopo ArUco detector per sincronizzazione
        actions=[
            Node(
                package='robot_nodes',
                executable='head_movement_action_node',
                name='head_movement_action_node',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 5. System Monitor - monitoraggio sistema completo
    system_monitor_node = TimerAction(
        period=15.0,  # Dopo che tutti i componenti sono attivi
        actions=[
            Node(
                package='robot_nodes',
                executable='system_monitor',
                name='system_monitor',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        # Sequenza di avvio orchestrata per funzionamento armonioso:
        state_machine_node,         # t=8s:  COORDINATORE CENTRALE
        ik_node,                    # t=10s: Servizio cinematica pronto
        aruco_detector_node,        # t=12s: Rilevamento ArUco attivo (PRIMA!)
        head_movement_action_node,  # t=13s: Movimento testa disponibile
        system_monitor_node         # t=15s: Monitoraggio sistema completo
    ])

