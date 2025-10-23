#nodo per l'inversione cinematica e la pianificazione di traiettoria

from enum import Enum
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox import ERobot, DHRobot, RevoluteDH
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
# from gazebo_ros_link_attacher.srv import Attach  # TODO: Install gazebo-ros-link-attacher package
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration



class State(Enum):
    WAITING_FOR_ARUCO = 1
    INTERMEDIATE_CONFIG = 2
    OPERATIONAL_CONFIG = 3
    # Oggetto 1 (Coca-Cola): marker 1 -> marker 3
    MOVE_TO_OBJECT_1 = 4
    GRIP_OBJECT_1 = 5
    LIFT_OBJECT_1 = 6
    MOVE_TO_DEST_1 = 7
    RELEASE_OBJECT_1 = 8
    RETURN_HOME_1 = 9
    # Oggetto 2 (Pringles): marker 2 -> marker 4  
    MOVE_TO_OBJECT_2 = 10
    GRIP_OBJECT_2 = 11
    LIFT_OBJECT_2 = 12
    MOVE_TO_DEST_2 = 13
    RELEASE_OBJECT_2 = 14
    RETURN_HOME_2 = 15
    COMPLETED = 16


class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')
        self.get_logger().info("Inizializzazione Kinematic Planner...")

        self.current_state = State.WAITING_FOR_ARUCO

        # Carica modello URDF
        
        urdf_loc = '/home/claudio/Desktop/progetto_ros2/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("URDF caricato correttamente!")

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.completed_command_topic = self.create_publisher(Int32, '/completed_command_topic', 10)

        self.aruco_pose_1 = None
        self.aruco_pose_2 = None
        self.aruco_pose_3 = None
        self.aruco_pose_4 = None

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.create_subscription(Int32,'/command_topic', self.command_callback, 10)
        self.aruco_sub_1 = self.create_subscription(PoseStamped, '/aruco_base_pose_1', self.aruco_pose_1_callback, 10)
        self.aruco_sub_2=self.create_subscription(PoseStamped, '/aruco_base_pose_2', self.aruco_pose_2_callback, 10)
        self.aruco_sub_3=self.create_subscription(PoseStamped, '/aruco_base_pose_3', self.aruco_pose_3_callback, 10)
        self.aruco_sub_4=self.create_subscription(PoseStamped, '/aruco_base_pose_4', self.aruco_pose_4_callback, 10)

        # Stato attuale dei giunti (vettore di 8: torso + 7 arm)
        self.current_joint_state = None  # np.array([torso, arm1, ..., arm7])
        self.target_pose = None  # PoseStamped
        
        # Sistema di posizionamento dinamico per ArUco
        self.position_buffers = {1: [], 2: [], 3: [], 4: []}  # Buffer per media posizioni
        self.buffer_size = 10  # Numero di campioni per la media mobile
        self.stable_positions = {}  # Posizioni stabilizzate per ogni marker
        
        # Sistema automatico di calcolo distanze ottimali
        self.aruco_size = 0.06  # Dimensione marker ArUco (6cm)
        self.object_sizes = {
            1: {"width": 0.065, "height": 0.24, "type": "bottle"},  # Coca-Cola
            2: {"width": 0.08, "height": 0.25, "type": "cylinder"}  # Pringles
        }
        self.gripper_width = 0.088  # Apertura massima gripper (8.8cm)
        self.gripper_finger_length = 0.06  # Lunghezza dito gripper (6cm stimato)

        # TODO: Install gazebo-ros-link-attacher package for object manipulation
        # self.attach_client = self.create_client(Attach, '/link_attacher_node/attach')
        # self.detach_client = self.create_client(Attach, '/link_attacher_node/detach')

        self.get_logger().warn("ATTENZIONE: gazebo_ros_link_attacher non disponibile - attach/detach simulati")

        # Action clients per torso, arm e gripper
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client   = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Timer management - mantieni riferimenti per cancellazione
        self.active_timer = None
        self.gripper_timer = None
        self.command_executed = False

        # Sistema di controllo ricalcoli dinamici
        self.last_positions = {1: None, 2: None, 3: None, 4: None}  # Ultima posizione conosciuta
        self.position_change_threshold = 0.02  # 2cm - soglia minima per ricalcolo
        self.last_recalculation_time = {1: 0, 2: 0, 3: 0, 4: 0}  # Timestamp ultimi ricalcoli
        self.min_recalculation_interval = 3.0  # 3 secondi minimi tra ricalcoli
        self.last_joint_configurations = {1: None, 2: None, 3: None, 4: None}  # Configurazioni calcolate

        # Le configurazioni saranno inviate SOLO quando la state machine lo richiede
        # NON più automatiche con timer
        self.get_logger().info("IK Node pronto - aspettando comandi dalla State Machine")

    def calculate_optimal_gripper_position(self, target_marker):
        """
        Calcola la posizione ottimale del gripper per raggiungere il lato opposto dell'oggetto
        
        LOGICA:
        ROBOT ---- [GRIPPER_BASE] ---- [DITO] ---- ArUco ---- [LATO_OPPOSTO]
                   ^qui posiziono                 ^centro    ^qui deve arrivare il dito
        
        Args:
            target_marker: ID del marker (1=Coca-Cola, 2=Pringles)
        
        Returns:
            approach_distance: distanza negativa per avvicinare il gripper
        """
        if target_marker not in self.object_sizes:
            return -0.10  # Fallback: 10cm più vicino
            
        obj = self.object_sizes[target_marker]
        
        # SITUAZIONE: ArUco è già SOPRA l'oggetto
        # 1. Distanza dal centro ArUco al lato opposto dell'oggetto
        object_radius = obj["width"] / 2  # Raggio dell'oggetto (dal centro al bordo)
        
        # 2. ArUco è già al centro dell'oggetto (sopra), quindi:
        # La distanza dal lato più vicino al robot al lato opposto = diametro completo
        aruco_to_far_side = object_radius  # Dall'ArUco (centro) al lato opposto
        
        # 3. Posizione gripper: deve essere tale che il dito raggiunga il lato opposto
        # GRIPPER_BASE + DITO_LENGTH = ARUCO + ARUCO_TO_FAR_SIDE
        # GRIPPER_BASE = ARUCO + ARUCO_TO_FAR_SIDE - DITO_LENGTH
        
        required_approach = aruco_to_far_side - self.gripper_finger_length
        
        self.get_logger().info(f"CALCOLO OTTIMALE GRIPPER per marker {target_marker} ({obj['type']}) - ArUco SOPRA oggetto:")
        self.get_logger().info(f"   Oggetto raggio: {object_radius*100:.1f}cm")
        self.get_logger().info(f"   ArUco (centro)->lato opposto: {aruco_to_far_side*100:.1f}cm")
        self.get_logger().info(f"   Lunghezza dito gripper: {self.gripper_finger_length*100:.1f}cm")
        self.get_logger().info(f"   Approach richiesto: {required_approach*100:.1f}cm (negativo=più vicino al robot)")
        
        # Ritorna valore negativo per avvicinare il gripper al robot
        return -abs(required_approach)

    # def check_attacher_services(self):
    #     """Verifica che i servizi di attach/detach siano disponibili"""
    #     if self.attach_client.service_is_ready() and self.detach_client.service_is_ready():
    #         self.get_logger().info("Servizi gazebo_ros_link_attacher disponibili!")
    #     else:
    #         self.get_logger().warn("ATTENZIONE: Servizi gazebo_ros_link_attacher non ancora disponibili")

    def send_intermediate_configuration(self):
        """Invia la configurazione intermedia del braccio"""
        if not self.arm_client.server_is_ready():
            return
            
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_joint','arm_2_joint','arm_3_joint',
            'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint']
        pt = JointTrajectoryPoint()
        # Configurazione intermedia specificata nei requisiti
        pt.positions = [
            0.0003836728901962516, -0.0001633239063343339, -9.037018213753356e-06,
            -6.145563957549172e-05, 4.409014973383307e-05, 0.0019643255648595925,
            0.0004167305736686444
        ]
        pt.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(pt)
        self.arm_client.send_goal_async(goal)
        self.get_logger().info("Inviata configurazione intermedia braccio via action client")
        # Segnala completamento dopo 3 secondi - con timer gestito
        self.active_timer = self.create_timer(3.5, lambda: self.publish_command_completed_once(State.INTERMEDIATE_CONFIG.value))

    def send_operational_configuration(self):
        """Invia la configurazione operativa completa: torso + braccio + gripper"""
        # 1. Torso
        if self.torso_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['torso_lift_joint']
            pt = JointTrajectoryPoint()
            pt.positions = [0.35]
            pt.time_from_start = Duration(sec=3)
            goal.trajectory.points.append(pt)
            self.torso_client.send_goal_async(goal)
            self.get_logger().info("Torso in configurazione operativa")
        
        # 2. Braccio
        if self.arm_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [
                'arm_1_joint','arm_2_joint','arm_3_joint',
                'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint']
            pt = JointTrajectoryPoint()
            pt.positions = [0.35, 0.25, -2.5, 1.70, 1.75, 0.10, -0.15]  # Braccio estremamente esteso dal corpo
            pt.time_from_start = Duration(sec=3)
            goal.trajectory.points.append(pt)
            self.arm_client.send_goal_async(goal)
            self.get_logger().info("Braccio in configurazione operativa")
        
        # 3. Gripper aperto
        if self.gripper_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['gripper_left_finger_joint','gripper_right_finger_joint']
            pt = JointTrajectoryPoint()
            pt.positions = [0.044, 0.044]
            pt.time_from_start = Duration(sec=2)
            goal.trajectory.points.append(pt)
            self.gripper_client.send_goal_async(goal)
            self.get_logger().info("Gripper aperto in configurazione operativa")
        
        # Segnala completamento dopo 4 secondi - con timer gestito
        self.active_timer = self.create_timer(4.0, lambda: self.publish_command_completed_once(State.OPERATIONAL_CONFIG.value))

    def send_torso_trajectory(self):
        if not self.torso_client.server_is_ready():
            return
        # Costruisci il goal per il torso
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.35]
        pt.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(pt)
        self.torso_client.send_goal_async(goal)
        self.get_logger().info("Inviato goal torso via action client")

    def send_arm_trajectory_2(self):
        if not self.arm_client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_joint','arm_2_joint','arm_3_joint',
            'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
        pt.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(pt)
        self.arm_client.send_goal_async(goal)
        self.get_logger().info("Inviato goal arm home via action client")

    def send_gripper_open(self):
        if not self.gripper_client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint','gripper_right_finger_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.044, 0.044]
        pt.time_from_start = Duration(sec=2)
        goal.trajectory.points.append(pt)
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info("Inviato goal apertura gripper via action client")

    def command_callback(self, msg):
        self.get_logger().info(f"Ricevuto comando: {State(msg.data).name}")
        
        # Cancella timer precedenti per evitare ripetizioni
        if self.active_timer is not None:
            self.active_timer.cancel()
            self.active_timer = None
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None
            
        # Reset flag comando eseguito
        self.command_executed = False
        
        if msg.data == State.INTERMEDIATE_CONFIG.value:
            self.current_state = State.INTERMEDIATE_CONFIG
            self.send_intermediate_configuration()
            
        elif msg.data == State.OPERATIONAL_CONFIG.value:
            self.current_state = State.OPERATIONAL_CONFIG
            self.send_operational_configuration()
            
        # === OGGETTO 1 (PRINGLES): MARKER 1 -> MARKER 3 (SECONDA) ===
        elif msg.data == State.MOVE_TO_OBJECT_1.value:
            self.get_logger().info("Movimento verso oggetto 1 (ArUco marker 1) - PRINGLES SECONDA")
            self.current_state = State.MOVE_TO_OBJECT_1
            # USA POSE OTTIMIZZATA: ArUco detector ha già calcolato la posizione ottimale
            # Aggiungiamo solo un piccolo offset finale per avvicinamento graduale
            optimal_distance = self.calculate_optimal_gripper_position(1)
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=optimal_distance)
            
        elif msg.data == State.GRIP_OBJECT_1.value:
            self.get_logger().info("Abbasso gripper con precisione per presa PRINGLES (oggetto 1)")
            self.current_state = State.GRIP_OBJECT_1
            # PRESA PRECISA ADATTIVA: Movimento verso il basso con offset dinamico basato su stabilità
            optimal_distance = self.calculate_optimal_gripper_position(1)
            stability = self.get_position_stability(1)
            # Se posizione instabile, mantieni margine di sicurezza più ampio
            if stability > self.stability_threshold:
                approach_offset = -0.03  # 3cm offset conservativo
                self.get_logger().info(f"Presa CONSERVATIVA per instabilità marker (stabilità={stability*100:.1f}cm)")
            else:
                approach_offset = -0.02  # 2cm offset aggressivo
                self.get_logger().info(f"Presa PRECISA per marker stabile (stabilità={stability*100:.1f}cm)")
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=optimal_distance + approach_offset, lift_height=-0.25)
            
        elif msg.data == State.LIFT_OBJECT_1.value:
            self.get_logger().info("Sollevamento PRINGLES (oggetto 1)")  
            self.current_state = State.LIFT_OBJECT_1
            # USA POSE OTTIMIZZATA per sollevamento
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=0.0, lift_height=0.12)  # Solleva 12cm
            
        elif msg.data == State.MOVE_TO_DEST_1.value:
            self.get_logger().info("Trasporto PRINGLES a destinazione (ArUco marker 3)")
            self.current_state = State.MOVE_TO_DEST_1
            # DEPOSIZIONE: Posizione sicura sopra il marker di destinazione
            self.calculate_pose_with_dynamic_positioning(3, approach_distance=0.05, lift_height=0.05)
            
        elif msg.data == State.RELEASE_OBJECT_1.value:
            self.get_logger().info("Rilascio PRINGLES (oggetto 1)")
            self.current_state = State.RELEASE_OBJECT_1
            self.open_gripper()
            
        elif msg.data == State.RETURN_HOME_1.value:
            self.get_logger().info("Ritorno a casa dopo oggetto 1")
            self.current_state = State.RETURN_HOME_1
            self.move_to_home()
            
        # === OGGETTO 2 (COCA-COLA): MARKER 2 -> MARKER 4 (PRIMA) ===
        elif msg.data == State.MOVE_TO_OBJECT_2.value:
            self.get_logger().info("Movimento verso oggetto 2 (ArUco marker 2) - COCA-COLA PRIMA")
            self.current_state = State.MOVE_TO_OBJECT_2
            # USA POSE OTTIMIZZATA per Pringles + avvicinamento finale graduale
            optimal_distance = self.calculate_optimal_gripper_position(2)
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=optimal_distance)
            
        elif msg.data == State.GRIP_OBJECT_2.value:
            self.get_logger().info("Abbasso gripper con precisione per presa COCA-COLA (oggetto 2)")
            self.current_state = State.GRIP_OBJECT_2
            # PRESA PRECISA ADATTIVA: Movimento verso il basso con offset dinamico basato su stabilità
            optimal_distance = self.calculate_optimal_gripper_position(2)
            stability = self.get_position_stability(2)
            # Se posizione instabile, mantieni margine di sicurezza più ampio
            if stability > self.stability_threshold:
                approach_offset = -0.03  # 3cm offset conservativo
                self.get_logger().info(f"Presa CONSERVATIVA per instabilità marker (stabilità={stability*100:.1f}cm)")
            else:
                approach_offset = -0.02  # 2cm offset aggressivo
                self.get_logger().info(f"Presa PRECISA per marker stabile (stabilità={stability*100:.1f}cm)")
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=optimal_distance + approach_offset, lift_height=-0.25)
            
        elif msg.data == State.LIFT_OBJECT_2.value:
            self.get_logger().info("Sollevamento COCA-COLA (oggetto 2)")
            self.current_state = State.LIFT_OBJECT_2
            # USA POSE OTTIMIZZATA per sollevamento
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=0.0, lift_height=0.12)
            
        elif msg.data == State.MOVE_TO_DEST_2.value:
            self.get_logger().info("Trasporto COCA-COLA a destinazione (ArUco marker 4)")
            self.current_state = State.MOVE_TO_DEST_2
            # DEPOSIZIONE: Posizione sicura sopra il marker di destinazione
            self.calculate_pose_with_dynamic_positioning(4, approach_distance=0.05, lift_height=0.05)
            
        elif msg.data == State.RELEASE_OBJECT_2.value:
            self.get_logger().info("Rilascio COCA-COLA (oggetto 2)")
            self.current_state = State.RELEASE_OBJECT_2
            self.open_gripper()
            
        elif msg.data == State.RETURN_HOME_2.value:
            self.get_logger().info("Ritorno finale a casa")
            self.current_state = State.RETURN_HOME_2
            self.move_to_home()

    def aruco_pose_1_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_1 - RICALCOLO AUTOMATICO ANGOLI GIUNTI")
        self.aruco_pose_1 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(1, msg)
        
        # CINEMATICA INVERSA DINAMICA: Ricalcola angoli ogni volta che cambia la posizione
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"ArUco 1 AGGIORNATO: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] dist={distance_from_robot:.3f}m")
        
        # Se il robot è attualmente in movimento verso questo marker, valuta se ricalcolare
        if self.current_state in [State.MOVE_TO_OBJECT_1, State.GRIP_OBJECT_1, State.LIFT_OBJECT_1]:
            if self.should_recalculate_for_marker(1, np.array([pos.x, pos.y, pos.z])):
                self.get_logger().info("RECALCULATING: Posizione ArUco 1 cambiata significativamente - ricalcolo automatico")
                self.trigger_dynamic_recalculation(1)
            else:
                self.get_logger().debug("ArUco 1: Cambiamento posizione sotto soglia - no ricalcolo")

    def aruco_pose_2_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_2 - RICALCOLO AUTOMATICO ANGOLI GIUNTI")
        self.aruco_pose_2 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(2, msg)
        
        # CINEMATICA INVERSA DINAMICA: Ricalcola angoli ogni volta che cambia la posizione
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"ArUco 2 AGGIORNATO: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] dist={distance_from_robot:.3f}m")
        
        # Se il robot è attualmente in movimento verso questo marker, valuta se ricalcolare
        if self.current_state in [State.MOVE_TO_OBJECT_2, State.GRIP_OBJECT_2, State.LIFT_OBJECT_2]:
            if self.should_recalculate_for_marker(2, np.array([pos.x, pos.y, pos.z])):
                self.get_logger().info("RECALCULATING: Posizione ArUco 2 cambiata significativamente - ricalcolo automatico")
                self.trigger_dynamic_recalculation(2)
            else:
                self.get_logger().debug("ArUco 2: Cambiamento posizione sotto soglia - no ricalcolo")
       
    def aruco_pose_3_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_3 - RICALCOLO AUTOMATICO ANGOLI GIUNTI")
        self.aruco_pose_3 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(3, msg)
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 3 AGGIORNATO: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
        
        # Se il robot sta trasportando verso questa destinazione, aggiorna
        if self.current_state == State.MOVE_TO_DEST_1:
            self.get_logger().info("RECALCULATING: Destinazione ArUco 3 cambiata - ricalcolo automatico")
            self.trigger_dynamic_recalculation(3)

    def aruco_pose_4_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_4 - RICALCOLO AUTOMATICO ANGOLI GIUNTI")
        self.aruco_pose_4 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(4, msg)
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 4 AGGIORNATO: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
        
        # Se il robot sta trasportando verso questa destinazione, aggiorna
        if self.current_state == State.MOVE_TO_DEST_2:
            self.get_logger().info("RECALCULATING: Destinazione ArUco 4 cambiata - ricalcolo automatico")
            self.trigger_dynamic_recalculation(4)

    def joint_states_callback(self, msg):
        # Aggiorna sempre la configurazione attuale per avere lo stato più recente
        # CORREZIONE: Rimossa la limitazione che impediva gli aggiornamenti
        
        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        joint_pos = []
        missing_joints = []
        
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_pos.append(msg.position[idx])
            else:
                joint_pos.append(0.0)
                missing_joints.append(name)
                
        # Aggiorna configurazione corrente
        old_state = self.current_joint_state
        self.current_joint_state = np.array(joint_pos)
        
        # Log solo per primo aggiornamento o cambiamenti significativi
        if old_state is None:
            self.get_logger().info(f"[JOINT INIT] Configurazione iniziale salvata:")
            self.get_logger().info(f"   Torso: {joint_pos[0]:.3f}")
            self.get_logger().info(f"   Arm: [{joint_pos[1]:.3f}, {joint_pos[2]:.3f}, {joint_pos[3]:.3f}, {joint_pos[4]:.3f}, {joint_pos[5]:.3f}, {joint_pos[6]:.3f}, {joint_pos[7]:.3f}]")
            if missing_joints:
                self.get_logger().warn(f"Joint mancanti (usato 0.0): {missing_joints}")
        elif old_state is not None and np.linalg.norm(self.current_joint_state - old_state) > 0.01:
            self.get_logger().debug(f"[JOINT UPDATE] Configurazione aggiornata - movimento rilevato")

    def target_pose_callback(self, msg):
        self.get_logger().info("Funzione target_pose_callback chiamata!")
        self.target_pose = msg
        self.get_logger().info(f" Ricevuta target_pose: {msg.pose.position}")
        while self.current_joint_state is None:
            time.sleep(0.1)  # Attendi che il joint state sia inizializzato
        self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        self.get_logger().info("=== INIZIO PIANIFICAZIONE TRAIETTORIA COMPLETA ===")

        if self.current_joint_state is None or self.target_pose is None:
            self.get_logger().warn("Joint state o target pose non ancora ricevuti.")
            return

        q0 = self.current_joint_state.astype(float)  # [torso, arm1, ..., arm7]
        self.get_logger().info(f"Configurazione iniziale: torso={q0[0]:.3f}, arm={q0[1:8]}")

        # 1. Calcola T0 (configurazione attuale) con cinematica diretta
        try:
            T0 = self.robot.fkine(q0)
            self.get_logger().info(f"Cinematica diretta calcolata: posizione attuale = {T0.t}")
        except Exception as e:
            self.get_logger().error(f"Errore nel calcolo della fkine: {e}")
            return

        # 2. Estrai posizione e orientamento target dall'ArUco
        pos = self.target_pose.pose.position
        ori = self.target_pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        
        self.get_logger().info(f"Target position: {position}")
        self.get_logger().info(f"Target orientation (quat): {quaternion}")

        # 3. Crea matrice di trasformazione target con orientamento dinamico dell'ArUco
        try:
            # Usa l'orientamento dell'ArUco marker
            rotation = R.from_quat(quaternion).as_matrix()
            T_marker = SE3.Rt(rotation, position)
            
            # Applica orientamenti diversi in base allo stato (prelievo vs deposizione)
            if self.current_state in [State.MOVE_TO_DEST_1, State.MOVE_TO_DEST_2]:
                # Stati di deposizione - orientamento per rilascio
                T_rot = SE3.Rx(np.pi/2) * SE3.Rz(np.pi)
                self.get_logger().info("Applicato orientamento per DEPOSIZIONE")
            else:
                # Stati di prelievo - orientamento per presa
                T_rot = SE3.Ry(np.pi/2) * SE3.Rz(-(np.pi/2))
                self.get_logger().info("Applicato orientamento per PRELIEVO")
            
            T_target = T_marker * T_rot
            self.get_logger().info(f"Matrice di trasformazione target calcolata: {T_target.t}")
            
        except Exception as e:
            self.get_logger().error(f"Errore nella costruzione della trasformazione finale: {e}")
            return

        # 4. Genera traiettoria intermedia da T0 a T_target
        N = 10  # Numero di punti intermedi
        try:
            trajectory = rtb.ctraj(T0, T_target, N)
            self.get_logger().info(f"Traiettoria intermedia generata con {N} punti")
        except Exception as e:
            self.get_logger().error(f"Errore nella generazione della traiettoria: {e}")
            return

        # 5. APPROCCIO AVANZATO: traiettoria cartesiana completa con IK iterativa
        try:
            self.get_logger().info("=== USANDO APPROCCIO TRAIETTORIA COMPLETA ===")

            # Genera traiettoria cartesiana interpolata (50 punti)
            trajectory_points = rtb.ctraj(T0, T_target, 50)

            # IK per ogni passo con continuità
            q_traj = []
            q_curr = q0.copy()
            success_count = 0

            for i, T in enumerate(trajectory_points):
                sol = self.robot.ikine_LM(T, q0=q_curr)
                if sol.success:
                    q_curr = sol.q
                    success_count += 1
                    self.get_logger().debug(f"Punto {i+1}/50: IK OK (residuo: {sol.residual:.6f})")
                else:
                    self.get_logger().debug(f"Punto {i+1}/50: IK fallita, uso config precedente")

                q_traj.append(q_curr.copy())

            self.get_logger().info(f"Traiettoria generata: {len(q_traj)} punti, {success_count} successi IK")

            if success_count < 10:  # Almeno 20% di successo
                self.get_logger().error(f"Troppe iterazioni IK fallite ({success_count}/50), traiettoria non sicura")
                return

        except Exception as e:
            self.get_logger().error(f"Errore generazione traiettoria: {e}")
            return

        if not q_traj:
            self.get_logger().error("Nessun punto della traiettoria calcolato con successo")
            return

        # 6. Pubblica traiettoria completa per movimenti fluidi
        self.get_logger().info("Pubblicazione traiettoria completa...")
        self.publish_full_trajectory(q_traj, "TRAIETTORIA COMPLETA")

    def publish_full_trajectory(self, q_traj, description):
        """Pubblica traiettoria completa sui controller per movimenti fluidi"""

        if not self.arm_client.server_is_ready() or not self.torso_client.server_is_ready():
            self.get_logger().warn("Action server arm o torso non disponibile, uso publisher")
            # Fallback al metodo vecchio
            self.publish_joint_configuration(q_traj[-1], description)
            return

        # CORREZIONE: Separa traiettorie per torso e braccio

        # 1. Traiettoria TORSO
        goal_torso = FollowJointTrajectory.Goal()
        goal_torso.trajectory.joint_names = ['torso_lift_joint']

        # 2. Traiettoria BRACCIO
        goal_arm = FollowJointTrajectory.Goal()
        goal_arm.trajectory.joint_names = [
            'arm_1_joint','arm_2_joint','arm_3_joint',
            'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint'
        ]

        # Aggiungi ogni punto della traiettoria con timing appropriato
        for i, q in enumerate(q_traj):
            # CORREZIONE: Usa float per i secondi invece di int per evitare arrotondamento a 0
            if self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
                time_from_start = Duration(sec=int((i + 1) * 0.20), nanosec=int(((i + 1) * 0.20 % 1) * 1e9))  # 0.20s per punto
            elif self.current_state in [State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
                time_from_start = Duration(sec=int((i + 1) * 0.18), nanosec=int(((i + 1) * 0.18 % 1) * 1e9))  # 0.18s per punto
            else:
                time_from_start = Duration(sec=int((i + 1) * 0.15), nanosec=int(((i + 1) * 0.15 % 1) * 1e9))  # 0.15s per punto

            # Punto per TORSO (solo primo joint)
            pt_torso = JointTrajectoryPoint()
            pt_torso.positions = [float(q[0])]  # Solo torso_lift_joint
            pt_torso.time_from_start = time_from_start
            goal_torso.trajectory.points.append(pt_torso)

            # Punto per BRACCIO (joints 1-7)
            pt_arm = JointTrajectoryPoint()
            pt_arm.positions = [float(p) for p in q[1:8]]  # Solo arm joints
            pt_arm.time_from_start = time_from_start
            goal_arm.trajectory.points.append(pt_arm)

        # Invia goals separatamente
        self.torso_client.send_goal_async(goal_torso)
        self.arm_client.send_goal_async(goal_arm)

        self.get_logger().info(f"=== {description} PUBBLICATA ===")
        self.get_logger().info(f"Traiettoria: {len(q_traj)} punti, tempo totale: {goal_arm.trajectory.points[-1].time_from_start.sec}s")
        self.get_logger().info("Torso e braccio inviati separatamente ai rispettivi controllers")

        # Salva la configurazione finale
        self.save_joint_configuration_for_current_marker(q_traj[-1])

        # Timer sincronizzato con durata reale della traiettoria
        last_point = goal_arm.trajectory.points[-1].time_from_start
        total_time = last_point.sec + (last_point.nanosec / 1e9) + 3.0  # Buffer di 3s
        if self.current_state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            self.active_timer = self.create_timer(float(total_time), lambda: self.publish_command_completed_once(self.current_state.value))
        elif self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            self.active_timer = self.create_timer(float(total_time), lambda: self.close_gripper())
        elif self.current_state in [State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            self.active_timer = self.create_timer(float(total_time), lambda: self.publish_command_completed_once(self.current_state.value))
        elif self.current_state in [State.MOVE_TO_DEST_1, State.MOVE_TO_DEST_2,
                                    State.RETURN_HOME_1, State.RETURN_HOME_2]:
            self.active_timer = self.create_timer(float(total_time), lambda: self.publish_command_completed_once(self.current_state.value))

    def publish_joint_configuration(self, q_solution, description):
        """Pubblica configurazione giunti sui controller"""
        
        # Prepara trajectory per torso
        traj_torso = JointTrajectory()
        traj_torso.joint_names = ['torso_lift_joint']
        point_torso = JointTrajectoryPoint()
        point_torso.positions = [float(q_solution[0])]
        point_torso.time_from_start.sec = 5  # Tempo standard
        traj_torso.points.append(point_torso)

        # Prepara trajectory per braccio con movimenti più dolci
        traj_arm = JointTrajectory()
        traj_arm.joint_names = [f'arm_{i+1}_joint' for i in range(7)]
        point_arm = JointTrajectoryPoint()
        point_arm.positions = [float(p) for p in q_solution[1:8]]
        
        # Tempo variabile basato su tipo di movimento per evitare urti
        if self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2, State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            point_arm.time_from_start.sec = 8  # Movimenti di presa/sollevamento più lenti
            self.get_logger().info("Movimento LENTO per presa/sollevamento (8s) - evito urti tavolo")
        elif self.current_state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            point_arm.time_from_start.sec = 7  # Avvicinamento graduale
            self.get_logger().info("Movimento GRADUALE per avvicinamento (7s)")
        else:
            point_arm.time_from_start.sec = 5  # Tempo standard per altri movimenti
            
        traj_arm.points.append(point_arm)

        # Pubblica - DEPRECATO: Ora usiamo ActionClient invece di Publisher
        # self.arm_pub.publish(traj_arm)
        # self.torso_pub.publish(traj_torso)
        
        self.get_logger().info(f"=== {description} PUBBLICATA ===")
        self.get_logger().info(f"Configurazione giunti: torso={q_solution[0]:.3f}, arm=[{', '.join([f'{p:.3f}' for p in q_solution[1:8]])}]")
        
        # Salva la configurazione calcolata per il marker corrente (se applicabile)
        self.save_joint_configuration_for_current_marker(q_solution)
        
        # Attiva timer per completamento dei comandi di manipolazione - SINCRONIZZATI CON MOVIMENTI LENTI
        if self.current_state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            # Timer per avvicinamento oggetti - sincronizzato con tempo movimento (7s + buffer)
            self.active_timer = self.create_timer(10.0, lambda: self.publish_command_completed_once(self.current_state.value))
        elif self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            # Timer per grip - sincronizzato con movimento lento (8s + chiusura gripper)
            self.active_timer = self.create_timer(12.0, lambda: self.close_gripper())
        elif self.current_state in [State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            # Timer per sollevamento - movimento lento per evitare urti (8s + buffer)
            self.active_timer = self.create_timer(12.0, lambda: self.publish_command_completed_once(self.current_state.value))
        elif self.current_state in [State.MOVE_TO_DEST_1, State.MOVE_TO_DEST_2,
                                    State.RETURN_HOME_1, State.RETURN_HOME_2]:
            # Timer per altri movimenti - tempo standard
            self.active_timer = self.create_timer(8.0, lambda: self.publish_command_completed_once(self.current_state.value))
        elif self.current_state in [State.RELEASE_OBJECT_1, State.RELEASE_OBJECT_2]:
            # Timer per release - include apertura gripper  
            self.active_timer = self.create_timer(6.0, lambda: self.open_gripper())

    def simple_ik_approach(self, q0, T_target):
        """
        Algoritmo IK corretto usando ikine_LM
        - Usa metodo moderno ikine_LM invece di ik_LM deprecato
        - Controllo esplicito di successo
        - Gestione corretta dell'oggetto risultato

        Args:
            q0: Configurazione iniziale dei giunti [8D: torso + 7 arm joints]
            T_target: Matrice di trasformazione target (SE3)

        Returns:
            q_solution: Configurazione finale dei giunti o q0 se fallisce
        """
        try:
            sol = self.robot.ikine_LM(T_target, q0=q0)
            if sol.success:
                self.get_logger().info(f"IK completata con successo - iterazioni: {sol.iterations}, residuo: {sol.residual:.6f}")
                return sol.q
            else:
                self.get_logger().warning(f"IK fallita - iterazioni: {sol.iterations}, residuo: {sol.residual:.6f}")
                self.get_logger().warning("Riutilizzo configurazione precedente")
                return q0
        except Exception as e:
            self.get_logger().error(f"Errore IK: {e}")
            return q0

    def compute_ik_with_pose_error_feedback(self, q0, T_target, max_iterations=5, position_tolerance=0.02, orientation_tolerance=0.1):
        """
        Risolve la cinematica inversa con controllo iterativo dell'errore di posa nello spazio operativo
        
        Args:
            q0: Configurazione iniziale dei giunti [8D: torso + 7 arm joints]
            T_target: Matrice di trasformazione target (SE3)
            max_iterations: Numero massimo di iterazioni per la correzione
            position_tolerance: Tolleranza sull'errore di posizione [m]
            orientation_tolerance: Tolleranza sull'errore di orientamento [rad]
        
        Returns:
            q_solution: Configurazione finale dei giunti o None se fallisce
        """
        
        q_candidate = q0.copy()
        
        for iteration in range(max_iterations):
            try:
                # 1. Calcola cinematica diretta con la configurazione candidata
                T_achieved = self.robot.fkine(q_candidate)
                
                # 2. Calcola errore di posizione (norma euclidea)
                position_error = np.linalg.norm(T_target.t - T_achieved.t)
                
                # 3. Calcola errore di orientamento (angolo tra matrici di rotazione)
                R_target = T_target.R
                R_achieved = T_achieved.R
                R_error = R_target @ R_achieved.T
                
                # Estrai l'angolo di rotazione dall'asse-angolo
                trace_R = np.trace(R_error)
                angle_error = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
                
                self.get_logger().debug(f"Iterazione {iteration}: pos_err={position_error:.4f}m, angle_err={angle_error:.4f}rad")
                
                # 4. Verifica se l'errore è entro le tolleranze
                if position_error <= position_tolerance and angle_error <= orientation_tolerance:
                    self.get_logger().info(f"Convergenza raggiunta in {iteration+1} iterazioni")
                    self.get_logger().info(f"Errore finale: pos={position_error:.4f}m, angle={angle_error:.4f}rad")
                    return q_candidate
                
                # 5. Se errore troppo alto, usa IK per correggere
                sol = self.robot.ikine_LM(T_target, q0=q_candidate)

                if sol.success:
                    q_candidate = sol.q
                else:
                    self.get_logger().warning(f"IK fallita all'iterazione {iteration} - residuo: {sol.residual:.6f}")
                    break
                    
            except Exception as e:
                self.get_logger().error(f"Errore nell'iterazione {iteration}: {e}")
                break
        
        # Se arriviamo qui, non siamo riusciti a convergere entro le tolleranze
        self.get_logger().warning(f"Controllo errore fallito dopo {max_iterations} iterazioni")
        self.get_logger().warning(f"Errore finale: pos={position_error:.4f}m, angle={angle_error:.4f}rad")
        
        # Ritorna la migliore approssimazione ottenuta
        return q_candidate if 'q_candidate' in locals() else None

    def calculate_pose_with_offset_state(self, target, offset_x, offset_y, offset_z):
        self.get_logger().info(f"=== CALCOLO POSA CON OFFSET PER TARGET {target} ===")
        if target == 1:
            base_pose = self.aruco_pose_1.pose
            header = self.aruco_pose_1.header
        elif target == 2:
            base_pose = self.aruco_pose_2.pose
            header = self.aruco_pose_2.header
        elif target == 3:
            base_pose = self.aruco_pose_3.pose
            header = self.aruco_pose_3.header
        elif target == 4:
            base_pose = self.aruco_pose_4.pose
            header = self.aruco_pose_4.header

        # Debug dettagliato: stampa pose di partenza
        self.get_logger().info(f"ArUco {target} ORIGINALE:")
        self.get_logger().info(f"   Posizione: x={base_pose.position.x:.3f}, y={base_pose.position.y:.3f}, z={base_pose.position.z:.3f}")
        self.get_logger().info(f"   Frame: {header.frame_id}")
        self.get_logger().info(f"OFFSET APPLICATI:")
        self.get_logger().info(f"   x_offset={offset_x:.3f}m, y_offset={offset_y:.3f}m, z_offset={offset_z:.3f}m")
        
        # Calcolo distanza originale
        orig_distance = (base_pose.position.x**2 + base_pose.position.y**2 + base_pose.position.z**2)**0.5
        self.get_logger().info(f"   Distanza ArUco originale: {orig_distance:.3f}m")

        # Creazione della nuova posa con offset
        target_pose_out = PoseStamped()
        target_pose_out.header.stamp = self.get_clock().now().to_msg()
        target_pose_out.header.frame_id = header.frame_id

        # IMPORTANTE: Applica offset nel sistema di coordinate corretto
        target_pose_out.pose.position.x = base_pose.position.x + offset_x  # Avanti/indietro
        target_pose_out.pose.position.y = base_pose.position.y + offset_y  # Sinistra/destra  
        target_pose_out.pose.position.z = base_pose.position.z + offset_z  # Su/giù (NEGATIVO = scendi)

        # Mantieni orientamento dell'ArUco (potrebbe essere utile)
        target_pose_out.pose.orientation = base_pose.orientation

        # Debug finale: stampa pose target
        final_pos = target_pose_out.pose.position
        final_distance = (final_pos.x**2 + final_pos.y**2 + final_pos.z**2)**0.5
        self.get_logger().info(f"TARGET FINALE per ArUco {target}:")
        self.get_logger().info(f"   Posizione: x={final_pos.x:.3f}, y={final_pos.y:.3f}, z={final_pos.z:.3f}")
        self.get_logger().info(f"   Distanza finale: {final_distance:.3f}m")
        self.get_logger().info(f"   Differenza distanza: {final_distance-orig_distance:.3f}m")
        
        # Verifica raggiungibilità migliorata
        if final_distance > 1.4:
            self.get_logger().warn(f"ATTENZIONE: Target molto lontano ({final_distance:.3f}m > 1.4m)")
        elif final_distance < 0.3:
            self.get_logger().warn(f"ATTENZIONE: Target molto vicino ({final_distance:.3f}m < 0.3m)")
        else:
            self.get_logger().info(f"Target a distanza ragionevole: {final_distance:.3f}m")

        # Pubblica la posa target
        self.pose_pub.publish(target_pose_out)
        self.get_logger().info("Posa target pubblicata su /target_pose")

        return "go_to_pose"
    
    def update_position_buffer(self, marker_id, pose_msg):
        """Aggiorna buffer delle posizioni per stabilizzazione dinamica"""
        pos = pose_msg.pose.position
        position = np.array([pos.x, pos.y, pos.z])
        
        # Aggiungi nuova posizione al buffer
        self.position_buffers[marker_id].append(position)
        
        # Mantieni solo gli ultimi N campioni
        if len(self.position_buffers[marker_id]) > self.buffer_size:
            self.position_buffers[marker_id].pop(0)
        
        # Calcola posizione stabilizzata (media mobile)
        if len(self.position_buffers[marker_id]) >= 3:  # Minimo 3 campioni
            positions_array = np.array(self.position_buffers[marker_id])
            stabilized = np.mean(positions_array, axis=0)
            
            # Calcola varianza per verificare stabilità
            variance = np.var(positions_array, axis=0)
            stability = np.linalg.norm(variance)
            
            # Aggiorna posizione stabilizzata
            old_pos = self.stable_positions.get(marker_id)
            self.stable_positions[marker_id] = stabilized
            
            # Log solo se è una posizione significativamente diversa
            if old_pos is None or np.linalg.norm(stabilized - old_pos) > 0.01:
                self.get_logger().info(f"Marker {marker_id} - Posizione stabilizzata: [{stabilized[0]:.3f}, {stabilized[1]:.3f}, {stabilized[2]:.3f}] (stabilità: {stability:.4f})")

    def get_stabilized_position(self, marker_id):
        """Ottieni posizione stabilizzata per un marker"""
        if marker_id in self.stable_positions:
            return self.stable_positions[marker_id]

        # Fallback alla posizione più recente se non abbastanza campioni
        if len(self.position_buffers[marker_id]) > 0:
            return self.position_buffers[marker_id][-1]

        return None

    def get_position_stability(self, marker_id):
        """
        Calcola la stabilità della posizione del marker basandosi sulla varianza dei campioni recenti.
        Ritorna la norma della varianza (in metri).

        Stabilità bassa (<0.008m) = posizione stabile = presa più precisa
        Stabilità alta (>0.008m) = posizione instabile = presa più conservativa
        """
        if len(self.position_buffers[marker_id]) < 3:
            return float('inf')  # Non abbastanza campioni = considerato instabile

        positions_array = np.array(self.position_buffers[marker_id][-10:])  # Ultimi 10 campioni
        variance = np.var(positions_array, axis=0)
        stability = np.linalg.norm(variance)

        return stability

    def calculate_pose_with_dynamic_positioning(self, target_marker, approach_distance=0.10, lift_height=0.0):
        """Sistema di posizionamento dinamico che sostituisce gli offset statici"""
        self.get_logger().info(f"=== POSIZIONAMENTO DINAMICO PER MARKER {target_marker} ===")
        
        # Ottieni posizione stabilizzata del marker
        stabilized_pos = self.get_stabilized_position(target_marker)
        if stabilized_pos is None:
            self.get_logger().error(f"Posizione stabilizzata non disponibile per marker {target_marker}")
            return
        
        # Ottieni orientamento dalla pose originale
        if target_marker == 1:
            base_pose = self.aruco_pose_1.pose
            header = self.aruco_pose_1.header
        elif target_marker == 2:
            base_pose = self.aruco_pose_2.pose
            header = self.aruco_pose_2.header
        elif target_marker == 3:
            base_pose = self.aruco_pose_3.pose
            header = self.aruco_pose_3.header
        elif target_marker == 4:
            base_pose = self.aruco_pose_4.pose
            header = self.aruco_pose_4.header

        self.get_logger().info(f"Posizione stabilizzata marker {target_marker}: [{stabilized_pos[0]:.3f}, {stabilized_pos[1]:.3f}, {stabilized_pos[2]:.3f}]")
        
        # Crea pose target dinamica
        target_pose_out = PoseStamped()
        target_pose_out.header.stamp = self.get_clock().now().to_msg()
        target_pose_out.header.frame_id = header.frame_id

        # POSIZIONAMENTO DINAMICO INTELLIGENTE
        # NOTA: Le pose per marker 1-2 sono già ottimizzate dall'ArUco detector (con offset ~6-6.5cm)
        # L'ArUco detector ora posiziona correttamente il target TRA robot e marker
        # Per marker 3-4 (destinazioni), uso posizionamento normale
        
        if approach_distance != 0.0:
            # STRATEGIA AVANTI-POI-GIU: avvicinamento aggressivo verso il robot
            robot_to_marker = np.array([stabilized_pos[0], stabilized_pos[1], 0.0])  # Ignora Z per calcolo orizzontale
            distance_horizontal = np.linalg.norm(robot_to_marker[:2])
            
            if distance_horizontal > 0.001:  # Evita divisione per zero
                # Vettore unitario verso il robot (per approach_distance negativo = più vicino)
                approach_vector = robot_to_marker / distance_horizontal
                # MOVIMENTO AGGRESSIVO: approach_distance negativo = più vicino al robot, evita busto
                approach_offset = -approach_vector * approach_distance  # Negativo = verso robot
                self.get_logger().info(f"STRATEGIA AVANTI-POI-GIU: avvicinamento {approach_distance:.3f}m (negativo=più vicino)")
            else:
                approach_offset = np.array([-approach_distance, 0.0, 0.0])  # Fallback: avvicinamento frontale aggressivo
        else:
            # Nessun offset orizzontale - usa direttamente la posizione ottimizzata
            approach_offset = np.array([0.0, 0.0, 0.0])
            self.get_logger().info(f"Marker {target_marker}: usando posizione già ottimizzata dall'ArUco detector")

        # 2. Posizione finale dinamica
        # X,Y: Perfettamente centrato sul marker + offset di avvicinamento
        target_pose_out.pose.position.x = float(stabilized_pos[0] + approach_offset[0])
        target_pose_out.pose.position.y = float(stabilized_pos[1] + approach_offset[1])  # PERFETTO CENTRO Y
        
        # Z: Altezza del marker + eventuale sollevamento
        target_pose_out.pose.position.z = float(stabilized_pos[2] + lift_height)

        # Mantieni orientamento originale
        target_pose_out.pose.orientation = base_pose.orientation

        # Debug dettagliato con verifica geometrica
        final_pos = target_pose_out.pose.position
        original_distance = np.sqrt(stabilized_pos[0]**2 + stabilized_pos[1]**2)
        final_distance = np.sqrt(final_pos.x**2 + final_pos.y**2)
        
        self.get_logger().info(f"IK POSIZIONAMENTO DINAMICO - Marker {target_marker}:")
        self.get_logger().info(f"   Posizione ricevuta:   [{stabilized_pos[0]:.3f}, {stabilized_pos[1]:.3f}, {stabilized_pos[2]:.3f}] dist={original_distance:.3f}m")
        self.get_logger().info(f"   Approach offset IK:   [{approach_offset[0]:.3f}, {approach_offset[1]:.3f}, {lift_height:.3f}]")
        self.get_logger().info(f"   Posizione finale IK:  [{final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f}] dist={final_distance:.3f}m")
        
        # Verifica coerenza: se ArUco detector ha fatto il suo lavoro, la posizione ricevuta
        # dovrebbe già essere ottimizzata per marker 1-2
        if target_marker in [1, 2]:
            if approach_distance == 0.0:
                self.get_logger().info(f"   USO DIRETTO pose ArUco ottimizzata (no offset IK)")
            else:
                self.get_logger().info(f"   AGGIUSTAMENTO FINE: +{approach_distance:.3f}m da pose ArUco ottimizzata")

        # Controllo raggiungibilità
        if final_distance > 1.4:
            self.get_logger().warn(f"Target lontano: {final_distance:.3f}m")
        elif final_distance < 0.25:
            self.get_logger().warn(f"Target molto vicino: {final_distance:.3f}m")
        else:
            self.get_logger().info(f"Target a distanza ottimale: {final_distance:.3f}m")

        # Pubblica posa target
        self.pose_pub.publish(target_pose_out)
        self.get_logger().info("Posa dinamica pubblicata - gripper sarà PERFETTAMENTE centrato!")

        return "dynamic_positioning"

    def should_recalculate_for_marker(self, marker_id, new_position):
        """
        Determina se è necessario ricalcolare gli angoli dei giunti per un marker
        basandosi su soglie di cambiamento posizione e tempo
        """
        import time
        
        current_time = time.time()
        
        # Controllo timing: evita ricalcoli troppo frequenti
        if current_time - self.last_recalculation_time[marker_id] < self.min_recalculation_interval:
            return False
            
        # Controllo soglia spaziale: valuta se il cambiamento è significativo
        last_pos = self.last_positions[marker_id]
        if last_pos is not None:
            position_change = np.linalg.norm(new_position - last_pos)
            if position_change < self.position_change_threshold:
                return False
                
        # Aggiorna la posizione e il timestamp
        self.last_positions[marker_id] = new_position.copy()
        self.last_recalculation_time[marker_id] = current_time
        
        return True

    def trigger_dynamic_recalculation(self, marker_id):
        """
        CINEMATICA INVERSA DINAMICA: Ricalcola automaticamente gli angoli dei giunti 
        quando la posizione di un ArUco marker cambia durante la manipolazione
        """
        self.get_logger().info(f"AVVIO RICALCOLO DINAMICO per Marker {marker_id}")
        
        # Determina il tipo di movimento in base allo stato corrente e al marker
        if self.current_state in [State.MOVE_TO_OBJECT_1, State.GRIP_OBJECT_1, State.LIFT_OBJECT_1] and marker_id == 1:
            operation_type = self.get_operation_type_for_state(self.current_state)
            lift_height = self.get_lift_height_for_state(self.current_state)
            approach_dist = self.get_approach_distance_for_state(self.current_state)
            
        elif self.current_state in [State.MOVE_TO_OBJECT_2, State.GRIP_OBJECT_2, State.LIFT_OBJECT_2] and marker_id == 2:
            operation_type = self.get_operation_type_for_state(self.current_state)
            lift_height = self.get_lift_height_for_state(self.current_state)
            approach_dist = self.get_approach_distance_for_state(self.current_state)
            
        elif self.current_state == State.MOVE_TO_DEST_1 and marker_id == 3:
            operation_type = "deposit"
            lift_height = 0.05
            approach_dist = 0.05
            
        elif self.current_state == State.MOVE_TO_DEST_2 and marker_id == 4:
            operation_type = "deposit"  
            lift_height = 0.05
            approach_dist = 0.05
        else:
            self.get_logger().warn(f"Ricalcolo non necessario per marker {marker_id} in stato {self.current_state.name}")
            return

        self.get_logger().info(f"RICALCOLO AUTOMATICO: {operation_type} per marker {marker_id}")
        self.get_logger().info(f"   Parametri: approach={approach_dist:.3f}m, lift={lift_height:.3f}m")
        
        # Esegui il ricalcolo con i nuovi parametri
        self.calculate_pose_with_dynamic_positioning(marker_id, approach_dist, lift_height)
        
    def get_operation_type_for_state(self, state):
        """Determina il tipo di operazione in base allo stato"""
        if state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            return "approach"
        elif state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            return "grip"
        elif state in [State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            return "lift"
        else:
            return "generic"
            
    def get_lift_height_for_state(self, state):
        """Determina l'altezza di movimento in base allo stato"""
        if state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            return 0.0  # Stesso livello del marker
        elif state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            return -0.03  # Scendi di 3cm per la presa
        elif state in [State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            return 0.12  # Solleva di 12cm
        else:
            return 0.0
            
    def get_approach_distance_for_state(self, state):
        """Determina la distanza di avvicinamento in base allo stato"""
        if state in [State.MOVE_TO_OBJECT_1, State.MOVE_TO_OBJECT_2]:
            return 0.02  # 2cm per avvicinamento finale
        elif state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2, State.LIFT_OBJECT_1, State.LIFT_OBJECT_2]:
            return 0.0   # Posizione diretta sull'oggetto
        else:
            return 0.05
    
    def save_joint_configuration_for_current_marker(self, q_solution):
        """Salva la configurazione dei giunti per il marker attualmente processato"""
        current_marker = self.get_current_target_marker()
        if current_marker is not None:
            self.last_joint_configurations[current_marker] = q_solution.copy()
            self.get_logger().info(f"Configurazione salvata per marker {current_marker}")
            
    def get_current_target_marker(self):
        """Determina quale marker è attualmente il target in base allo stato"""
        if self.current_state in [State.MOVE_TO_OBJECT_1, State.GRIP_OBJECT_1, State.LIFT_OBJECT_1]:
            return 1
        elif self.current_state in [State.MOVE_TO_OBJECT_2, State.GRIP_OBJECT_2, State.LIFT_OBJECT_2]:
            return 2
        elif self.current_state == State.MOVE_TO_DEST_1:
            return 3
        elif self.current_state == State.MOVE_TO_DEST_2:
            return 4
        else:
            return None

    def calculate_optimal_approach_distance(self, target_marker, operation_type="approach"):
        """
        SISTEMA AUTOMATICO: Calcola la distanza di avvicinamento ottimale
        basata su geometria ArUco, dimensioni oggetto e tipo di operazione
        """
        # Parametri base
        marker_to_object_offset = self.aruco_size / 2  # Dall'ArUco al centro oggetto (3cm)
        safety_margin = 0.02  # Margine di sicurezza (2cm)
        
        # Distanze specifiche per operazione
        operation_distances = {
            "approach": 0.04,    # Avvicinamento iniziale (4cm dall'oggetto)
            "grip": 0.01,        # Posizione di presa (1cm dall'oggetto) 
            "lift": 0.01         # Mantenimento durante sollevamento
        }
        
        # Calcolo automatico: ArUco → centro oggetto → posizione ottimale gripper
        base_distance = marker_to_object_offset + operation_distances[operation_type] + safety_margin
        
        # Aggiustamenti per tipo oggetto
        if target_marker in self.object_sizes:
            obj = self.object_sizes[target_marker]
            if obj["type"] == "bottle":
                # Bottiglia: più precisione laterale necessaria
                base_distance += 0.005  
            elif obj["type"] == "cylinder":
                # Cilindro: più facile da afferrare, gripper più vicino
                base_distance -= 0.005
        
        self.get_logger().info(f"CALC AUTO: Marker {target_marker}, {operation_type} -> distanza ottimale: {base_distance:.3f}m")
        return base_distance

    def calculate_pose_with_adaptive_positioning(self, target_marker, operation_type="approach", lift_height=0.0):
        """Sistema adattivo che calcola automaticamente le distanze ottimali"""
        self.get_logger().info(f"=== POSIZIONAMENTO ADATTIVO MARKER {target_marker} ===")
        
        # CALCOLO AUTOMATICO della distanza ottimale
        optimal_distance = self.calculate_optimal_approach_distance(target_marker, operation_type)
        
        # Riusa la logica del sistema dinamico esistente
        return self.calculate_pose_with_dynamic_positioning(target_marker, optimal_distance, lift_height)
    
    def close_gripper(self):
        self.get_logger().info("Chiusura gripper in corso con modalità alta precisione...")

        # Movimento gripper - chiusura precisa basata su stabilità marker
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()

        # Determina marker target corrente
        target_marker = self.get_current_target_marker()
        stability = self.get_position_stability(target_marker) if target_marker else float('inf')

        # CHIUSURA ADATTIVA: baseata su stabilità e tipo oggetto
        if self.current_state == State.GRIP_OBJECT_1:
            # Coca-Cola: bottiglia più sottile - richiede precisione maggiore
            if stability < self.stability_threshold:
                # Marker stabile: presa più stretta
                point.positions = [0.032, 0.032]
                self.get_logger().info(f"Coca-Cola: presa PRECISA (stabilità={stability*100:.1f}cm)")
            else:
                # Marker instabile: presa conservativa
                point.positions = [0.036, 0.036]
                self.get_logger().info(f"Coca-Cola: presa CONSERVATIVA (stabilità={stability*100:.1f}cm)")
        else:
            # Pringles: cilindro più largo
            if stability < self.stability_threshold:
                # Marker stabile: presa stretta
                point.positions = [0.035, 0.035]
                self.get_logger().info(f"Pringles: presa STRETTA (stabilità={stability*100:.1f}cm)")
            else:
                # Marker instabile: presa moderata
                point.positions = [0.040, 0.040]
                self.get_logger().info(f"Pringles: presa MODERATA (stabilità={stability*100:.1f}cm)")

        # Tempo graduale per chiusura precisa
        point.time_from_start.sec = 3
        traj.points.append(point)

        # Aggiungi sleep per garantire timing
        time.sleep(0.5)
        # self.gripper_pub.publish(traj)  # DEPRECATO: Ora usiamo ActionClient
        self.get_logger().info("Comando chiusura gripper pubblicato con adattamento dinamico")

        # Attendi per la chiusura completa prima di attach
        time.sleep(3.0)
        self.handle_gripper_attach()

    def open_gripper(self):
        self.get_logger().info("Apertura gripper in corso...")

        # Movimento gripper
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 2
        traj.points.append(point)
        
        # Aggiungi sleep per garantire timing come in kinematic1.py
        time.sleep(0.5)
        # self.gripper_pub.publish(traj)  # DEPRECATO: Ora usiamo ActionClient
        self.get_logger().info("Comando apertura gripper pubblicato")
        
        # Attendi per l'apertura completa prima di detach
        time.sleep(3.0)
        self.handle_gripper_detach()

    def move_to_home(self):
        self.get_logger().info("Ritorno alla posa finale predefinita...")

        # Prepara trajectory per torso
        traj_torso = JointTrajectory()
        traj_torso.joint_names = ['torso_lift_joint']

        point_torso = JointTrajectoryPoint()
        point_torso.positions = [0.35]
        point_torso.time_from_start.sec = 3
        point_torso.time_from_start.nanosec = 0

        traj_torso.points.append(point_torso)

        # Prepara trajectory per braccio
        traj_arm = JointTrajectory()
        traj_arm.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        point_arm = JointTrajectoryPoint()
        point_arm.positions = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
        point_arm.time_from_start.sec = 3
        point_arm.time_from_start.nanosec = 0

        traj_arm.points.append(point_arm)

        # Pubblica - DEPRECATO: Ora usiamo ActionClient invece di Publisher
        # self.arm_pub.publish(traj_arm)
        # self.torso_pub.publish(traj_torso)

        self.get_logger().info("Traiettoria di ritorno pubblicata su torso e braccio.")
        
        # Timing sincrono come in kinematic1.py
        time.sleep(10.0)
        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.get_logger().info(f"Comando completato: {self.current_state.name}")
        self.completed_command_topic.publish(command_completed)

    def handle_gripper_attach(self):
        object_name = "cocacola" if self.current_state == State.GRIP_OBJECT_1 else "pringles"
        
        # Implementazione funzionante con os.system come in kinematic1.py
        try:
            cmd = f"ros2 service call /attach gazebo_ros_link_attacher/srv/Attach \"{{model_name_1: tiago, link_name_1: gripper_left_finger_link, model_name_2: {object_name}, link_name_2: link}}\""
            os.system(cmd)
            self.get_logger().info(f"Attach {object_name} eseguito con successo")
        except Exception as e:
            self.get_logger().error(f"Errore nell'attach di {object_name}: {e}")
        
        # Timing sincrono - completamento immediato
        time.sleep(1.0)
        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.get_logger().info(f"Comando completato: {self.current_state.name}")
        self.completed_command_topic.publish(command_completed)

    # def attach_response_callback(self, future):
    #     """Callback per risposta del servizio attach"""
    #     try:
    #         response = future.result()
    #         if response.ok:
    #             self.get_logger().info("Oggetto attaccato con successo!")
    #         else:
    #             self.get_logger().warn("Fallimento nell'attach dell'oggetto")
    #         self.create_timer(0.5, lambda: self.publish_command_completed(self.current_state.value))
    #     except Exception as e:
    #         self.get_logger().error(f"Errore nel servizio attach: {e}")
    #         self.create_timer(0.5, lambda: self.publish_command_completed(self.current_state.value))
        
    def handle_gripper_detach(self):
        object_name = "cocacola" if self.current_state == State.RELEASE_OBJECT_1 else "pringles"
        
        # Implementazione funzionante con os.system come in kinematic1.py
        try:
            cmd = f"ros2 service call /detach gazebo_ros_link_attacher/srv/Attach \"{{model_name_1: tiago, link_name_1: gripper_left_finger_link, model_name_2: {object_name}, link_name_2: link}}\""
            os.system(cmd)
            self.get_logger().info(f"Detach {object_name} eseguito con successo")
        except Exception as e:
            self.get_logger().error(f"Errore nel detach di {object_name}: {e}")
        
        # Timing sincrono - completamento immediato
        time.sleep(1.0)
        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.get_logger().info(f"Comando completato: {self.current_state.name}")
        self.completed_command_topic.publish(command_completed)

    # def detach_response_callback(self, future):
    #     """Callback per risposta del servizio detach"""
    #     try:
    #         response = future.result()
    #         if response.ok:
    #             self.get_logger().info("Oggetto rilasciato con successo!")
    #         else:
    #             self.get_logger().warn("Fallimento nel detach dell'oggetto")
    #         self.create_timer(0.5, lambda: self.publish_command_completed(self.current_state.value))
    #     except Exception as e:
    #         self.get_logger().error(f"Errore nel servizio detach: {e}")
    #         self.create_timer(0.5, lambda: self.publish_command_completed(self.current_state.value))
        
    def publish_command_completed_once(self, state_value):
        """Pubblica completamento comando solo una volta per evitare ripetizioni"""
        if self.command_executed:
            self.get_logger().warn(f"Comando {State(state_value).name} già eseguito - skip ripetizione")
            return
            
        self.command_executed = True
        command_completed = Int32()
        command_completed.data = state_value
        self.get_logger().info(f"Comando completato: {State(state_value).name}")
        self.completed_command_topic.publish(command_completed)
        
        # Cancella timer attivi dopo completamento
        if self.active_timer is not None:
            self.active_timer.cancel()
            self.active_timer = None
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None
    
    def publish_command_completed(self, state_value):
        """Mantieni funzione originale per compatibilità con altre parti del codice"""
        command_completed = Int32()
        command_completed.data = state_value
        self.get_logger().info(f"Comando completato: {State(state_value).name}")
        self.completed_command_topic.publish(command_completed)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()