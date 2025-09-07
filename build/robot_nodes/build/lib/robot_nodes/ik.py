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

        # Publishers per i controller (arm e torso)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
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
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

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

        # Le configurazioni saranno inviate SOLO quando la state machine lo richiede
        # NON più automatiche con timer
        self.get_logger().info("IK Node pronto - aspettando comandi dalla State Machine")

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
        self.active_timer = self.create_timer(3.5, lambda: self.publish_command_completed(State.INTERMEDIATE_CONFIG.value))

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
            pt.positions = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
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
        self.active_timer = self.create_timer(4.0, lambda: self.publish_command_completed(State.OPERATIONAL_CONFIG.value))

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
            
        # === OGGETTO 1 (COCA-COLA): MARKER 1 -> MARKER 3 ===
        elif msg.data == State.MOVE_TO_OBJECT_1.value:
            self.get_logger().info("Movimento verso oggetto 1 (ArUco marker 1)")
            self.current_state = State.MOVE_TO_OBJECT_1
            # 🎯 USA POSE OTTIMIZZATA calcolata dall'ArUco Detector
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=0.0)  # No offset, usa pose ottimizzata
            
        elif msg.data == State.GRIP_OBJECT_1.value:
            self.get_logger().info("Abbasso gripper per presa oggetto 1")
            self.current_state = State.GRIP_OBJECT_1
            # 🎯 USA POSE OTTIMIZZATA per presa precisa
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=0.0, lift_height=-0.05)  # Scendi di 5cm
            
        elif msg.data == State.LIFT_OBJECT_1.value:
            self.get_logger().info("Sollevamento oggetto 1")  
            self.current_state = State.LIFT_OBJECT_1
            # 🎯 USA POSE OTTIMIZZATA per sollevamento
            self.calculate_pose_with_dynamic_positioning(1, approach_distance=0.0, lift_height=0.12)  # Solleva 12cm
            
        elif msg.data == State.MOVE_TO_DEST_1.value:
            self.get_logger().info("Trasporto oggetto 1 a destinazione (ArUco marker 3)")
            self.current_state = State.MOVE_TO_DEST_1
            # NUOVO: Deposizione dinamica per marker 3
            self.calculate_pose_with_dynamic_positioning(3, approach_distance=0.08, lift_height=0.08)
            
        elif msg.data == State.RELEASE_OBJECT_1.value:
            self.get_logger().info("Rilascio oggetto 1")
            self.current_state = State.RELEASE_OBJECT_1
            self.open_gripper()
            
        elif msg.data == State.RETURN_HOME_1.value:
            self.get_logger().info("Ritorno a casa dopo oggetto 1")
            self.current_state = State.RETURN_HOME_1
            self.move_to_home()
            
        # === OGGETTO 2 (PRINGLES): MARKER 2 -> MARKER 4 ===
        elif msg.data == State.MOVE_TO_OBJECT_2.value:
            self.get_logger().info("Movimento verso oggetto 2 (ArUco marker 2)")
            self.current_state = State.MOVE_TO_OBJECT_2
            # 🎯 USA POSE OTTIMIZZATA per Pringles
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=0.0)
            
        elif msg.data == State.GRIP_OBJECT_2.value:
            self.get_logger().info("Abbasso gripper per presa oggetto 2")
            self.current_state = State.GRIP_OBJECT_2
            # 🎯 USA POSE OTTIMIZZATA per presa cilindro
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=0.0, lift_height=-0.05)
            
        elif msg.data == State.LIFT_OBJECT_2.value:
            self.get_logger().info("Sollevamento oggetto 2")
            self.current_state = State.LIFT_OBJECT_2
            # 🎯 USA POSE OTTIMIZZATA per sollevamento
            self.calculate_pose_with_dynamic_positioning(2, approach_distance=0.0, lift_height=0.12)
            
        elif msg.data == State.MOVE_TO_DEST_2.value:
            self.get_logger().info("Trasporto oggetto 2 a destinazione (ArUco marker 4)")
            self.current_state = State.MOVE_TO_DEST_2
            # NUOVO: Deposizione dinamica per marker 4
            self.calculate_pose_with_dynamic_positioning(4, approach_distance=0.08, lift_height=0.08)
            
        elif msg.data == State.RELEASE_OBJECT_2.value:
            self.get_logger().info("Rilascio oggetto 2")
            self.current_state = State.RELEASE_OBJECT_2
            self.open_gripper()
            
        elif msg.data == State.RETURN_HOME_2.value:
            self.get_logger().info("Ritorno finale a casa")
            self.current_state = State.RETURN_HOME_2
            self.move_to_home()

    def aruco_pose_1_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_1")
        self.aruco_pose_1 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(1, msg)
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"ArUco 1: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] ori=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}] dist={distance_from_robot:.3f}m frame={msg.header.frame_id}")

    def aruco_pose_2_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_2")
        self.aruco_pose_2 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(2, msg)
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"ArUco 2: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] ori=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}] dist={distance_from_robot:.3f}m frame={msg.header.frame_id}")
       
    def aruco_pose_3_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_3")
        self.aruco_pose_3 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(3, msg)
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 3 pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} in frame {msg.header.frame_id}")

    def aruco_pose_4_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_4")
        self.aruco_pose_4 = msg
        # Aggiorna buffer per posizionamento dinamico
        self.update_position_buffer(4, msg)
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 4 pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} in frame {msg.header.frame_id}")

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

        # 5. Calcola cinematica inversa per ogni punto della traiettoria
        q_traj = []
        q_curr = q0
        
        for i, T in enumerate(trajectory):
            try:
                sol = self.robot.ik_LM(T, q0=q_curr, mask=[1,1,1,1,1,1])
                if sol is not None and len(sol) > 0:
                    q_curr = sol[0]
                    q_traj.append(q_curr)
                    if i % 3 == 0:  # Log ogni 3 punti per non sovraccaricare
                        self.get_logger().info(f"IK punto {i}: successo")
                else:
                    self.get_logger().warning(f"IK fallita al punto {i}. Riutilizzo configurazione precedente")
                    q_traj.append(q_curr.copy())
            except Exception as e:
                self.get_logger().error(f"Errore IK al punto {i}: {e}")
                q_traj.append(q_curr.copy())

        if not q_traj:
            self.get_logger().error("Nessun punto della traiettoria calcolato con successo")
            return
            
        q_traj = np.array(q_traj)
        self.get_logger().info(f"Traiettoria completa generata: {len(q_traj)} punti")

        # 6. Pubblica solo l'ultimo punto (configurazione finale)
        q_final = q_traj[-1]
        self.get_logger().info("Pubblicazione configurazione finale...")
        self.publish_joint_configuration(q_final, "TRAIETTORIA COMPLETA")

    def publish_joint_configuration(self, q_solution, description):
        """Pubblica configurazione giunti sui controller"""
        
        # Prepara trajectory per torso
        traj_torso = JointTrajectory()
        traj_torso.joint_names = ['torso_lift_joint']
        point_torso = JointTrajectoryPoint()
        point_torso.positions = [float(q_solution[0])]
        point_torso.time_from_start.sec = 5  # Tempo standard
        traj_torso.points.append(point_torso)

        # Prepara trajectory per braccio
        traj_arm = JointTrajectory()
        traj_arm.joint_names = [f'arm_{i+1}_joint' for i in range(7)]
        point_arm = JointTrajectoryPoint()
        point_arm.positions = [float(p) for p in q_solution[1:8]]
        point_arm.time_from_start.sec = 5  # Tempo standard
        traj_arm.points.append(point_arm)

        # Pubblica
        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)
        
        self.get_logger().info(f"=== {description} PUBBLICATA ===")
        
        # Timer per completamento con gestione migliorata e prevenzione ripetizioni
        if self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            # Movimento + chiusura gripper - tempo maggiore per presa sicura
            self.gripper_timer = self.create_timer(8.0, lambda: self.close_gripper())
            self.active_timer = self.create_timer(15.0, lambda: self.publish_command_completed_once(self.current_state.value))
        else:
            # Tempo aumentato per permettere movimenti più complessi
            self.active_timer = self.create_timer(12.0, lambda: self.publish_command_completed_once(self.current_state.value))

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
        # 1. Calcola vettore di avvicinamento verso il robot
        robot_to_marker = np.array([stabilized_pos[0], stabilized_pos[1], 0.0])  # Ignora Z per calcolo orizzontale
        distance_horizontal = np.linalg.norm(robot_to_marker[:2])
        
        if distance_horizontal > 0.001:  # Evita divisione per zero
            # Vettore unitario di avvicinamento orizzontale
            approach_vector = robot_to_marker / distance_horizontal
            # Avvicinamento dall'esterno verso il marker
            approach_offset = -approach_vector * approach_distance
        else:
            approach_offset = np.array([-approach_distance, 0.0, 0.0])  # Fallback: avvicinamento frontale

        # 2. Posizione finale dinamica
        # X,Y: Perfettamente centrato sul marker + offset di avvicinamento
        target_pose_out.pose.position.x = float(stabilized_pos[0] + approach_offset[0])
        target_pose_out.pose.position.y = float(stabilized_pos[1] + approach_offset[1])  # PERFETTO CENTRO Y
        
        # Z: Altezza del marker + eventuale sollevamento
        target_pose_out.pose.position.z = float(stabilized_pos[2] + lift_height)

        # Mantieni orientamento originale
        target_pose_out.pose.orientation = base_pose.orientation

        # Debug dettagliato
        final_pos = target_pose_out.pose.position
        self.get_logger().info(f"POSIZIONAMENTO DINAMICO - Marker {target_marker}:")
        self.get_logger().info(f"   Posizione originale:  [{stabilized_pos[0]:.3f}, {stabilized_pos[1]:.3f}, {stabilized_pos[2]:.3f}]")
        self.get_logger().info(f"   Approach offset:      [{approach_offset[0]:.3f}, {approach_offset[1]:.3f}, {lift_height:.3f}]")
        self.get_logger().info(f"   Posizione finale:     [{final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f}]")
        
        final_distance = np.sqrt(final_pos.x**2 + final_pos.y**2 + final_pos.z**2)
        self.get_logger().info(f"   Distanza finale dal robot: {final_distance:.3f}m")

        # Controllo raggiungibilità
        if final_distance > 1.4:
            self.get_logger().warn(f"Target lontano: {final_distance:.3f}m")
        elif final_distance < 0.25:
            self.get_logger().warn(f"Target molto vicino: {final_distance:.3f}m")
        else:
            self.get_logger().info(f"✓ Target a distanza ottimale: {final_distance:.3f}m")

        # Pubblica posa target
        self.pose_pub.publish(target_pose_out)
        self.get_logger().info("✓ Posa dinamica pubblicata - gripper sarà PERFETTAMENTE centrato!")

        return "dynamic_positioning"

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
        self.get_logger().info("Chiusura gripper in corso...")

        # Movimento gripper - chiusura più precisa per oggetti piccoli
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        
        # CORREZIONE: Chiusura più stretta per presa sicura
        # Coca-Cola e Pringles sono oggetti di dimensioni diverse
        if self.current_state == State.GRIP_OBJECT_1:
            # Coca-Cola: bottiglia più sottile
            point.positions = [0.035, 0.035]  # Chiusura più stretta
            self.get_logger().info("Chiusura gripper ottimizzata per Coca-Cola")
        else:
            # Pringles: cilindro più largo
            point.positions = [0.038, 0.038]  # Chiusura media
            self.get_logger().info("Chiusura gripper ottimizzata per Pringles")
            
        point.time_from_start.sec = 3  # Tempo più lungo per chiusura graduale
        traj.points.append(point)
        
        # Aggiungi sleep per garantire timing come in kinematic1.py
        time.sleep(0.5)
        self.gripper_pub.publish(traj)
        self.get_logger().info("Comando chiusura gripper pubblicato")
        
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
        self.gripper_pub.publish(traj)
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

        # Pubblica
        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)

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