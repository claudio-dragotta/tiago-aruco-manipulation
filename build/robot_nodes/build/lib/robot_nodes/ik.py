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

        # TODO: Install gazebo-ros-link-attacher package for object manipulation
        # self.attach_client = self.create_client(Attach, '/link_attacher_node/attach')
        # self.detach_client = self.create_client(Attach, '/link_attacher_node/detach')
        
        self.get_logger().warn("ATTENZIONE: gazebo_ros_link_attacher non disponibile - attach/detach simulati")
        
        # Action clients per torso, arm e gripper
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client   = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

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
        # Segnala completamento dopo 3 secondi
        self.create_timer(3.5, lambda: self.publish_command_completed(State.INTERMEDIATE_CONFIG.value))

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
        
        # Segnala completamento dopo 4 secondi
        self.create_timer(4.0, lambda: self.publish_command_completed(State.OPERATIONAL_CONFIG.value))

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
            # CORREZIONE: ArUco è SOPRA l'oggetto, scendi di più e avvicinati
            self.calculate_pose_with_offset_state(1, offset_x=0.05, offset_y=-0.05, offset_z=-0.08)
            
        elif msg.data == State.GRIP_OBJECT_1.value:
            self.get_logger().info("Abbasso gripper per presa oggetto 1")
            self.current_state = State.GRIP_OBJECT_1
            # CORREZIONE: Scendi molto di più per raggiungere l'oggetto sotto l'ArUco
            self.calculate_pose_with_offset_state(1, offset_x=0.05, offset_y=-0.05, offset_z=-0.15)
            
        elif msg.data == State.LIFT_OBJECT_1.value:
            self.get_logger().info("Sollevamento oggetto 1")  
            self.current_state = State.LIFT_OBJECT_1
            # Solleva gradualmente dall'oggetto, non dall'ArUco
            self.calculate_pose_with_offset_state(1, offset_x=0.05, offset_y=-0.05, offset_z=-0.05)
            
        elif msg.data == State.MOVE_TO_DEST_1.value:
            self.get_logger().info("Trasporto oggetto 1 a destinazione (ArUco marker 3)")
            self.current_state = State.MOVE_TO_DEST_1
            # CORREZIONE: Posizione di deposizione sotto l'ArUco di destinazione
            self.calculate_pose_with_offset_state(3, offset_x=0.02, offset_y=-0.05, offset_z=-0.10)
            
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
            # CORREZIONE: ArUco è SOPRA l'oggetto, scendi di più e avvicinati
            self.calculate_pose_with_offset_state(2, offset_x=0.05, offset_y=-0.05, offset_z=-0.08)
            
        elif msg.data == State.GRIP_OBJECT_2.value:
            self.get_logger().info("Abbasso gripper per presa oggetto 2")
            self.current_state = State.GRIP_OBJECT_2
            # CORREZIONE: Scendi molto di più per raggiungere l'oggetto sotto l'ArUco
            self.calculate_pose_with_offset_state(2, offset_x=0.05, offset_y=-0.05, offset_z=-0.15)
            
        elif msg.data == State.LIFT_OBJECT_2.value:
            self.get_logger().info("Sollevamento oggetto 2")
            self.current_state = State.LIFT_OBJECT_2
            # Solleva gradualmente dall'oggetto, non dall'ArUco
            self.calculate_pose_with_offset_state(2, offset_x=0.05, offset_y=-0.05, offset_z=-0.05)
            
        elif msg.data == State.MOVE_TO_DEST_2.value:
            self.get_logger().info("Trasporto oggetto 2 a destinazione (ArUco marker 4)")
            self.current_state = State.MOVE_TO_DEST_2
            # CORREZIONE: Posizione di deposizione sotto l'ArUco di destinazione
            self.calculate_pose_with_offset_state(4, offset_x=0.02, offset_y=-0.05, offset_z=-0.10)
            
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
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"📍 ArUco 1: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] ori=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}] dist={distance_from_robot:.3f}m frame={msg.header.frame_id}")

    def aruco_pose_2_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_2")
        self.aruco_pose_2 = msg
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        ori = msg.pose.orientation
        distance_from_robot = (pos.x**2 + pos.y**2 + pos.z**2)**0.5
        self.get_logger().info(f"📍 ArUco 2: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] ori=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}] dist={distance_from_robot:.3f}m frame={msg.header.frame_id}")
       
    def aruco_pose_3_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_3")
        self.aruco_pose_3 = msg
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 3 pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} in frame {msg.header.frame_id}")

    def aruco_pose_4_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_4")
        self.aruco_pose_4 = msg
        # Debug: stampa pose ricevuta
        pos = msg.pose.position
        self.get_logger().info(f"ArUco 4 pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} in frame {msg.header.frame_id}")

    def joint_states_callback(self, msg):
        if self.current_joint_state is not None:
        # Hai già la configurazione iniziale, ignora i nuovi messaggi
            return
        
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
                
        self.current_joint_state = np.array(joint_pos)
        self.get_logger().info(f"🦾 [JOINT INIT] Configurazione salvata:")
        self.get_logger().info(f"   Torso: {joint_pos[0]:.3f}")
        self.get_logger().info(f"   Arm: [{joint_pos[1]:.3f}, {joint_pos[2]:.3f}, {joint_pos[3]:.3f}, {joint_pos[4]:.3f}, {joint_pos[5]:.3f}, {joint_pos[6]:.3f}, {joint_pos[7]:.3f}]")
        if missing_joints:
            self.get_logger().warn(f"❗ Joint mancanti (usato 0.0): {missing_joints}")

    def target_pose_callback(self, msg):
        self.get_logger().info("Funzione target_pose_callback chiamata!")
        self.target_pose = msg
        self.get_logger().info(f" Ricevuta target_pose: {msg.pose.position}")
        while self.current_joint_state is None:
            time.sleep(0.1)  # Attendi che il joint state sia inizializzato
        self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        self.get_logger().info("=== INIZIO PIANIFICAZIONE TRAIETTORIA ===")

        if self.current_joint_state is None or self.target_pose is None:
            self.get_logger().warn("Joint state o target pose non ancora ricevuti.")
            return

        q0 = self.current_joint_state.astype(float)  # [torso, arm1, ..., arm7]
        self.get_logger().info(f"Configurazione iniziale: torso={q0[0]:.3f}, arm={q0[1:8]}")

        # Estrai posizione target
        pos = self.target_pose.pose.position
        ori = self.target_pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        
        self.get_logger().info(f"Target position: {position}")

        # Crea matrice di trasformazione target con orientamento fisso
        T_target = SE3.Rt(np.eye(3), position) * SE3.Ry(np.pi/2) * SE3.Rz(-np.pi/2)

        # Calcola inverse kinematics
        try:
            sol = self.robot.ik_LM(T_target, q0=q0, mask=[1,1,1,1,1,1])
            
            if sol is not None and len(sol) > 0:
                q_solution = sol[0]
                self.get_logger().info("✅ Soluzione IK trovata")
                self.publish_joint_configuration(q_solution, "SOLUZIONE IK")
            else:
                self.get_logger().error("❌ IK fallita: nessuna soluzione trovata")
                return
                
        except Exception as e:
            self.get_logger().error(f"❌ Errore durante calcolo IK: {e}")
            return

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
        
        # Timer per completamento
        if self.current_state in [State.GRIP_OBJECT_1, State.GRIP_OBJECT_2]:
            # Movimento + chiusura gripper - tempo maggiore per presa sicura
            self.create_timer(5.0, lambda: self.close_gripper())
            self.create_timer(10.0, lambda: self.publish_command_completed(self.current_state.value))
        else:
            self.create_timer(8.0, lambda: self.publish_command_completed(self.current_state.value))

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
        self.get_logger().info(f"📍 ArUco {target} ORIGINALE:")
        self.get_logger().info(f"   Posizione: x={base_pose.position.x:.3f}, y={base_pose.position.y:.3f}, z={base_pose.position.z:.3f}")
        self.get_logger().info(f"   Frame: {header.frame_id}")
        self.get_logger().info(f"🎯 OFFSET APPLICATI:")
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
        self.get_logger().info(f"🎯 TARGET FINALE per ArUco {target}:")
        self.get_logger().info(f"   Posizione: x={final_pos.x:.3f}, y={final_pos.y:.3f}, z={final_pos.z:.3f}")
        self.get_logger().info(f"   Distanza finale: {final_distance:.3f}m")
        self.get_logger().info(f"   Differenza distanza: {final_distance-orig_distance:.3f}m")
        
        # Verifica raggiungibilità migliorata
        if final_distance > 1.4:
            self.get_logger().warn(f"⚠️  ATTENZIONE: Target molto lontano ({final_distance:.3f}m > 1.4m)")
        elif final_distance < 0.3:
            self.get_logger().warn(f"⚠️  ATTENZIONE: Target molto vicino ({final_distance:.3f}m < 0.3m)")
        else:
            self.get_logger().info(f"✅ Target a distanza ragionevole: {final_distance:.3f}m")

        # Pubblica la posa target
        self.pose_pub.publish(target_pose_out)
        self.get_logger().info("📤 Posa target pubblicata su /target_pose")

        return "go_to_pose"
    
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
        
        # Pubblica immediatamente invece di usare timer
        self.gripper_pub.publish(traj)
        self.get_logger().info("Comando chiusura gripper pubblicato")
        
        # Attiva attach dopo 4 secondi (tempo maggiore per chiusura completa)
        self.create_timer(4.0, lambda: self.handle_gripper_attach())

    def open_gripper(self):
        self.get_logger().info("Apertura gripper in corso...")

        # Movimento gripper
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 2
        traj.points.append(point)
        
        # Pubblica immediatamente invece di usare timer
        self.gripper_pub.publish(traj)
        self.get_logger().info("Comando apertura gripper pubblicato")
        
        # Attiva detach dopo 3 secondi
        self.create_timer(3.0, lambda: self.handle_gripper_detach())

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
        self.create_timer(10.0, lambda: self.publish_command_completed(self.current_state.value))

    def handle_gripper_attach(self):
        object_name = "cocacola" if self.current_state == State.GRIP_OBJECT_1 else "pringles"
        
        # TODO: Install gazebo-ros-link-attacher for real object attachment
        # if self.attach_client.service_is_ready():
        #     attach_request = Attach.Request()
        #     attach_request.model_name_1 = 'tiago'
        #     attach_request.link_name_1 = 'gripper_left_finger_link'
        #     attach_request.model_name_2 = object_name
        #     attach_request.link_name_2 = 'link'
        #     
        #     future = self.attach_client.call_async(attach_request)
        #     future.add_done_callback(self.attach_response_callback)
        #     self.get_logger().info(f"Richiesto attach di {object_name} al gripper")
        # else:
        self.get_logger().info(f"[SIMULATO] Attach {object_name} - gazebo_ros_link_attacher non disponibile")
        
        # Simula completamento immediato
        self.create_timer(1.0, lambda: self.publish_command_completed(self.current_state.value))

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
        
        # TODO: Install gazebo-ros-link-attacher for real object detachment  
        # if self.detach_client.service_is_ready():
        #     detach_request = Attach.Request()
        #     detach_request.model_name_1 = 'tiago'
        #     detach_request.link_name_1 = 'gripper_left_finger_link'
        #     detach_request.model_name_2 = object_name
        #     detach_request.link_name_2 = 'link'
        #     
        #     future = self.detach_client.call_async(detach_request)
        #     future.add_done_callback(self.detach_response_callback)
        #     self.get_logger().info(f"Richiesto detach di {object_name} dal gripper")
        # else:
        self.get_logger().info(f"[SIMULATO] Detach {object_name} - gazebo_ros_link_attacher non disponibile")
        
        # Simula completamento immediato
        self.create_timer(1.0, lambda: self.publish_command_completed(self.current_state.value))

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
        
    def publish_command_completed(self, state_value):
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