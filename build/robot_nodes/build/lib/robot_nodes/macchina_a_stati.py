#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from spatialmath import SE3

import time

class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        self.state = 0
        self.pose_rilevate = {}
        self.intermediate_q = [0.0004, -0.00016, -0.000009, -0.00006, 0.00004, 0.0019, 0.0004]
        self.current_torso  = 0.35
        self.current_arm    = [0.07,   0.1,    -3.1,    1.36,    2.05,    0.01,   -0.05]
        self.arm_goal_done  = False
                
        urdf_loc = '/home/claudio/progetto_ros2/src/robot_nodes/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info(f"[INIT] Robot caricato: {self.robot.name}")

        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        self.torso_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        # Sottoscrivo i 4 marker ArUco
        for m in [1,2,3,4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_pose_base_{m}',
                self.make_pose_callback(m),
                10
            )

        # Timer per la macchina a stati
        self.timer = self.create_timer(1.0, self.macchina_stati)
        self.get_logger().info('[START] Nodo macchina a stati avviato.')

    def make_pose_callback(self, marker_id):
        def callback(msg):
            self.pose_rilevate[marker_id] = msg.pose
            p = msg.pose.position
            self.get_logger().info(
                f"[CALLBACK] Marker {marker_id} = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
        return callback

    def macchina_stati(self):

        if self.state == 0:
            self.get_logger().info(f"[STATE] {self.state}, marker: {list(self.pose_rilevate.keys())}")
            if len(self.pose_rilevate) < 4:
                self.get_logger().info("[WAIT] Aspetto 4 marker...")
                return
            self.get_logger().info("[ACTION] Invio posa intermedia")
            self.invia_punto_arm(self.intermediate_q)
            self.state = 1

        elif self.state == 1:
            if self.arm_goal_done:
                self.get_logger().info("[ACTION] Posa intermedia OK, invio configurazione finale")
                self.arm_goal_done = False
                self.invia_punto(self.current_torso, self.current_arm)
                self.state = 2

        elif self.state == 2:
            if self.arm_goal_done:
                self.arm_goal_done = False
                self.get_logger().info("[ACTION] Inizio spostamenti IK")
                self.start_q = [self.current_torso] + self.current_arm
                self.get_logger().info("[TASK] 2 → 3")
                self.esegui_movimento(2, 3)
                self.state = 3

        elif self.state == 3:
            if self.arm_goal_done:
                self.arm_goal_done = False
                self.get_logger().info("[TASK] 1 → 4")
                self.esegui_movimento(1, 4)
        
        elif self.state == 4:
            if self.arm_goal_done:   
                self.get_logger().info("[END] Fine, shutdown")
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

    def invia_punto_arm(self, arm_vals):
        self.get_logger().info(f'Configurazione braccio: {arm_vals}')
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions = arm_vals
        arm_pt.time_from_start.sec = 5
        arm_goal.trajectory.points.append(arm_pt)

        self.arm_goal_done = False 
        fut = self.arm_client.send_goal_async(arm_goal)
        fut.add_done_callback(self._on_goal_response)  

    def invia_punto(self, torso_val, arm_vals):
        self.get_logger().info(f'[TORQUE] Invio configurazione torso: {torso_val}')
        torso_goal = FollowJointTrajectory.Goal()
        torso_goal.trajectory.joint_names = ['torso_lift_joint']
        torso_pt = JointTrajectoryPoint()
        torso_pt.positions = [torso_val]
        torso_pt.time_from_start.sec = 5
        torso_goal.trajectory.points.append(torso_pt)

        self.torso_client.send_goal_async(torso_goal)

        self.get_logger().info(f'[ARM] Invio configurazione braccio: {arm_vals}')
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = [
            'arm_1_joint','arm_2_joint','arm_3_joint',
            'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint'
        ]
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions = arm_vals
        arm_pt.time_from_start.sec = 5
        arm_goal.trajectory.points.append(arm_pt)

        self.arm_goal_done = False
        fut = self.arm_client.send_goal_async(arm_goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("[ERROR] Goal rifiutato")
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info("Movimento completato")
        self.arm_goal_done = True

    def esegui_movimento(self, m_from, m_to):
        p1 = self.pose_rilevate[m_from]
        p2 = self.pose_rilevate[m_to]

        # Estrai posizione (lista [x, y, z])
        pos1 = [p1.position.x, p1.position.y, p1.position.z]
        pos2 = [p2.position.x, p2.position.y, p2.position.z]

        # Estrai quaternione
        quat1 = [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w]
        quat2 = [p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w]

        # Converti quaternione → matrice di rotazione (3x3)
        R1 = R.from_quat(quat1).as_matrix()
        R2 = R.from_quat(quat2).as_matrix()

        # (Opzionale) Rotazione extra sul gripper (90° su X)
        R_extra = R.from_euler('x', 90, degrees=True).as_matrix()
        R1_total = R1 @ R_extra
        R2_total = R2 @ R_extra

        # # Costruisci trasformazioni SE3
        T1 = SE3.Rt(R1_total, pos1)
        T2 = SE3.Rt(R2_total, pos2)


        self.get_logger().info(f"[IK] Calcolo configurazioni {m_from}→{m_to}")
        try:
            self.robot.q = np.array(self.start_q)
            sol1 = self.robot.ikine_LM(T1, q0=self.start_q)

            self.robot.q = np.array(sol1.q)
            sol2 = self.robot.ikine_LM(T2, q0=sol1.q if sol1.success else self.start_q)

            if not sol1.success or not sol2.success:
                self.get_logger().error("[IK] Cinematica inversa fallita su uno dei punti")
                return

            q1 = sol1.q
            q1l = q1.tolist()
            self.invia_punto(q1l[0], q1l[1:])

            # time.sleep(5)
            node = rclpy.create_node('pausa_node')
            rate = node.create_rate(5)
            rate.sleep()
            
            q2 = sol2.q
            q2l = q2.tolist()
            self.invia_punto(q2l[0], q2l[1:])

            self.start_q = q2l

            # Simula presa e rilascio
            self.get_logger().info(f"[GRIPPER] Simulo presa su marker {m_from}")
            self.attach()
            self.get_logger().info(f"[GRIPPER] Simulo rilascio su marker {m_to}")
            self.detach()

        except Exception as e:
            self.get_logger().error(f"[IK] Errore: {e}")
            return

    def attach(self):
        self.get_logger().info("[ATTACH] Oggetto afferrato (simulato)")

    def detach(self):
        self.get_logger().info("[DETACH] Oggetto rilasciato (simulato)")
        

def main(args=None):
    rclpy.init(args=args)
    nodo = MacchinaStati()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
