#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from spatialmath import SE3

class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        self.state = 0
        self.pose_rilevate = {}
        # Configurazioni
        self.current_torso = 0.0
        self.current_arm_q = [0.0]*7

        # Carica robot URDF (torso + arm)
        urdf_loc = '/home/claudio/progetto_ros2/dev_ros/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info(f"Robot caricato: {self.robot.name} con {self.robot.n} giunti")

        # ActionClient per torso+arm
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()

        # Subscribe ai marker
        for m in [1, 2, 3, 4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_pose_base_{m}',
                lambda msg, id=m: self.pose_callback(msg, id),
                10
            )

        self.timer = self.create_timer(1.0, self.esegui_macchina_stati)
        self.get_logger().info('Nodo macchina_a_stati avviato.')

    def pose_callback(self, msg: PoseStamped, marker_id: int):
        self.pose_rilevate[marker_id] = msg.pose
        self.get_logger().info(f"Marker rilevato: {marker_id}")

    def esegui_macchina_stati(self):
        if self.state == 0:
            # Attendi 4 marker
            count = len(self.pose_rilevate)
            if count < 4:
                self.get_logger().info(
                    f"Attendo 4 marker (ricevuti: {count} -> {sorted(self.pose_rilevate.keys())})"
                )
                return
            self.get_logger().info(f"Ricevuti marker {sorted(self.pose_rilevate.keys())}, invio configurazione intermedia")
            # Intermedia
            self.current_torso = 0.0
            self.current_arm_q = [
                0.0003836728901962516, -0.0001633239063343339,
                -9.037018213753356e-06, -6.145563957549172e-05,
                4.409014973383307e-05, 0.0019643255648595925,
                0.0004167305736686444
            ]
            self.invia_punto(self.current_torso, self.current_arm_q)
            self.inter_sent_time = time.time()
            self.state = 1

        elif self.state == 1:
            # Attendi completamento intermedia
            if time.time() - self.inter_sent_time < 5.0:
                self.get_logger().info('Configurazione intermedia in transito...')
                return
            self.get_logger().info('Invio configurazione finale (torso + arm)')
            # Finale
            self.current_torso = 0.35
            self.current_arm_q = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
            self.invia_punto(self.current_torso, self.current_arm_q)
            self.state = 2

        elif self.state == 2:
            # Cinematica inversa
            if not {2, 3}.issubset(self.pose_rilevate):
                self.get_logger().info('In attesa dei marker 2 e 3 per IK')
                return
            self.get_logger().info('Inizio cin. inv. marker 2->3')
            # Poses
            p2 = self.pose_rilevate[2].position
            p3 = self.pose_rilevate[3].position
            R_align = SE3.Rx(-np.pi/2)
            # Build initial full q0 (torso + arm)
            q0 = np.array([self.current_torso] + self.current_arm_q)
            T0 = self.robot.fkine(q0)
            T2 = R_align * SE3(p2.x, p2.y, p2.z)
            T3 = R_align * SE3(p3.x, p3.y, p3.z)
            # Cartesian trajectory
            Ts1 = rtb.ctraj(T0, T2, 50)
            Ts2 = rtb.ctraj(T2, T3, 50)
            Ts = np.vstack((Ts1, Ts2))
            # IK per ogni passo
            q_traj = []
            q_curr = q0.copy()
            for T in Ts:
                sol = self.robot.ikine_LM(T, q0=q_curr)
                if sol.success:
                    q_curr = sol.q
                q_traj.append(q_curr.tolist())
            # Send full trajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [
                'torso_lift_joint',
                'arm_1_joint','arm_2_joint','arm_3_joint',
                'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint'
            ]
            for i, q in enumerate(q_traj, start=1):
                pt = JointTrajectoryPoint()
                pt.positions = q
                pt.time_from_start.sec = i
                goal.trajectory.points.append(pt)
            self.arm_client.send_goal_async(goal)
            self.get_logger().info('Traiettoria IK inviata.')
            self.timer.cancel()

    def invia_punto(self, torso_val, arm_vals):
        # Singolo punto torso+arm
        config = [torso_val] + arm_vals
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'torso_lift_joint',
            'arm_1_joint','arm_2_joint','arm_3_joint',
            'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint'
        ]
        pt = JointTrajectoryPoint()
        pt.positions = config
        pt.time_from_start.sec = 5
        goal.trajectory.points.append(pt)
        self.arm_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    nodo = MacchinaStati()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
