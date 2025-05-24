#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import rclpy.duration

# Offset per i marker
offset_per_marker = {1: 0.05, 2: 0.05, 3: 0.05, 4: 0.05}
# Configurazione intermedia (Task 2 slide)
intermediate_config = [
    0.0003836728901962516,
    -0.0001633239063343339,
    -9.037018213753356e-06,
    -6.145563957549172e-05,
    4.409014973383307e-05,
    0.0019643255648595925,
    0.0004167305736686444
]

class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        # stato e dati
        self.state = 0
        self.pose_rilevate = {}
        # controlli asincroni
        self.action_in_progress = False
        self.azioni = []
        self.az_index = 0

        # parametri cinematica
        self.dh_params = np.array([
            [0.1500, 0.0000, 0.0000, np.pi/2],
            [0.0000, 0.0000, 0.0000, -np.pi/2],
            [0.2250, 0.0000, 0.0000, np.pi/2],
            [0.0000, 0.0000, 0.0000, -np.pi/2],
            [0.2250, 0.0000, 0.0000, np.pi/2],
            [0.0000, 0.0000, 0.0000, -np.pi/2],
            [0.1850, 0.0000, 0.0000, 0.0000]
        ])
        self.joint_limits = np.array([
            [-2.2689, 2.2689],
            [-1.5708, 1.5708],
            [-2.2689, 2.2689],
            [-1.5708, 1.5708],
            [-2.2689, 2.2689],
            [-1.5708, 1.5708],
            [-2.2689, 2.2689]
        ])

        # action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.torso_client.wait_for_server()

        # subscriber ai marker
        for mid in [1, 2, 3, 4]:
            self.create_subscription(
                PoseStamped, f'/aruco_pose_base_{mid}',
                lambda msg, m=mid: self.pose_callback(msg, m), 10
            )

        # timer macchina a stati
        self.timer = self.create_timer(0.1, self.esegui_macchina_stati)

    def pose_callback(self, msg: PoseStamped, marker_id: int):
        self.pose_rilevate[marker_id] = msg.pose.position
        self.get_logger().info(
            f'Marker {marker_id}: '
            f'x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} '
            f'z={msg.pose.position.z:.3f}'
        )

    def esegui_macchina_stati(self):
        # Stato 0: invia configurazione intermedia
        if self.state == 0 and not self.action_in_progress:
            self.get_logger().info('Stato 0: invio configurazione intermedia')
            self.invia_trajectory(intermediate_config)
            self.state = 1
            return

        # Stato 1: attendo tutte le pose rilevate
        if self.state == 1 and not self.action_in_progress:
            if len(self.pose_rilevate) >= 4:
                self.get_logger().info('Task2: tutte le pose lette, genero sequenza')
                self.costruisci_sequenza()
                self.state = 2
            return

        # Stato 2: esecuzione sequenza
        if self.state == 2 and not self.action_in_progress:
            if self.az_index < len(self.azioni):
                q, grip = self.azioni[self.az_index]
                self.invia_trajectory(q)
            else:
                self.get_logger().info('Sequenza Task2 completata')
                self.state = 3
            return

    def costruisci_sequenza(self):
        # sequenza di pick-and-place: (1->4), (2->3)
        seq = [(1, 4), (2, 3)]
        q_prev = intermediate_config.copy()
        self.azioni = []

        for orig, dest in seq:
            p_o = self.pose_rilevate[orig]
            p_d = self.pose_rilevate[dest]
            pos_sopra_o = np.array([p_o.x, p_o.y, p_o.z + 0.1])
            pos_grab_o  = np.array([p_o.x, p_o.y, p_o.z - offset_per_marker[orig]])
            pos_sopra_d = np.array([p_d.x, p_d.y, p_d.z + 0.1])
            pos_rel_d   = np.array([p_d.x, p_d.y, p_d.z - offset_per_marker[orig]])

            steps = [
                (pos_sopra_o, None),
                (pos_grab_o, 'chiudi'),
                (pos_sopra_o, None),
                (pos_sopra_d, None),
                (pos_rel_d, 'apri'),
                (pos_sopra_d, None)
            ]

            for pos, action in steps:
                q_sol, ok = self.inverse_kinematics_jacobian(
                    self.dh_params, q_prev, pos, np.zeros(3), self.joint_limits
                )
                if not ok:
                    self.get_logger().error(f'IK fallita per {pos}')
                    return
                self.azioni.append((q_sol, action))
                q_prev = q_sol

    def invia_trajectory(self, q):
        self.action_in_progress = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        # header.stamp nel futuro di 100ms
        now = self.get_clock().now()
        future_time = now + rclpy.duration.Duration(seconds=0.1)
        goal.trajectory.header.stamp = future_time.to_msg()

        # punto iniziale t=0
        p0 = JointTrajectoryPoint()
        prev_q = self.azioni[self.az_index-1][0] if self.az_index > 0 else intermediate_config
        p0.positions = prev_q
        p0.time_from_start.sec = 0

        # punto finale t=5
        p1 = JointTrajectoryPoint()
        p1.positions = q
        p1.time_from_start.sec = 5
        goal.trajectory.points = [p0, p1]

        fut = self.arm_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rifiutato')
            self.action_in_progress = False
            return
        done = handle.get_result_async()
        done.add_done_callback(self._on_trajectory_done)

    def _on_trajectory_done(self, future):
        res = future.result().result
        self.get_logger().info(f'Traiettoria completata, code {res.error_code}')
        _, action = self.azioni[self.az_index]
        if action == 'chiudi':
            self.controlla_gripper(True)
        elif action == 'apri':
            self.controlla_gripper(False)
        self.action_in_progress = False
        self.az_index += 1

    def controlla_gripper(self, chiudi: bool):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0] if chiudi else [0.04, 0.04]
        pt.time_from_start.sec = 2
        goal.trajectory.points.append(pt)
        self.gripper_client.send_goal_async(goal)

    # Cinematica (unchanged)
    def dh_transformation_matrix(self, d, theta, a, alpha):
        ct = np.cos(theta); st = np.sin(theta)
        ca = np.cos(alpha); sa = np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0,0,0,1]])

    def forward_kinematics(self, dh_params, q):
        T = np.eye(4); trans = []
        for i,(d,_,a,al) in enumerate(dh_params):
            T = T @ self.dh_transformation_matrix(d, q[i], a, al)
            trans.append(T.copy())
        return trans[:-1], T, trans

    def calculate_geometric_jacobian(self, dh_params, q):
        trans, T_end, _ = self.forward_kinematics(dh_params, q)
        p_end = T_end[:3,3]; J = np.zeros((6,len(q)))
        for i in range(len(q)):
            Ti = np.eye(4) if i==0 else trans[i-1]
            z, p = Ti[:3,2], Ti[:3,3]
            J[:3,i] = np.cross(z,p_end-p); J[3:,i]=z
        return J

    def inverse_kinematics_jacobian(self, dh_params, q_init, target_pos, target_vel, joint_limits, K=0.1, max_iter=1500, tol=1e-4):
        q = np.array(q_init, dtype=float); step=0.05; damp=0.1
        for _ in range(max_iter):
            _, T,_ = self.forward_kinematics(dh_params,q); p=T[:3,3]
            err = target_pos - p
            if np.linalg.norm(err)<tol: return q.tolist(),True
            J = self.calculate_geometric_jacobian(dh_params, q)[:3,:]
            JJt = J@J.T; inv = np.linalg.inv(JJt + damp*np.eye(3))
            dq = J.T@inv@(target_vel + K*err)
            q_new = np.clip(q + dq*step, joint_limits[:,0], joint_limits[:,1])
            if np.allclose(q,q_new,atol=1e-6): return q.tolist(),False
            q=q_new
        return q.tolist(),False


def main():
    rclpy.init()
    nodo = MacchinaStati()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
