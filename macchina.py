#!/usr/bin/env python3

import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

# Definizione degli offset per i marker
offset_per_marker = {
    1: 0.05,  # Offset per Pringles
    2: 0.05,  # Offset per Coca Cola
    3: 0.05,  # Offset per posizione destinazione
    4: 0.05   # Offset per posizione destinazione
}

class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        self.state = 0
        self.pose_rilevate = {}

        # Parametri DH per il braccio del Tiago
        self.dh_params = np.array([
            [0.1500,  0.0000,  0.0000,  np.pi/2],  # Joint 1
            [0.0000,  0.0000,  0.0000, -np.pi/2],  # Joint 2
            [0.2250,  0.0000,  0.0000,  np.pi/2],  # Joint 3
            [0.0000,  0.0000,  0.0000, -np.pi/2],  # Joint 4
            [0.2250,  0.0000,  0.0000,  np.pi/2],  # Joint 5
            [0.0000,  0.0000,  0.0000, -np.pi/2],  # Joint 6
            [0.1850,  0.0000,  0.0000,  0.0000]    # Joint 7
        ])

        # Limiti dei giunti del Tiago (in radianti)
        self.joint_limits = np.array([
            [-2.2689, 2.2689],  # Joint 1
            [-1.5708, 1.5708],  # Joint 2
            [-2.2689, 2.2689],  # Joint 3
            [-1.5708, 1.5708],  # Joint 4
            [-2.2689, 2.2689],  # Joint 5
            [-1.5708, 1.5708],  # Joint 6
            [-2.2689, 2.2689]   # Joint 7
        ])

        # Aggiungi il client per il torso
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        
        for marker_id in [1, 2, 3, 4]:
            topic = f'/aruco_pose_base_{marker_id}'
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, marker_id=marker_id: self.pose_callback(msg, marker_id),
                10
            )

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.torso_client.wait_for_server()
        
        # Inizializza il timer per la macchina a stati
        self.timer = self.create_timer(0.1, self.esegui_macchina_stati)

    def pose_callback(self, msg, marker_id):
        self.pose_rilevate[marker_id] = msg
        self.get_logger().info(f'Ricevuta posa per marker {marker_id}')

    def is_position_reachable(self, pos, marker_id=None):
        # Limiti approssimativi del braccio del Tiago
        MAX_REACH_XY = 0.85  # Raggio massimo sul piano XY
        MIN_Z = 0.5  # Altezza minima
        MAX_Z = 2.0  # Altezza massima
        
        horizontal_distance = np.sqrt(pos[0]**2 + pos[1]**2)
        
        self.get_logger().info(f'📏 Analisi raggiungibilità per posizione:\n'
                               f'   - Distanza orizzontale: {horizontal_distance:.3f}m\n'
                               f'   - Altezza (Z): {pos[2]:.3f}m')
        
        if horizontal_distance > MAX_REACH_XY:
            return False, f"Posizione {marker_id} troppo distante (dist: {horizontal_distance:.3f}m, max: {MAX_REACH_XY}m)"
        if pos[2] < MIN_Z:
            return False, f"Posizione {marker_id} troppo bassa (z: {pos[2]:.3f}m, min: {MIN_Z}m)"
        if pos[2] > MAX_Z:
            return False, f"Posizione {marker_id} troppo alta (z: {pos[2]:.3f}m, max: {MAX_Z}m)"
            
        return True, "Posizione raggiungibile"

    def calculate_torso_height(self, pos):
        BASE_TORSO_HEIGHT = 0.0
        MAX_TORSO_HEIGHT = 0.35
        OPTIMAL_Z_REACH = 1.5
        
        if pos[2] > OPTIMAL_Z_REACH:
            needed_height = min(MAX_TORSO_HEIGHT, pos[2] - OPTIMAL_Z_REACH)
            self.get_logger().info(f'🔼 Calcolo altezza torso necessaria:\n'
                                  f'   - Altezza target: {pos[2]:.3f}m\n'
                                  f'   - Altezza ottimale: {OPTIMAL_Z_REACH}m\n'
                                  f'   - Altezza torso necessaria: {needed_height:.3f}m')
            return needed_height
        return BASE_TORSO_HEIGHT

    def move_torso(self, height):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        punto = JointTrajectoryPoint()
        punto.positions = [height]
        punto.time_from_start.sec = 2
        goal.trajectory.points.append(punto)
        self.get_logger().info(f'🔄 Movimento torso a altezza: {height:.3f}m')
        self.torso_client.send_goal_async(goal)

    def esegui_macchina_stati(self):
        if self.state == 0:
            self.get_logger().info('Stato 0: invio configurazione intermedia')
            # Manteniamo la configurazione iniziale fornita dal professore
            configurazione_intermedia = [
                0.0004, -0.0002, 0.0, -0.00006, 0.00004, 0.0019, 0.0004
            ]
            self.invia_trajectory(configurazione_intermedia)
            self.state = 1
            # Inizializziamo l'ultima configurazione valida con quella del professore
            self.ultima_config_valida = configurazione_intermedia.copy()

        elif self.state == 1 and len(self.pose_rilevate) >= 4:
            self.get_logger().info('Stato 1: calcolo traiettorie IK per gli oggetti')
            
            sequenza = [
                (1, 4),  # Pringles → posizione 4
                (2, 3)   # Coca Cola → posizione 3
            ]

            # Prima verifica tutte le posizioni
            max_torso_height = 0.0
            posizioni_non_raggiungibili = []
            
            for origine, destinazione in sequenza:
                posa_origine = self.pose_rilevate[origine].pose.position
                posa_destinazione = self.pose_rilevate[destinazione].pose.position

                pos_sopra = np.array([posa_origine.x, posa_origine.y, posa_origine.z])
                pos_grab = pos_sopra.copy()
                pos_grab[2] -= offset_per_marker.get(origine, 0.05)

                pos_sopra_dest = np.array([posa_destinazione.x, posa_destinazione.y, posa_destinazione.z])
                pos_release = pos_sopra_dest.copy()
                pos_release[2] -= offset_per_marker.get(origine, 0.05)

                for pos, azione in [
                    (pos_sopra, None),
                    (pos_grab, 'chiudi'),
                    (pos_sopra, None),
                    (pos_sopra_dest, None),
                    (pos_release, 'apri'),
                    (pos_sopra_dest, None)
                ]:
                    reachable, message = self.is_position_reachable(pos, origine)
                    if not reachable:
                        self.get_logger().warn(f'⚠️ {message}')
                        needed_height = self.calculate_torso_height(pos)
                        max_torso_height = max(max_torso_height, needed_height)
                        posizioni_non_raggiungibili.append((pos, needed_height))
            
            if posizioni_non_raggiungibili:
                self.get_logger().info(f'🔼 Necessario alzare il torso a {max_torso_height:.3f}m')
                self.move_torso(max_torso_height)
                time.sleep(3)  # Attendi che il torso si muova

            # Procedi con la cinematica inversa
            self.azioni = []
            for origine, destinazione in sequenza:
                posa_origine = self.pose_rilevate[origine].pose.position
                posa_destinazione = self.pose_rilevate[destinazione].pose.position

                pos_sopra = np.array([posa_origine.x, posa_origine.y, posa_origine.z])
                pos_grab = pos_sopra.copy()
                pos_grab[2] -= offset_per_marker.get(origine, 0.05)

                pos_sopra_dest = np.array([posa_destinazione.x, posa_destinazione.y, posa_destinazione.z])
                pos_release = pos_sopra_dest.copy()
                pos_release[2] -= offset_per_marker.get(origine, 0.05)

                for pos, azione in [
                    (pos_sopra, None),
                    (pos_grab, 'chiudi'),
                    (pos_sopra, None),
                    (pos_sopra_dest, None),
                    (pos_release, 'apri'),
                    (pos_sopra_dest, None)
                ]:
                    # Usiamo l'ultima configurazione valida come punto di partenza
                    angoli, ok = self.inverse_kinematics_jacobian(
                        self.dh_params, self.ultima_config_valida, pos, np.zeros(3), self.joint_limits
                    )
                    if ok:
                        self.azioni.append((angoli, azione))
                        # Aggiorniamo l'ultima configurazione valida
                        self.ultima_config_valida = angoli
                    else:
                        self.get_logger().warn(f'IK fallita per posizione: {pos}')
                        # Se fallisce l'IK, interrompiamo la sequenza per questo oggetto
                        break

            self.az_index = 0
            self.state = 2

        elif self.state == 2 and self.az_index < len(self.azioni):
            angoli, azione = self.azioni[self.az_index]
            self.get_logger().info(f'Eseguo step {self.az_index+1}/{len(self.azioni)}')
            self.invia_trajectory(angoli)
            if azione:
                self.get_logger().info(f'Azione gripper: {azione}')
                self.controlla_gripper(chiudi=(azione == "chiudi"))
            self.az_index += 1
            time.sleep(2)

        elif self.state == 2 and self.az_index >= len(self.azioni):
            self.get_logger().info('Sequenza completata.')
            self.timer.cancel()

    def invia_trajectory(self, configurazione):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        punto = JointTrajectoryPoint()
        punto.positions = configurazione
        punto.time_from_start.sec = 5
        goal.trajectory.points.append(punto)
        self.arm_client.send_goal_async(goal)

    def controlla_gripper(self, chiudi=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        punto = JointTrajectoryPoint()
        posizione = 0.0 if chiudi else 0.04
        punto.positions = [posizione, posizione]
        punto.time_from_start.sec = 2
        goal.trajectory.points.append(punto)
        self.gripper_client.send_goal_async(goal)

    def inverse_kinematics_jacobian(self, dh_params, joint_angles, target_position, target_velocity, joint_limits, K=1.0, max_iterations=500, tol=1e-2):
        q = np.array(joint_angles)
        for _ in range(max_iterations):
            _, T_current, _ = self.forward_kinematics(dh_params, q)
            current_position = T_current[:3, 3]
            error = target_position - current_position
            if np.linalg.norm(error) < tol:
                return q.tolist(), True
            J = self.calculate_geometric_jacobian(dh_params, q)
            J_pinv = np.linalg.pinv(J)
            dq = np.dot(J_pinv, target_velocity + K * error)
            q = q + dq * 0.5
            q = np.clip(q, joint_limits[:, 0], joint_limits[:, 1])
        return q.tolist(), False

    def dh_transformation_matrix(self, d, theta, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, dh_params, joint_angles):
        T = np.eye(4)
        for i, (d, _, a, alpha) in enumerate(dh_params):
            T = np.dot(T, self.dh_transformation_matrix(d, joint_angles[i], a, alpha))
        return None, T, None

    def calculate_geometric_jacobian(self, dh_params, joint_angles):
        T = np.eye(4)
        z_axes = [np.array([0, 0, 1])]
        positions = [np.array([0, 0, 0])]
        for i, (d, _, a, alpha) in enumerate(dh_params):
            Ti = self.dh_transformation_matrix(d, joint_angles[i], a, alpha)
            T = np.dot(T, Ti)
            z_axes.append(T[:3, 2])
            positions.append(T[:3, 3])
        
        p_e = positions[-1]
        J = np.zeros((6, len(joint_angles)))
        
        for i in range(len(joint_angles)):
            J[:3, i] = np.cross(z_axes[i], p_e - positions[i])
            J[3:, i] = z_axes[i]
        
        return J


def main(args=None):
    rclpy.init(args=args)
    nodo = MacchinaStati()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()