#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ClientMovimentoTesta(Node):

    def __init__(self):
        super().__init__('client_movimento_testa')
        self._client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.get_logger().info('🔁 In attesa che il server sia disponibile...')
        self._client.wait_for_server()

        # Crea il goal: sequenza di posizioni
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        punti = [
            (-0.217, -0.57),  # sinistra
            (0.0, -0.57),     # centro
            (0.217, -0.57),   # destra
            (0.0, -0.57),     # ritorno al centro
        ]

        for i, pos in enumerate(punti):
            punto = JointTrajectoryPoint()
            punto.positions = list(pos)
            punto.time_from_start.sec = 4 * (i + 1)  # 4 secondi per step, più lenti
            goal_msg.trajectory.points.append(punto)
            self.get_logger().info(f'🧭 Aggiungo punto {i+1}: {pos}')


        self.get_logger().info('📤 Invio la sequenza al server...')
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Comando rifiutato dal server.')
            return

        self.get_logger().info('✅ Comando accettato, attendo risultato...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('🏁 Sequenza completata.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = ClientMovimentoTesta()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
