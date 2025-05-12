#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from builtin_interfaces.msg import Duration

class ServerMovimentoTesta(Node):
    """
    Action Server che riceve una traiettoria per muovere la testa di TIAGO.
    """

    def __init__(self):
        super().__init__('server_movimento_testa')

        # Action server su FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            self.esegui_movimento
        )

        # Publisher per il topic /head_controller/joint_trajectory
        self.pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.get_logger().info('Server movimento testa avviato.')

    def esegui_movimento(self, goal_handle):
        """
        Callback chiamata quando il client invia una traiettoria.
        """
        self.get_logger().info('Richiesta ricevuta dal client.')

        # Estrae la traiettoria dal messaggio ricevuto
        traiettoria = goal_handle.request.trajectory

        # Pubblica la traiettoria
        self.pub.publish(traiettoria)

        # Aspetta un po’ per simulare l'esecuzione
        self.get_logger().info('Eseguo movimento...')
        self.get_clock().sleep_for(Duration(sec=2))

        # Termina l'action
        goal_handle.succeed()
        risultato = FollowJointTrajectory.Result()
        self.get_logger().info('Movimento completato.')
        return risultato

def main(args=None):
    rclpy.init(args=args)
    nodo = ServerMovimentoTesta()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

