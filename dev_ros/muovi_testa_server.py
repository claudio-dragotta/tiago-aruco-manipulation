#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

class ServerMovimentoTesta(Node):

    def __init__(self):
        super().__init__('server_movimento_testa')

        # Publisher sul topic che muove la testa
        self.publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

        # Action server che ascolta i comandi dal client
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            self.esegui_callback)

        self.get_logger().info('✅ Action Server movimento testa avviato.')

    def esegui_callback(self, goal_handle):
        self.get_logger().info('🎯 Ricevuta richiesta dal client.')

        traiettoria = goal_handle.request.trajectory

        # Applica il vincolo head_2_joint = -0.57 se non già impostato
        for punto in traiettoria.points:
            if len(punto.positions) == 1:
                punto.positions.append(-0.57)  # fissa head_2_joint

        # Pubblica sul topic
        self.publisher.publish(traiettoria)

        # Attendi simulazione del movimento
        self.get_logger().info('⏳ Eseguo movimento...')
        self.get_clock().sleep_for(Duration(sec=2))

        goal_handle.succeed()
        risultato = FollowJointTrajectory.Result()
        self.get_logger().info('✅ Movimento completato.')
        return risultato

def main(args=None):
    rclpy.init(args=args)
    nodo = ServerMovimentoTesta()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
