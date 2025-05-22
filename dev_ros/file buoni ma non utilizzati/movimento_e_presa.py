#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MovimentoEPresa(Node):
    def __init__(self):
        super().__init__('movimento_e_presa')

        self.cli = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.cli.wait_for_server()

        self.get_logger().info("🎯 Client pronto. Inizio movimento...")
        self.esegui_movimento()

    def esegui_movimento(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Configurazione intermedia (da slide)
        intermedia = [
            0.0003836728901962516,
            -0.0001633239063343339,
            -9.037018213753356e-06,
            -6.145563957549172e-05,
            4.409014973383307e-05,
            0.0019643255648595925,
            0.0004167305736686444
        ]

        # Configurazione finale (dal punto 2 delle slide)
        finale = [
            0.32028299570036024,
            1.221374508470005,
            -2.2819973537488867,
            1.125180443191409,
            1.3328805375953824,
            1.1018080198745896,
            0.47716232198716934
        ]

        # Punto intermedio
        punto1 = JointTrajectoryPoint()
        punto1.positions = intermedia
        punto1.time_from_start.sec = 3

        # Punto finale
        punto2 = JointTrajectoryPoint()
        punto2.positions = finale
        punto2.time_from_start.sec = 6

        goal_msg.trajectory.points = [punto1, punto2]

        self.cli.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovimentoEPresa()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
