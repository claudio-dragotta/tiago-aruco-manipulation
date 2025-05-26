#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

# Robotics Toolbox
import roboticstoolbox as rtb

class PickAruco(Node):
    def __init__(self):
        super().__init__('pick_aruco')

        # -- PARAMETRI --
        self.marker_id = 1
        self.required_poses = 4
        self.intermediate_q = [
            0.0003836728901962516,
            -0.0001633239063343339,
            -9.037018213753356e-06,
            -6.145563957549172e-05,
            4.409014973383307e-05,
            0.0019643255648595925,
            0.0004167305736686444
        ]
        self.poses = []
        self.moved = False

        # -- CARICA URDF TIAGO --
        urdf_path = os.path.join(
            os.path.dirname(__file__),
            'tiago_robot.urdf'
        )
        self.robot = rtb.models.ERobot.URDFRobot(urdf_path, name='tiago')
        self.get_logger().info(f'URDF caricato da {urdf_path}')

        # -- ACTION CLIENTS --
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(
            self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')

        # aspetta che i server siano pronti
        self.arm_client.wait_for_server()
        self.torso_client.wait_for_server()

        # -- SUBSCRIBER per la Pose di marker 1 --
        topic = f'/aruco_pose_base_{self.marker_id}'
        self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            10
        )
        self.get_logger().info(f'In attesa di pose da {topic}...')

    def pose_callback(self, msg: PoseStamped):
        if self.moved:
            return

        # salva la pose
        self.poses.append(msg)
        self.get_logger().info(
            f'Ricevuta pose #{len(self.poses)}/{self.required_poses}'
        )

        # quando ne ho 4, muovo in intermedia
        if len(self.poses) >= self.required_poses:
            self.moved = True
            self.move_to_intermediate()

    def move_to_intermediate(self):
        self.get_logger().info('Invio configurazione intermedia al braccio...')

        # TRAIETTORIA ARM
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        pt = JointTrajectoryPoint()
        pt.positions = self.intermediate_q
        pt.time_from_start.sec = 5
        arm_goal.trajectory.points.append(pt)
        self.arm_client.send_goal_async(arm_goal)

        # TRAIETTORIA TORSO (resta su 0.0)
        self.get_logger().info('Invio configurazione al torso (0.0 m)...')
        torso_goal = FollowJointTrajectory.Goal()
        torso_goal.trajectory.joint_names = ['torso_lift_joint']
        pt2 = JointTrajectoryPoint()
        pt2.positions = [0.0]
        pt2.time_from_start.sec = 5
        torso_goal.trajectory.points.append(pt2)
        self.torso_client.send_goal_async(torso_goal)

        self.get_logger().info('Movimento intermedio avviato.')

def main(args=None):
    rclpy.init(args=args)
    node = PickAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
