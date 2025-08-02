import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from spatialmath import SE3
from spatialmath.base import q2r
class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        self.state = 0
        self.pose_rilevate = {}
        self.intermediate_q = [0.0004, -0.00016, -0.000009, -0.00006, 0.00004, 0.0019, 0.0004]
        self.current_torso = 0.35
        self.current_arm_q = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
        self.arm_goal_done = False
        self.torso_goal_done = False

        urdf_loc = '/home/claudio/progetto_ros2/src/robot_nodes/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info(f"Robot caricato: {self.robot.name}")

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.arm_client.wait_for_server()
        self.torso_client.wait_for_server()
        self.gripper_client.wait_for_server()

        for m in [1, 2, 3, 4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_pose_base_{m}',
                self.make_pose_callback(m),
                10
            )

        self.timer = self.create_timer(1.0, self.macchina_stati)
        self.get_logger().info('Nodo macchina a stati avviato.')

    def make_pose_callback(self, marker_id):
        def callback(msg):
            self.pose_callback(msg, marker_id)
        return callback

    def pose_callback(self, msg, marker_id):
        self.pose_rilevate[marker_id] = msg.pose
        self.get_logger().info(f"Marker {marker_id} aggiornato: posizione=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")

    def macchina_stati(self):
        self.get_logger().info(f'Stato attuale: {self.state}')
        self.get_logger().info(f'Marker rilevati finora: {list(self.pose_rilevate.keys())}')

        if self.state == 0:
            if len(self.pose_rilevate) < 4:
                self.get_logger().info(' Attendo tutti e 4 i marker ArUco...')
                return
            self.get_logger().info('Marker rilevati, invio posa intermedia.')
            self.invia_punto_arm(self.intermediate_q)
            self.state = 1

        elif self.state == 1:
            if self.arm_goal_done:
                self.get_logger().info('Posa intermedia completata. Invio configurazione finale.')
                self.invia_punto(self.current_torso, self.current_arm_q)
                self.state = 2

        elif self.state == 2:
            if self.torso_goal_done and self.arm_goal_done:
                self.get_logger().info('Configurazione finale completata. Fine sequenza.')
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
        future = self.arm_client.send_goal_async(arm_goal)
        future.add_done_callback(self.arm_goal_response_callback)

    def invia_punto(self, torso_val, arm_vals):
        self.get_logger().info(f' Configurazione torso: {torso_val}, braccio: {arm_vals}')
        torso_goal = FollowJointTrajectory.Goal()
        torso_goal.trajectory.joint_names = ['torso_lift_joint']
        torso_pt = JointTrajectoryPoint()
        torso_pt.positions = [torso_val]
        torso_pt.time_from_start.sec = 5
        torso_goal.trajectory.points.append(torso_pt)

        self.torso_goal_done = False
        future = self.torso_client.send_goal_async(torso_goal)
        future.add_done_callback(self.torso_goal_response_callback)

        self.invia_punto_arm(arm_vals)  # Reinvio anche il braccio (pose finale), quindi resettiamo il flag
        self.arm_goal_done = False

    def arm_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal braccio rifiutato.')
            return

        self.get_logger().info('Goal braccio accettato.')
        goal_handle.get_result_async().add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Movimento braccio completato.')
        self.arm_goal_done = True

    def torso_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal torso rifiutato.')
            return

        self.get_logger().info('Goal torso accettato.')
        goal_handle.get_result_async().add_done_callback(self.torso_result_callback)

    def torso_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Movimento torso completato.')
        self.torso_goal_done = True


def main(args=None):
    rclpy.init(args=args)
    nodo = MacchinaStati()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
