import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import numpy as np
from ik_solver_dh_con_orientamento import (
    inverse_kinematics_with_orientation,
    dh_params_tiago,
    joint_limits_tiago
)


class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati_finale_con_torso')
        self.state = 0
        self.posizioni = {}

        for marker_id in [1, 2, 3, 4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_pose_base_{marker_id}',
                self.pose_callback_factory(marker_id),
                10
            )

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')

        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.torso_client.wait_for_server()

        self.get_logger().info('🧠 Nodo macchina con torso avviato.')
        self.timer = self.create_timer(1.0, self.esegui_macchina_stati)

    def pose_callback_factory(self, marker_id):
        def callback(msg):
            self.posizioni[marker_id] = msg
        return callback

    def calcola_cinematica_inversa(self, pose_stamped):
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        target_position = np.array([pos.x, pos.y, pos.z])
        target_quat = np.array([ori.x, ori.y, ori.z, ori.w])

        norm = np.linalg.norm(target_quat)
        if norm == 0:
            self.get_logger().warn("❗ Quaternione nullo. Uso orientamento neutro.")
            target_quat = np.array([0, 0, 0, 1])
        else:
            target_quat = target_quat / norm

        self.get_logger().info(f'🎯 Target pos: {target_position.round(3)}, quat: {target_quat.round(3)}')

        q0 = np.zeros(7)
        q_sol, success = inverse_kinematics_with_orientation(
            dh_params_tiago, q0, target_position, target_quat, joint_limits_tiago
        )

        if not success:
            self.get_logger().warn("⚠️ IK fallita. Uso fallback.")
            return [0.2, -0.6, 0.3, 1.2, 0.1, -0.5, 0.0]

        self.get_logger().info(f'✅ Soluzione IK: {[round(q, 2) for q in q_sol]}')
        return q_sol

    def esegui_macchina_stati(self):
        match self.state:
            case 0:
                self.get_logger().info('🟡 Stato 0: sollevo torso')
                self.alza_torso()
                self.state += 1  # passaggio controllato dopo la conferma

            case 1:
                self.get_logger().info('🟡 Stato 1: configurazione intermedia')
                config = [
                    0.00038, -0.00016, -9e-06,
                    -6e-05, 4e-05, 0.00196, 0.00042
                ]
                self.invia_trajectory(config, next_state=2)

            case 2:
                if 1 in self.posizioni:
                    self.get_logger().info('🟡 Stato 2: vado a prendere oggetto 1')
                    config = self.calcola_cinematica_inversa(self.posizioni[1])
                    self.invia_trajectory(config, next_state=3)

            case 3:
                self.get_logger().info('🤖 Stato 3: chiudo gripper')
                self.controlla_gripper(chiudi=True)
                self.state = 4

            case 4:
                if 3 in self.posizioni:
                    self.get_logger().info('🟡 Stato 4: porto oggetto 1 a destinazione')
                    config = self.calcola_cinematica_inversa(self.posizioni[3])
                    self.invia_trajectory(config, next_state=5)

            case 5:
                self.get_logger().info('🤖 Stato 5: apro gripper')
                self.controlla_gripper(chiudi=False)
                self.state = 6

            case 6:
                if 2 in self.posizioni:
                    self.get_logger().info('🟡 Stato 6: vado a prendere oggetto 2')
                    config = self.calcola_cinematica_inversa(self.posizioni[2])
                    self.invia_trajectory(config, next_state=7)

            case 7:
                self.get_logger().info('🤖 Stato 7: chiudo gripper')
                self.controlla_gripper(chiudi=True)
                self.state = 8

            case 8:
                if 4 in self.posizioni:
                    self.get_logger().info('🟡 Stato 8: porto oggetto 2 a destinazione')
                    config = self.calcola_cinematica_inversa(self.posizioni[4])
                    self.invia_trajectory(config, next_state=9)

            case 9:
                self.get_logger().info('🤖 Stato 9: apro gripper')
                self.controlla_gripper(chiudi=False)
                self.state = 10

            case 10:
                self.get_logger().info('✅ Sequenza COMPLETATA')
                self.timer.cancel()

    def invia_trajectory(self, configurazione, next_state=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        punto = JointTrajectoryPoint()
        punto.positions = configurazione
        punto.time_from_start.sec = 5
        goal.trajectory.points.append(punto)

        send_future = self.arm_client.send_goal_async(goal)
        if next_state is not None:
            send_future.add_done_callback(lambda f: self.set_state(next_state))

    def alza_torso(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        punto = JointTrajectoryPoint()
        punto.positions = [0.25]
        punto.time_from_start.sec = 3
        goal.trajectory.points.append(punto)

        send_future = self.torso_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self.set_state(1))

    def controlla_gripper(self, chiudi=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        punto = JointTrajectoryPoint()
        posizione = 0.0 if chiudi else 0.04
        punto.positions = [posizione, posizione]
        punto.time_from_start.sec = 2
        goal.trajectory.points.append(punto)

        self.gripper_client.send_goal_async(goal)

    def set_state(self, next_state):
        self.get_logger().info(f'➡️ Passaggio a stato {next_state}')
        self.state = next_state


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
