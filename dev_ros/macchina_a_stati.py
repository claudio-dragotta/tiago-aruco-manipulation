import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped


class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        self.state = 0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose_base_1',  # supponiamo che marker ID 1 sia quello d'interesse
            self.pose_callback,
            10
        )

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.pose_rilevata = None
        self.get_logger().info('🧠 Nodo macchina_a_stati avviato.')

        self.timer = self.create_timer(1.0, self.esegui_macchina_stati)

    def pose_callback(self, msg):
        self.pose_rilevata = msg

    def esegui_macchina_stati(self):
        if self.state == 0:
            self.get_logger().info('🟡 Stato 0: invio configurazione intermedia')
            configurazione_intermedia = [
                0.0003836728901962516,
                -0.0001633239063343339,
                -9.037018213753356e-06,
                -6.145563957549172e-05,
                4.409014973383307e-05,
                0.0019643255648595925,
                0.0004167305736686444
            ]
            self.invia_trajectory(configurazione_intermedia)
            self.state = 1

        elif self.state == 1 and self.pose_rilevata is not None:
            self.get_logger().info('🟡 Stato 1: invio configurazione finale verso marker')
            x = self.pose_rilevata.pose.position.x
            y = self.pose_rilevata.pose.position.y
            z = self.pose_rilevata.pose.position.z

            self.get_logger().info(f'🎯 Marker rilevato a x={x:.2f}, y={y:.2f}, z={z:.2f}')

            # Inserire qui eventuale calcolo della configurazione tramite IK
            # Configurazione di esempio:
            configurazione_finale = [0.2, -0.6, 0.3, 1.2, 0.1, -0.5, 0.0]
            self.invia_trajectory(configurazione_finale)
            self.state = 2

        elif self.state == 2:
            self.get_logger().info('🤖 Stato 2: chiusura gripper')
            self.controlla_gripper(chiudi=True)
            self.state = 3

        elif self.state == 3:
            self.get_logger().info('✅ Fine sequenza')
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
        posizione = 0.0 if chiudi else 0.04  # chiuso = 0.0, aperto = 0.04
        punto.positions = [posizione, posizione]
        punto.time_from_start.sec = 2
        goal.trajectory.points.append(punto)

        self.gripper_client.send_goal_async(goal)


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
