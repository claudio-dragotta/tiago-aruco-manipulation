
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HeadMover(Node):
    def __init__(self):
        super().__init__('head_movement_action_node')
        self.stop = False
        self.direction = 1
        self.joint_names = ['head_1_joint', 'head_2_joint']
        self.pan_angle = 0.6
        self.tilt_angle = -0.57
        self.client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.create_subscription(Bool, '/all_markers_found', self.stop_callback, 10)

        self.get_logger().info('Head Movement Node pronto - in attesa di comando...')
        self.client.wait_for_server()
        self.get_logger().info('Movimento testa disponibile')
        
        # Avvia automaticamente il movimento per la ricerca ArUco
        self.send_goal()

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Esplorazione completata - movimento testa arrestato')
            self.stop = True

    def send_goal(self):
        if self.stop:
            self.get_logger().info('Movimento testa interrotto')
            return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.pan_angle * self.direction, self.tilt_angle]
        point.time_from_start = Duration(sec=2)
        traj.points = [point]
        goal.trajectory = traj

        self.client.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato!')
            return
        self.get_logger().info(f'Scansione direzione {"sinistra" if self.direction > 0 else "destra"}')
        goal_handle.get_result_async().add_done_callback(self.goal_done)

    def goal_done(self, future):
        if not self.stop:
            self.direction *= -1
            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    node = HeadMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
