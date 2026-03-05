import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool

class HeadMovementClient(Node):

    """
    Scansione continua della testa finché tutti i marker ArUco non vengono trovati.
    Alterna destra/sinistra (20s per passaggio) e si ferma quando riceve /all_markers_found=True.
    """

    def __init__(self):
        super().__init__("head_movement_client")

        self._client=ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")

        # Stato scansione: alterna tra destra (+0.50) e sinistra (-0.50)
        self.markers_found=False
        self.scan_direction=1  # +1 = destra, -1 = sinistra

        # Si ferma appena il detector segnala che tutti i marker sono stati trovati
        self.create_subscription(Bool, '/all_markers_found', self.markers_found_callback, 10)

    def markers_found_callback(self, msg):
        if msg.data and not self.markers_found:
            self.markers_found=True
            self.get_logger().info("Tutti i marker trovati - scansione testa completata.")

    def send_goal(self):
        if self.markers_found:
            return

        goal=FollowJointTrajectory.Goal()
        goal.trajectory=JointTrajectory()
        goal.trajectory.joint_names=["head_1_joint", "head_2_joint"]

        # 20s per ogni passaggio (adeguato ai gap camera ~34s su WSL2)
        target_pos=0.50 * self.scan_direction
        p=JointTrajectoryPoint()
        p.positions=[target_pos, -0.57]
        p.time_from_start.sec=20

        goal.trajectory.points=[p]

        self.get_logger().info(f"Scansione verso {'destra' if self.scan_direction > 0 else 'sinistra'} ({target_pos:.2f})...")
        self._client.wait_for_server()
        self._send_goal_future=self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle=future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Non è possibile eseguire il movimento.")
            return

        self.get_logger().info("Movimento della testa iniziato. Esecuzione in corso...")
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info("Passaggio completato.")

        if self.markers_found:
            self.get_logger().info("Scansione terminata: tutti i marker trovati.")
            return

        # Cambia direzione e invia nuovo passaggio
        self.scan_direction *= -1
        self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node=HeadMovementClient()
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
