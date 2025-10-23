import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HeadMovementClient(Node):

    """
    Il nodo permette il movimento della testa di TiaGo dalla posizione [0.217, -0.57] alla posizione [-0.217, -0.57] ai fini
    della scansione del tavolo; viene utilizzato un Action Client.
    """

    def __init__(self):
        super().__init__("head_movement_client")

        #Action client per il movimento della testa
        self._client=ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")

    def send_goal(self):

        #Crea un messaggio FollowJointTrajectory per l'invio del comando di movimento della testa
        goal=FollowJointTrajectory.Goal()
        goal.trajectory=JointTrajectory()
        goal.trajectory.joint_names=["head_1_joint", "head_2_joint"]

        #Punto di partenza (raggiunge la posizione in 4 secondo)
        starting_point=JointTrajectoryPoint()
        starting_point.positions=[0.217, -0.57]
        starting_point.time_from_start.sec=4

        #Punto di arrivo (raggiunge la posizione in 6 secondi)
        final_point=JointTrajectoryPoint()
        final_point.positions=[-0.217, -0.57]
        final_point.time_from_start.sec=6

        goal.trajectory.points=[starting_point, final_point]

        #Invia il goal ed esegue goal_response_callback
        self._client.wait_for_server()
        self._send_goal_future=self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle=future.result()

        #In caso di errore invia un warn a terminale ed esce
        if not goal_handle.accepted:
            self.get_logger().warn("Non è possibile eseguire il movimento.")
            return

        #Altrimenti inizia il movimento della testa ed esegue goal_result_callback al completamento del movimento
        self.get_logger().info("Movimento della testa iniziato. Esecuzione in corso...")
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info("Movimento completato.")


def main(args=None):
    rclpy.init(args=args)
    node=HeadMovementClient()
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
