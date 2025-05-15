import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, ERobot
import rclpy
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# modo 1 per caricare robot --> urdf
urdf_loc = '/home/claudio/progetto_ros2/dev_ros/file buoni ma non utilizzati/kinematicToolbox_pkg/kinematicToolbox_pkg/my_robot.urdf'
robot = ERobot.URDF(urdf_loc)
print(robot)

# modo 2 --> parametri DH
links = [
    RevoluteDH(d=0.08, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0.17, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0, a=0, alpha=np.pi/2),
    RevoluteDH(d=0.25, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0.04, a=0, alpha=0)
]
robot = DHRobot(links)
print(robot)
joint_angle_ranges = np.array([
        [-150 * (np.pi / 180), 114 * (np.pi / 180)],
        [-67 * (np.pi / 180), 109 * (np.pi / 180)],
        [-150 * (np.pi / 180), 41 * (np.pi / 180)],
        [-92 * (np.pi / 180), 110 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)],
        [92 * (np.pi / 180), 113 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)]
    ])

# Configurazione iniziale e finale dell'end-effector
q0 = np.mean(joint_angle_ranges, axis=1)
qf = joint_angle_ranges[:, 1]    # Limiti superiori
 # Pose
T0 = robot.fkine(q0)
Tf = robot.fkine(qf)

# 1. Traiettoria cartesiana tra le due pose (SE3)
N = 100
Ts = rtb.ctraj(T0, Tf, N)  # genera N pose interpolate

# 2. Inverse kinematics per ciascun frame
q_traj = []
q_curr = q0
for T in Ts:
    sol = robot.ik_NR(T, q0=q_curr, pinv=True)
    if sol is not None and len(sol) > 0:
        q_curr = sol[0]
    q_traj.append(q_curr)

q_traj = np.array(q_traj)

# 3. Visualizzazione
robot.plot(q_traj, block=True)

# 4. publisher per inviare la traiettoria su Gazebo
class GazeboTrajectoryPublisher(Node):
    def __init__(self, q_traj):
        super().__init__('gazebo_trajectory_publisher')
        self.q_traj = q_traj
        self.index = 0
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.index >= len(self.q_traj):
            self.get_logger().info('Traiettoria completata.')
            self.destroy_timer(self.timer)
            return

        q = self.q_traj[self.index]
        msg = Float64MultiArray()
        msg.data = q.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pubblicato: {msg.data}')
        self.index += 1

def main():
    rclpy.init()
    node = GazeboTrajectoryPublisher(q_traj)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
