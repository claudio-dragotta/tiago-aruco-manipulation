#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np

from roboticstoolbox import URDF
from spatialmath import SE3

class MacchinaAStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati')
        # 1) Carica URDF via RTB-P
        urdf_path = '/home/claudio/progetto_ros2/dev_ros/tiago_robot.urdf'
        self.get_logger().info(f'Carico URDF da {urdf_path}')
        self.robot = URDF.load(urdf_path)
        self.n = self.robot.n
        self.get_logger().info(f'Robot con {self.n} giunti')

        # 2) Publisher JointState
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)

        # 3) Configurazioni
        self.q_torso_up = np.array([0.2])
        self.q_inter = np.array([0.0004, -0.0002, 0.0, -0.00006, 0.00004, 0.0019, 0.0004])
        self.marker_poses = {}

        # 4) Subscribe Aruco
        for m in [1,2,3,4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_pose_base_{m}',
                lambda msg, mid=m: self.cb_marker(msg, mid),
                10
            )

        # 5) Avvia la logica dopo 1s
        self.create_timer(1.0, self.execute_task, once=True)

    def cb_marker(self, msg, mid):
        self.marker_poses[mid] = msg.pose
        self.get_logger().debug(f'Marker {mid} → {msg.pose.position}')

    def publish_js(self, q_all):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['torso_lift_joint'] + \
                  [f'arm_{i}_joint' for i in range(1, self.n+1)] + \
                  ['gripper_finger_joint']
        js.position = q_all.tolist()
        self.pub_js.publish(js)

    def execute_task(self):
        # aspetto tutte e 4 le pose
        if any(m not in self.marker_poses for m in [1,2,3,4]):
            self.get_logger().info('In attesa delle pose Aruco…')
            self.create_timer(1.0, self.execute_task, once=True)
            return

        # sequenza pick→place
        seq = [(1,4),(2,3)]
        # partenza: torso up + config intermedia + gripper aperto
        q_curr = np.hstack((self.q_torso_up, self.q_inter, [0.0]))
        self.publish_js(q_curr)
        self.get_logger().info('Posizione intermedia inviata')

        def toSE3(p):
            return SE3(p.position.x, p.position.y, p.position.z,
                       p.orientation.x, p.orientation.y,
                       p.orientation.z, p.orientation.w)

        for pick_id, place_id in seq:
            self.get_logger().info(f'** Ciclo pick {pick_id} → place {place_id} **')
            p = self.marker_poses[pick_id]
            d = self.marker_poses[place_id]

            # 1) solleva torso
            q_curr[0] = self.q_torso_up
            self.publish_js(q_curr)

            # 2) approach e pick
            for fase, z_off in [('approach',0.10), ('pick',0.02)]:
                T = toSE3(p)
                T.z += z_off
                self.get_logger().info(f'IK {fase}: target =\n{T}')
                try:
                    sol = self.robot.ikine_LM(T, q0=q_curr[1:1+self.n], mask=[1,1,1,0,0,0])
                    self.get_logger().info(f'Soluzione IK: {sol.q}')
                except Exception as e:
                    self.get_logger().error(f'IK {fase} FALLITA: {e}')
                    return
                # invia traiettoria
                traj = self.robot.jtraj(q_curr[1:1+self.n], sol.q, 50)
                for qj in traj.q:
                    self.publish_js(np.hstack((q_curr[0], qj, [0.0])))
                q_curr = np.hstack((q_curr[0], sol.q, [0.0]))
                self.get_logger().info(f'{fase} completata')

            # 3) chiudi gripper
            q_curr[-1] = 1.0
            self.publish_js(q_curr)

            # 4) risali e deposita
            for fase, z_off in [('lift',0.10), ('approach_dep',0.15), ('dep',0.05)]:
                T = toSE3(p if fase=='lift' else d)
                T.z += z_off
                self.get_logger().info(f'IK {fase}: target =\n{T}')
                try:
                    sol = self.robot.ikine_LM(T, q0=q_curr[1:1+self.n], mask=[1,1,1,0,0,0])
                    self.get_logger().info(f'Soluzione IK: {sol.q}')
                except Exception as e:
                    self.get_logger().error(f'IK {fase} FALLITA: {e}')
                    return
                traj = self.robot.jtraj(q_curr[1:1+self.n], sol.q, 50)
                for qj in traj.q:
                    self.publish_js(np.hstack((q_curr[0], qj, [1.0])))
                q_curr = np.hstack((q_curr[0], sol.q, [1.0]))
                self.get_logger().info(f'{fase} completata')

            # 5) apri gripper (rilascio)
            q_curr[-1] = 0.0
            self.publish_js(q_curr)
            self.get_logger().info('Oggetto rilasciato')

        self.get_logger().info('*** TASK COMPLETATO ***')

def main():
    rclpy.init()
    node = MacchinaAStati()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
