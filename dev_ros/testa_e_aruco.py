#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R
import threading

class TestaEAruco(Node):
    def __init__(self):
        super().__init__('testa_e_aruco')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)

        self.pose_publishers = {}  # Publisher per ogni marker ID

        self.client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.client.wait_for_server()

        self.get_logger().info(' Movimento testa + Rilevamento ArUco avviati...')
        threading.Thread(target=self.muovi_testa).start()

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("ArUco Rilevati", frame)
        cv2.waitKey(1)

        if ids is not None:
            self.get_logger().info(f'Rilevati {len(ids)} marker: {ids.flatten().tolist()}')
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.06, self.camera_matrix, self.dist_coeffs)

                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'head_front_camera_color_optical_frame'

                pose.pose.position.x = float(tvec[0][0][0])
                pose.pose.position.y = float(tvec[0][0][1])
                pose.pose.position.z = float(tvec[0][0][2])

                rot_matrix, _ = cv2.Rodrigues(rvec[0][0])
                q = self.rotation_matrix_to_quaternion(rot_matrix)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_footprint',
                        'head_front_camera_color_optical_frame',
                        rclpy.time.Time()
                    )

                    # Trasformazione manuale
                    T = np.array([transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z])

                    Q = [transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z,
                         transform.transform.rotation.w]

                    R_mat = R.from_quat(Q).as_matrix()
                    tvec_cam = np.array([pose.pose.position.x,
                                         pose.pose.position.y,
                                         pose.pose.position.z])
                    tvec_base = R_mat @ tvec_cam + T

                    pose_base = PoseStamped()
                    pose_base.header.stamp = self.get_clock().now().to_msg()
                    pose_base.header.frame_id = 'base_footprint'
                    pose_base.pose.position.x = float(tvec_base[0])
                    pose_base.pose.position.y = float(tvec_base[1])
                    pose_base.pose.position.z = float(tvec_base[2])
                    pose_base.pose.orientation.x = q[0]
                    pose_base.pose.orientation.y = q[1]
                    pose_base.pose.orientation.z = q[2]
                    pose_base.pose.orientation.w = q[3]

                    # Crea publisher per ogni marker ID
                    if marker_id not in self.pose_publishers:
                        self.pose_publishers[marker_id] = self.create_publisher(
                            PoseStamped, f'/aruco_pose_base_{marker_id}', 10)

                    self.pose_publishers[marker_id].publish(pose_base)
                    self.get_logger().info(f' ArUco {marker_id} in base_footprint: '
                                           f'x={pose_base.pose.position.x:.2f}, '
                                           f'y={pose_base.pose.position.y:.2f}, '
                                           f'z={pose_base.pose.position.z:.2f}')

                except Exception as e:
                    self.get_logger().warn(f'Trasformazione fallita per ArUco {marker_id}: {e}')
        else:
            self.get_logger().info('Nessun marker rilevato')

    def muovi_testa(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        posizioni = [
            (0.217, -0.57),
            (0.0,   -0.57),
            (-0.217, -0.57),
            (0.0,   -0.57),
            (0.100, -0.57)
        ]
        for i, pos in enumerate(posizioni):
            punto = JointTrajectoryPoint()
            punto.positions = list(pos)
            punto.time_from_start.sec = 3 * (i + 1)
            goal.trajectory.points.append(punto)
        self.client.send_goal_async(goal)

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty((4,))
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q[0], q[1], q[2], q[3]

def main(args=None):
    rclpy.init(args=args)
    node = TestaEAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
