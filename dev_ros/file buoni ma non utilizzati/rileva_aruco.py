#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF2 per trasformare tra frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sottoscrizioni
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)

        # Publisher pose trasformata
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose_base', 10)

        self.get_logger().info('🎯 Nodo ArUco pronto.')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info_once('📷 Parametri camera ricevuti.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('⚠️ In attesa dei parametri della camera.')
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.06, self.camera_matrix, self.dist_coeffs)

                pose = PoseStamped()
                pose.header = msg.header
                pose.header.frame_id = 'head_front_camera_rgb_optical_frame'
                pose.pose.position.x = float(tvec[0][0][0])
                pose.pose.position.y = float(tvec[0][0][1])
                pose.pose.position.z = float(tvec[0][0][2])

                # Calcolo orientamento
                rotation_matrix, _ = cv2.Rodrigues(rvec[0][0])
                quat = self.rotation_matrix_to_quaternion(rotation_matrix)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                # Trasforma la posa in base_footprint
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_footprint',
                        pose.header.frame_id,
                        rclpy.time.Time()
                    )

                    pose_base = do_transform_pose(pose, transform)
                    pose_base.header.stamp = self.get_clock().now().to_msg()
                    pose_base.header.frame_id = 'base_footprint'

                    self.pose_pub.publish(pose_base)
                    self.get_logger().info(
                        f'📌 Marker ID {marker_id} rilevato. '
                        f'Posizione in base_footprint: '
                        f'x={pose_base.pose.position.x:.2f}, '
                        f'y={pose_base.pose.position.y:.2f}, '
                        f'z={pose_base.pose.position.z:.2f}'
                    )
                except Exception as e:
                    self.get_logger().warn(f'❌ Trasformazione fallita: {e}')

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
    nodo = ArucoDetector()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
