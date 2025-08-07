import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import transforms3d
import cv2
import numpy as np
from std_msgs.msg import Int32
from enum import Enum


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Subscribers
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)

        # Publishers
        self.stop_pub = self.create_publisher(Bool, '/all_markers_found', 10)
        self.discovery_pub = self.create_publisher(Int32, '/marker_discovered', 10)

        self.pose_publishers = {
            1: self.create_publisher(PoseStamped, '/aruco_pose_1', 10),
            2: self.create_publisher(PoseStamped, '/aruco_pose_2', 10),
            3: self.create_publisher(PoseStamped, '/aruco_pose_3', 10),
            4: self.create_publisher(PoseStamped, '/aruco_pose_4', 10),
        }

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Utils
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.found_ids = set()
        self.poses_dict = {}
        self.already_published = False

        self.get_logger().info('Nodo ArUco inizializzato.')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Calibrazione della camera ricevuta.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # Disegna i marker se presenti
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        # Mostra sempre l'immagine della telecamera, anche senza ArUco
        cv2.imshow("ArUco Rilevati - Camera View", img)
        cv2.waitKey(1)
        
        if ids is None:
            return
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, self.camera_matrix, self.dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) in self.found_ids:
                continue

            self.found_ids.add(int(marker_id))
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            rot_mat, _ = cv2.Rodrigues(rvec)
            quat_cm = transforms3d.quaternions.mat2quat(rot_mat)

            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    'head_front_camera_color_optical_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().warn(f"TF lookup fallito: {e}")
                continue

            t = transform.transform.translation
            q = transform.transform.rotation
            T_cb = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])
            T_cb = np.vstack((np.hstack((T_cb, np.zeros((3,1)))), [0, 0, 0, 1]))
            T_cb[:3, 3] = [t.x, t.y, t.z]

            T_cm = transforms3d.quaternions.quat2mat(quat_cm)
            T_cm = np.vstack((np.hstack((T_cm, np.zeros((3,1)))), [0, 0, 0, 1]))
            T_cm[:3, 3] = tvec

            T_base_marker = np.dot(T_cb, T_cm)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_footprint'
            pose.pose.position.x = float(T_base_marker[0, 3])
            pose.pose.position.y = float(T_base_marker[1, 3])
            pose.pose.position.z = float(T_base_marker[2, 3])
            q_bm = transforms3d.quaternions.mat2quat(T_base_marker[:3, :3])
            pose.pose.orientation.w = float(q_bm[0])
            pose.pose.orientation.x = float(q_bm[1])
            pose.pose.orientation.y = float(q_bm[2])
            pose.pose.orientation.z = float(q_bm[3])

            # pose_pub rimosso - non utilizzato

            # Pubblica su topic dedicato
            # invece di "if marker_id not in self.pose_publishers:"
            if marker_id in self.pose_publishers:
                self.pose_publishers[marker_id].publish(pose)
            else:
                self.get_logger().warn(f"ID {marker_id} fuori range")

            self.get_logger().info(f'🎯 ArUco {marker_id} rilevato: '
                                   f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})')
            
            # Notifica scoperta immediata
            discovery_msg = Int32()
            discovery_msg.data = marker_id
            self.discovery_pub.publish(discovery_msg)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = pose.header.stamp
            tf_msg.header.frame_id = 'base_footprint'
            tf_msg.child_frame_id = f'aruco_marker_{marker_id}'
            tf_msg.transform.translation.x = pose.pose.position.x
            tf_msg.transform.translation.y = pose.pose.position.y
            tf_msg.transform.translation.z = pose.pose.position.z
            tf_msg.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

            self.poses_dict[int(marker_id)] = pose

        if len(self.found_ids) >= 4 and not self.already_published:
            self.get_logger().info('🎉 Tutti e 4 i marker trovati! Sistema pronto per manipolazione.')
            self.stop_pub.publish(Bool(data=True))
            self.already_published = True
            cv2.destroyAllWindows()  # Chiude finestra debug

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
