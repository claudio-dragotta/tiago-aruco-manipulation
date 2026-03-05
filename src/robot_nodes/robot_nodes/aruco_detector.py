import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class ArucoDetector(Node):

    """
    Rileva marker ArUco dalla camera di TIAGo e pubblica le pose in base_footprint.
    Approccio identico al progetto di riferimento:
    - Timer 1Hz per la detection (salva ultimo frame nel callback)
    - Immagine a colori (bgr8) - il grayscale causa difficolta di rilevamento
    - DetectorParameters() di default
    - scipy per le rotazioni
    - Accumulo delle pose e pubblicazione della media (come aruco_coord_transformation.py)
    """

    def __init__(self):
        super().__init__('aruco_detector_node')

        # Mapping: ID fisico ArUco nel mondo Gazebo -> ID logico del sistema
        # Coca-Cola:       ArUco ID 26 -> logico 1
        # Pringles:        ArUco ID 63 -> logico 2
        # Dest Coca-Cola:  ArUco ID 3  -> logico 3
        # Dest Pringles:   ArUco ID 4  -> logico 4
        self.id_mapping = {26: 1, 63: 2, 3: 3, 4: 4}

        # Subscription alla camera (come aruco_scan_publisher.py del collega)
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.callback_function, 10)

        # Publishers pose in base_footprint per l'IK node
        self.pose_publishers = {}
        for mid in [1, 2, 3, 4]:
            self.pose_publishers[mid] = self.create_publisher(PoseStamped, f'/aruco_base_pose_{mid}', 10)

        # Publisher segnale "tutti i marker trovati" per la state machine
        self.stop_pub = self.create_publisher(Bool, '/all_markers_found', 10)
        self.discovery_pub = self.create_publisher(Int32, '/marker_discovered', 10)

        # Publisher immagine debug (visualizzabile con rqt_image_view)
        self.debug_image_pub = self.create_publisher(Image, '/aruco_debug_image', 10)
        self.markers_viz_pub = self.create_publisher(MarkerArray, '/aruco_markers_viz', 10)

        self.bridge = CvBridge()
        self.last_image = None       # Ultimo frame a colori (aggiornato nel callback)
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF listener per trasformazione camera -> base_footprint
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Accumulo pose per fare la media (come aruco_coord_transformation.py del collega)
        self.pose_sums = {}    # {logical_id: dict con somme x,y,z,qx,qy,qz,qw}
        self.pose_counts = {}  # {logical_id: numero osservazioni}
        self.pose_final = {}   # {logical_id: PoseStamped media corrente}

        self.found_ids = set()
        self.already_published = False
        self.frame_count = 0

        # Timer 1Hz per detection - identico al collega (self.create_timer(1.0, self.publish_aruco_pose))
        self.create_timer(1.0, self.publish_aruco_pose)
        # Timeout: dopo 60s procedi con i marker trovati (minimo 1)
        self.create_timer(60.0, self.timeout_fallback)

        self.get_logger().info("ArUco Detector avviato (approccio del progetto di riferimento)")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera calibrata: focal={self.camera_matrix[0,0]:.1f}px')

    def callback_function(self, msg):
        # Salva ultima immagine a colori bgr8 - identico al collega
        if msg is not None:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def publish_aruco_pose(self):
        # Identico a publish_aruco_pose del collega: detection su immagine a colori, 1Hz

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()  # parametri di default come il collega

        if self.last_image is None:
            return

        if self.camera_matrix is None:
            self.get_logger().warn('Camera matrix non ancora ricevuta')
            return

        self.frame_count += 1
        corners, ids, _ = cv2.aruco.detectMarkers(self.last_image, aruco_dict, parameters=parameters)

        # Pubblica immagine debug con marker evidenziati
        debug_img = self.last_image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
            # Log TUTTI gli ID rilevati (anche quelli non nel mapping) per debug
            all_ids = ids.flatten().tolist()
            mapped = [i for i in all_ids if i in self.id_mapping]
            unmapped = [i for i in all_ids if i not in self.id_mapping]
            self.get_logger().info(f'[frame {self.frame_count}] ArUco rilevati: {all_ids} | mappati={mapped} | NON mappati={unmapped}')
            found_text = f'Trovati: {len(self.found_ids)}/4'
            cv2.putText(debug_img, found_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            if self.frame_count % 5 == 0:
                self.get_logger().info(f'[frame {self.frame_count}] Nessun marker rilevato (immagine OK, camera calibrata: {self.camera_matrix is not None})')
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception:
            pass

        if ids is None:
            return

        marker_length = 0.06
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, self.camera_matrix, self.dist_coeffs)

        for i in range(len(ids)):
            physical_id = int(ids[i])

            if physical_id not in self.id_mapping:
                continue

            logical_id = self.id_mapping[physical_id]

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # Trasformazione da camera frame a base_footprint (identica al collega)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    'head_front_camera_optical_frame',
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(f'TF lookup fallito per marker {logical_id}: {e}')
                continue

            t = transform.transform.translation
            q = transform.transform.rotation

            pos_head = np.array([[t.x], [t.y], [t.z]])
            rot_head = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

            rot_mat, _ = cv2.Rodrigues(rvec)
            tvec_col = tvec.reshape(3, 1)

            # Identico al collega: pos_head + rot_head * tvec_marker
            new_pos = pos_head + np.dot(rot_head, tvec_col)
            new_rot_mat = np.dot(rot_head, rot_mat)
            new_quat = Rotation.from_matrix(new_rot_mat).as_quat()  # [x,y,z,w]

            # Prima osservazione di questo marker
            if logical_id not in self.pose_sums:
                self.pose_sums[logical_id] = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                                               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0}
                self.pose_counts[logical_id] = 0
                self.found_ids.add(logical_id)
                self.get_logger().info(f'NUOVO MARKER: fisico={physical_id} -> logico={logical_id} ({len(self.found_ids)}/4)')

                discovery_msg = Int32()
                discovery_msg.data = logical_id
                self.discovery_pub.publish(discovery_msg)

            # Accumula (come aruco_coord_transformation.py del collega)
            s = self.pose_sums[logical_id]
            s['x'] += float(new_pos[0])
            s['y'] += float(new_pos[1])
            s['z'] += float(new_pos[2])
            s['qx'] += float(new_quat[0])
            s['qy'] += float(new_quat[1])
            s['qz'] += float(new_quat[2])
            s['qw'] += float(new_quat[3])
            self.pose_counts[logical_id] += 1

            # Pubblica la media corrente
            count = self.pose_counts[logical_id]
            avg_pose = PoseStamped()
            avg_pose.header.stamp = self.get_clock().now().to_msg()
            avg_pose.header.frame_id = 'base_footprint'
            avg_pose.pose.position.x = s['x'] / count
            avg_pose.pose.position.y = s['y'] / count
            avg_pose.pose.position.z = s['z'] / count
            avg_pose.pose.orientation.x = s['qx'] / count
            avg_pose.pose.orientation.y = s['qy'] / count
            avg_pose.pose.orientation.z = s['qz'] / count
            avg_pose.pose.orientation.w = s['qw'] / count

            self.pose_publishers[logical_id].publish(avg_pose)
            self.pose_final[logical_id] = avg_pose

            pos = avg_pose.pose.position
            self.get_logger().info(f'Marker {logical_id}: pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) [N={count}]')

        # Pubblica visualizzazione RViz
        self.publish_markers_viz()

        # Controlla se tutti i marker sono stati trovati
        if len(self.found_ids) >= 4 and not self.already_published:
            self.get_logger().info(f'TUTTI I 4 MARKER TROVATI: {sorted(list(self.found_ids))}')
            self.stop_pub.publish(Bool(data=True))
            self.already_published = True
            self.create_timer(2.0, self._republish_all_found)
        elif len(self.found_ids) > 0:
            missing = set([1, 2, 3, 4]) - self.found_ids
            if len(missing) > 0:
                self.get_logger().info(f'Trovati {len(self.found_ids)}/4 - mancano: {sorted(list(missing))}')

    def timeout_fallback(self):
        """Dopo 60s pubblica all_markers_found comunque per non bloccare il sistema"""
        if self.already_published:
            return
        if len(self.found_ids) >= 1:
            self.get_logger().warn(f'TIMEOUT 60s: trovati {len(self.found_ids)}/4 marker {sorted(list(self.found_ids))} - procedo comunque!')
        else:
            self.get_logger().error('TIMEOUT 60s: ZERO marker trovati!')
            self.get_logger().error(f'ID attesi (fisici): {list(self.id_mapping.keys())} -> logici: {list(self.id_mapping.values())}')
            self.get_logger().error('Invio segnale alla state machine comunque per sbloccare il sistema')
        self.stop_pub.publish(Bool(data=True))
        self.already_published = True
        self.create_timer(2.0, self._republish_all_found)

    def _republish_all_found(self):
        self.stop_pub.publish(Bool(data=True))
        for logical_id, pose in self.pose_final.items():
            self.pose_publishers[logical_id].publish(pose)

    def publish_markers_viz(self):
        if not self.pose_final:
            return

        colors = {1: (1.0, 0.2, 0.2), 2: (0.2, 0.6, 1.0),
                  3: (0.2, 1.0, 0.2), 4: (1.0, 0.8, 0.0)}
        names = {1: 'CocaCola', 2: 'Pringles', 3: 'DestCoca', 4: 'DestPring'}

        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for mid, pose in self.pose_final.items():
            r, g, b = colors.get(mid, (1.0, 1.0, 1.0))
            pos = pose.pose.position

            sphere = Marker()
            sphere.header.frame_id = 'base_footprint'
            sphere.header.stamp = now
            sphere.ns = 'aruco'
            sphere.id = mid
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pos.x
            sphere.pose.position.y = pos.y
            sphere.pose.position.z = pos.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            sphere.lifetime.sec = 3
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = 'base_footprint'
            label.header.stamp = now
            label.ns = 'aruco_labels'
            label.id = mid + 10
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pos.x
            label.pose.position.y = pos.y
            label.pose.position.z = pos.z + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.1
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{names.get(mid, str(mid))}\n({pos.x:.2f},{pos.y:.2f},{pos.z:.2f})'
            label.lifetime.sec = 3
            array.markers.append(label)

        self.markers_viz_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
