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

        # Debug: stampa topic disponibili
        self.get_logger().info("ArUco Detector avviato - Debug attivo")

        # Subscribers
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        
        # Debug callback counters
        self.image_count = 0
        self.detection_attempts = 0

        # Publishers
        self.stop_pub = self.create_publisher(Bool, '/all_markers_found', 10)
        self.discovery_pub = self.create_publisher(Int32, '/marker_discovered', 10)

        # Publishers per ogni marker (1-4)
        # Pubblica sui topic che ik.py si aspetta: /aruco_base_pose_X
        self.pose_publishers = {}
        self.pose_base_publishers = {}
        for marker_id in [1, 2, 3, 4]:
            # Topic principale per IK node (formato richiesto: /aruco_base_pose_X)
            self.pose_publishers[marker_id] = self.create_publisher(
                PoseStamped, f'/aruco_base_pose_{marker_id}', 10
            )
            # Topic addizionale per macchina_a_stati (formato: /aruco_pose_base_X)
            self.pose_base_publishers[marker_id] = self.create_publisher(
                PoseStamped, f'/aruco_pose_base_{marker_id}', 10
            )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Utils
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Parametri ArUco più permissivi per debug
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        self.aruco_params.minMarkerDistanceRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1

        self.found_ids = set()
        self.poses_dict = {}
        self.already_published = False
        
        # Timer per debug periodico
        self.create_timer(5.0, self.debug_status)

        self.get_logger().info('Nodo ArUco inizializzato con parametri debug.')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Calibrazione camera ricevuta: {self.camera_matrix[0,0]:.1f}px focal length')

    def image_callback(self, msg):
        self.image_count += 1
        
        if self.camera_matrix is None:
            if self.image_count % 100 == 0:  # Log ogni 100 immagini
                self.get_logger().warn(f'Camera matrix non ricevuta ancora (immagini: {self.image_count})')
            return
        
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Tentativo detection
            self.detection_attempts += 1
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            # Debug: aggiungi info sull'immagine
            debug_img = img.copy()
            
            # Disegna i marker se presenti
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
                self.get_logger().info(f'RILEVATI {len(ids)} marker: {ids.flatten().tolist()}')
            
            # Aggiungi testo debug all'immagine
            cv2.putText(debug_img, f'Immagini: {self.image_count}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f'Trovati: {len(self.found_ids)}/4', (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f'Risoluzione: {img.shape[1]}x{img.shape[0]}', (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Mostra immagine con debug
            cv2.imshow("ArUco Debug - Camera View", debug_img)
            cv2.waitKey(1)
            
            if ids is None:
                return
                
        except Exception as e:
            self.get_logger().error(f'Errore nella conversione immagine: {e}')
            return
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, self.camera_matrix, self.dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            marker_id_int = int(marker_id)
            
            # Rilevamento progressivo: anche se già trovato, processa sempre
            if marker_id_int not in self.found_ids:
                self.found_ids.add(marker_id_int)
                self.get_logger().info(f'NUOVO MARKER SCOPERTO: {marker_id_int} (totale: {len(self.found_ids)}/4)')
            
            # Processa sempre per aggiornare pose anche di marker già noti
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

            # Pose in base_footprint frame (only one we need)
            base_pose = PoseStamped()
            base_pose.header.stamp = self.get_clock().now().to_msg()
            base_pose.header.frame_id = 'base_footprint'
            base_pose.pose.position.x = float(T_base_marker[0, 3])
            base_pose.pose.position.y = float(T_base_marker[1, 3])
            base_pose.pose.position.z = float(T_base_marker[2, 3])
            q_bm = transforms3d.quaternions.mat2quat(T_base_marker[:3, :3])
            base_pose.pose.orientation.w = float(q_bm[0])
            base_pose.pose.orientation.x = float(q_bm[1])
            base_pose.pose.orientation.y = float(q_bm[2])
            base_pose.pose.orientation.z = float(q_bm[3])

            # Pubblica sui topic corretti per entrambi i sistemi
            if marker_id_int in self.pose_publishers:
                # Topic principale per IK node (/aruco_base_pose_X)
                self.pose_publishers[marker_id_int].publish(base_pose)
                # Topic addizionale per macchina_a_stati (/aruco_pose_base_X)
                if marker_id_int in self.pose_base_publishers:
                    self.pose_base_publishers[marker_id_int].publish(base_pose)
                    
                self.get_logger().info(f'ArUco {marker_id_int} pubblicato sui topic di comunicazione')
            else:
                self.get_logger().warn(f"ID {marker_id_int} fuori range")

            self.get_logger().info(f'ArUco {marker_id_int} rilevato: '
                                   f'Base({base_pose.pose.position.x:.2f}, {base_pose.pose.position.y:.2f}, {base_pose.pose.position.z:.2f})')
            
            # Notifica scoperta SOLO per marker nuovi (non re-detection)
            if marker_id_int not in self.poses_dict:
                discovery_msg = Int32()
                discovery_msg.data = marker_id_int
                self.discovery_pub.publish(discovery_msg)
                self.get_logger().info(f'📢 SEGNALATO NUOVO MARKER {marker_id_int} alla state machine')

            tf_msg = TransformStamped()
            tf_msg.header.stamp = base_pose.header.stamp
            tf_msg.header.frame_id = 'base_footprint'
            tf_msg.child_frame_id = f'aruco_marker_{marker_id_int}'
            tf_msg.transform.translation.x = base_pose.pose.position.x
            tf_msg.transform.translation.y = base_pose.pose.position.y
            tf_msg.transform.translation.z = base_pose.pose.position.z
            tf_msg.transform.rotation = base_pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

            self.poses_dict[marker_id_int] = base_pose

        # Condizione più flessibile: procede quando ha trovato almeno 2 marker 
        # (o tutti e 4 se li trova rapidamente)
        if len(self.found_ids) >= 2 and not self.already_published:
            # Aspetta un po' per vedere se trova altri marker
            if not hasattr(self, 'discovery_timer_started'):
                self.get_logger().info(f'⏳ Trovati {len(self.found_ids)}/4 marker. Aspetto 10 secondi per eventuali altri...')
                self.create_timer(10.0, self.finalize_discovery)
                self.discovery_timer_started = True
        elif len(self.found_ids) >= 4 and not self.already_published:
            # Se trova tutti e 4, procede immediatamente
            self.finalize_discovery()

    def finalize_discovery(self):
        """Finalizza la fase di scoperta dei marker"""
        if self.already_published:
            return
            
        self.get_logger().info(f'FASE SCOPERTA COMPLETATA! Marker trovati: {len(self.found_ids)}/4 -> {sorted(list(self.found_ids))}')
        self.get_logger().info('📍 Posizioni salvate per la manipolazione. Sistema pronto!')
        
        # Segnala alla state machine che può procedere con la manipolazione
        self.stop_pub.publish(Bool(data=True))
        self.already_published = True
        
        # Mantiene la finestra debug aperta per monitoraggio durante manipolazione
        # cv2.destroyAllWindows()  

    def debug_status(self):
        """Debug periodico dello stato del detector"""
        self.get_logger().info(f'=== DEBUG ARUCO ===')
        self.get_logger().info(f'Immagini ricevute: {self.image_count}')
        self.get_logger().info(f'Tentativi detection: {self.detection_attempts}')
        self.get_logger().info(f'Camera calibrata: {self.camera_matrix is not None}')
        self.get_logger().info(f'Marker trovati: {len(self.found_ids)}/4 -> {list(self.found_ids)}')
        
        if self.image_count == 0:
            self.get_logger().warn('PROBLEMA: Nessuna immagine ricevuta! Verifica topic camera.')
        elif self.camera_matrix is None:
            self.get_logger().warn('PROBLEMA: Camera info non ricevuta! Verifica topic camera_info.')
        elif self.detection_attempts > 100 and len(self.found_ids) == 0:
            self.get_logger().warn('PROBLEMA: Molti tentativi ma nessun marker rilevato! Verifica marker ArUco nella scena.')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
