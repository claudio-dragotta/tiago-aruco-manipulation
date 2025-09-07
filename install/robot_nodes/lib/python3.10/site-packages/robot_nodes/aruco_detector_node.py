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
        
        # Sistema automatico di calcolo distanze ottimali
        self.aruco_size = 0.06  # Dimensione marker ArUco (6cm)
        self.object_configs = {
            1: {"width": 0.065, "height": 0.24, "type": "bottle", "name": "Coca-Cola"},
            2: {"width": 0.08, "height": 0.25, "type": "cylinder", "name": "Pringles"},
            3: {"type": "destination", "name": "Dest_CocaCola"},
            4: {"type": "destination", "name": "Dest_Pringles"}
        }
        
        # Controllo log per evitare spam
        self.last_logged_markers = set()
        self.detection_count = 0
        
        # Timer per debug periodico (ogni 10 secondi invece di 5)
        self.create_timer(10.0, self.debug_status)

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
                # Log solo se ci sono nuovi marker rispetto all'ultima detection
                current_markers = set(ids.flatten().tolist())
                if current_markers != self.last_logged_markers:
                    self.get_logger().info(f'RILEVATI {len(ids)} marker: {ids.flatten().tolist()}')
                    self.last_logged_markers = current_markers
            
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
            
            # Processa marker per ottenere la pose PRIMA di loggarla
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
                self.get_logger().warn(f"TF lookup fallito per marker {marker_id_int}: {e}")
                continue

            # Costruzione matrice trasformazione camera->base
            t = transform.transform.translation
            q = transform.transform.rotation
            T_cb = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])
            T_cb = np.vstack((np.hstack((T_cb, np.zeros((3,1)))), [0, 0, 0, 1]))
            T_cb[:3, 3] = [t.x, t.y, t.z]

            # Costruzione matrice trasformazione camera->marker
            T_cm = transforms3d.quaternions.quat2mat(quat_cm)
            T_cm = np.vstack((np.hstack((T_cm, np.zeros((3,1)))), [0, 0, 0, 1]))
            T_cm[:3, 3] = tvec
            
            # Trasformazione finale: base_footprint <- camera <- marker
            T_base_marker = np.dot(T_cb, T_cm)

            # Pose in base_footprint frame
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

            # Rilevamento progressivo: log solo per marker NUOVI con posizione
            if marker_id_int not in self.found_ids:
                self.found_ids.add(marker_id_int)
                pos = base_pose.pose.position
                self.get_logger().info(f'NUOVO MARKER {marker_id_int} SCOPERTO! Posizione Base: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) - Totale: {len(self.found_ids)}/4')
                
                # Notifica scoperta alla state machine
                discovery_msg = Int32()
                discovery_msg.data = marker_id_int
                self.discovery_pub.publish(discovery_msg)
                self.get_logger().info(f'Notificato nuovo marker {marker_id_int} alla state machine')

            # Validazione pose: verifica che sia ragionevole (solo per marker nuovi)
            if marker_id_int not in self.poses_dict:
                final_pos = T_base_marker[:3, 3]
                distance_from_robot = np.sqrt(final_pos[0]**2 + final_pos[1]**2)
                
                if distance_from_robot > 3.0:
                    self.get_logger().warn(f"ATTENZIONE: Marker {marker_id_int} molto lontano ({distance_from_robot:.2f}m)")
                elif distance_from_robot < 0.3:
                    self.get_logger().warn(f"ATTENZIONE: Marker {marker_id_int} molto vicino ({distance_from_robot:.2f}m)")
                
                if final_pos[2] < -0.1:
                    self.get_logger().warn(f"ATTENZIONE: Marker {marker_id_int} sotto il livello robot (z={final_pos[2]:.2f}m)")
                elif final_pos[2] > 2.0:
                    self.get_logger().warn(f"ATTENZIONE: Marker {marker_id_int} troppo alto (z={final_pos[2]:.2f}m)")

            # CALCOLO AUTOMATICO pose ottimali per manipolazione
            optimized_poses = self.calculate_optimal_manipulation_poses(marker_id_int, base_pose)
            
            # Pubblica pose sui topic corretti
            if marker_id_int in self.pose_publishers:
                # Topic principale per IK node (/aruco_base_pose_X) - USA POSE OTTIMIZZATA
                self.pose_publishers[marker_id_int].publish(optimized_poses["manipulation"])
                # Topic addizionale per macchina_a_stati (/aruco_pose_base_X) - Pose originale
                if marker_id_int in self.pose_base_publishers:
                    self.pose_base_publishers[marker_id_int].publish(base_pose)
            else:
                self.get_logger().warn(f"Marker ID {marker_id_int} fuori range (1-4)")

            # Pubblica TF transform
            tf_msg = TransformStamped()
            tf_msg.header.stamp = base_pose.header.stamp
            tf_msg.header.frame_id = 'base_footprint'
            tf_msg.child_frame_id = f'aruco_marker_{marker_id_int}'
            tf_msg.transform.translation.x = base_pose.pose.position.x
            tf_msg.transform.translation.y = base_pose.pose.position.y
            tf_msg.transform.translation.z = base_pose.pose.position.z
            tf_msg.transform.rotation = base_pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

            # Salva pose nel dizionario
            self.poses_dict[marker_id_int] = base_pose

        # Verifica stato scoperta marker
        if len(self.found_ids) >= 4 and not self.already_published:
            self.get_logger().info(f'TUTTI I 4 MARKER TROVATI: {sorted(list(self.found_ids))} - Proceeding alla manipolazione!')
            self.finalize_discovery()
        elif len(self.found_ids) > 0 and len(self.found_ids) < 4:
            # Log progresso solo ogni 20 detection per evitare spam
            if self.detection_attempts % 20 == 0:
                missing_markers = set([1, 2, 3, 4]) - self.found_ids
                self.get_logger().info(f'Progresso: {len(self.found_ids)}/4 marker trovati. Mancano: {sorted(list(missing_markers))}')

    def finalize_discovery(self):
        """Finalizza la fase di scoperta dei marker"""
        if self.already_published:
            return
            
        self.get_logger().info(f'FASE SCOPERTA COMPLETATA!')
        self.get_logger().info(f'Marker rilevati: {sorted(list(self.found_ids))} ({len(self.found_ids)}/4)')
        
        # Stampa riassunto posizioni per debug
        for marker_id in sorted(list(self.found_ids)):
            if marker_id in self.poses_dict:
                pos = self.poses_dict[marker_id].pose.position
                self.get_logger().info(f'  - Marker {marker_id}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
        
        self.get_logger().info('Sistema pronto per la manipolazione!')
        
        # Segnala alla state machine che può procedere
        self.stop_pub.publish(Bool(data=True))
        self.already_published = True  

    def debug_status(self):
        """Debug periodico dello stato del detector"""
        self.get_logger().info(f'=== STATO ARUCO DETECTOR ===')
        self.get_logger().info(f'Immagini ricevute: {self.image_count}')
        self.get_logger().info(f'Tentativi detection: {self.detection_attempts}')
        self.get_logger().info(f'Camera calibrata: {"SI" if self.camera_matrix is not None else "NO"}')
        
        if len(self.found_ids) > 0:
            self.get_logger().info(f'Marker trovati: {len(self.found_ids)}/4 -> {sorted(list(self.found_ids))}')
            # Mostra posizioni dei marker trovati
            for marker_id in sorted(list(self.found_ids)):
                if marker_id in self.poses_dict:
                    pos = self.poses_dict[marker_id].pose.position
                    dist = np.sqrt(pos.x**2 + pos.y**2)
                    self.get_logger().info(f'  Marker {marker_id}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) dist={dist:.1f}m')
        else:
            self.get_logger().info('Marker trovati: 0/4')
            
        if len(self.found_ids) < 4:
            missing = set([1, 2, 3, 4]) - self.found_ids
            self.get_logger().info(f'Mancano ancora: {sorted(list(missing))}')
        
        # Diagnostica problemi
        if self.image_count == 0:
            self.get_logger().warn('PROBLEMA: Nessuna immagine ricevuta! Verifica topic camera.')
        elif self.camera_matrix is None:
            self.get_logger().warn('PROBLEMA: Camera info non ricevuta! Verifica topic camera_info.')
        elif self.detection_attempts > 100 and len(self.found_ids) == 0:
            self.get_logger().warn('PROBLEMA: Molti tentativi ma nessun marker rilevato! Verifica marker ArUco nella scena.')
        
        self.get_logger().info('===============================')

    def calculate_optimal_manipulation_poses(self, marker_id, original_pose):
        """
        SISTEMA AUTOMATICO: Calcola pose ottimali per manipolazione
        basate su dimensioni oggetto, geometria ArUco e distanza dal robot
        """
        if marker_id not in self.object_configs:
            # Marker sconosciuto, usa pose originale
            return {"manipulation": original_pose}
            
        config = self.object_configs[marker_id]
        pos = original_pose.pose.position
        
        # Calcola distanza dal robot per aggiustamenti dinamici
        distance_from_robot = np.sqrt(pos.x**2 + pos.y**2)
        
        # Parametri base per calcolo automatico
        aruco_to_object_center = self.aruco_size / 2  # 3cm dall'ArUco al centro oggetto
        safety_margin = 0.015  # 1.5cm margine di sicurezza
        
        # Calcolo offset ottimale basato su tipo oggetto e distanza
        if config["type"] == "bottle":
            # Coca-Cola: bottiglia stretta, serve più precisione
            optimal_approach = aruco_to_object_center + 0.055 + safety_margin  # ~7.5cm (+2cm)
            if distance_from_robot > 1.0:
                optimal_approach += 0.01  # +1cm se lontano
        elif config["type"] == "cylinder":
            # Pringles: cilindro più largo, più facile da afferrare
            optimal_approach = aruco_to_object_center + 0.045 + safety_margin  # ~6.5cm (+2cm)
            if distance_from_robot > 1.0:
                optimal_approach += 0.005  # +0.5cm se lontano
        else:
            # Destinazioni: usa distanza standard
            optimal_approach = aruco_to_object_center + 0.05 + safety_margin  # ~7cm (+2cm)
        
        # Crea nuova pose ottimizzata
        optimized_pose = PoseStamped()
        optimized_pose.header = original_pose.header
        
        # Calcola vettore di avvicinamento (verso il robot)
        if distance_from_robot > 0.001:
            approach_vector = np.array([-pos.x, -pos.y]) / distance_from_robot
            offset = approach_vector * optimal_approach
        else:
            offset = np.array([optimal_approach, 0.0])  # Fallback
        
        # Applica offset ottimizzato
        optimized_pose.pose.position.x = pos.x + offset[0]
        optimized_pose.pose.position.y = pos.y + offset[1] 
        optimized_pose.pose.position.z = pos.z  # Mantieni altezza originale
        optimized_pose.pose.orientation = original_pose.pose.orientation
        
        self.get_logger().info(f"AUTO-CALC Marker {marker_id} ({config['name']}):")
        self.get_logger().info(f"   Originale:   [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
        self.get_logger().info(f"   Ottimizzata: [{optimized_pose.pose.position.x:.3f}, {optimized_pose.pose.position.y:.3f}, {optimized_pose.pose.position.z:.3f}]")
        self.get_logger().info(f"   Offset calcolato: {optimal_approach:.3f}m per {config['type']}")
        
        return {"manipulation": optimized_pose, "original": original_pose}

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
