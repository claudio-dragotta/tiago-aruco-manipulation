import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from std_msgs.msg import Int32

class ArUcoPosePublisher(Node):

    """
    Il nodo pubblica la posa degli aruco rispetto al sistema di riferimento solidale alla fotocamera (head_front_camera_optical_frame),
    mostrandli a video dalla prospettiva di TiaGo.
    Per ogni aruco, finchè inquadrato dalla fotocamera, pubblica:
        -   il suo id in Int32, sul topic /aruco_poses
        -   la sua posa in PoseStamped, sul topic /aruco_[id]_pose, creato dalla prima osservazione dell'aruco con id=[id]
    Se la fotocamera non inquadra aruco, il nodo rimane in attesa.
    """

    def __init__(self):
        super().__init__("aruco_scan_publisher")

        #Publisher per gli id degli aruco marker osservati dalla fotocamera in tempo reale
        self.aruco_pose_publisher=self.create_publisher(Int32,"aruco_poses",10)
        self.timer = self.create_timer(1.0, self.publish_aruco_pose)

        #Subscription alla fotocamera di TiaGo
        self.image=self.create_subscription(Image, "/head_front_camera/rgb/image_raw", self.callback_function, 10)
        #Subscription alle info della fotocamera di TiaGo
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)

        self.bridge=CvBridge()      #Bridge per permettere l'analisi dell'immagine a ros2
        self.last_image=None        #Ultimo frame osservato
        self.camera_matrix=None     #Info della fotocamera
        self.dist_coeffs=None       #Coefficienti di distorsione della fotocamera

        #Array di 250 elementi (uno per ognuno dei 250 aruco 6x6) contenete un oggetto publisher per la posa dell'aruco
        #con id=i nella posizione i-esima; gli oggetti vengono creati e inseriti quando l'aruco con id=i viene rilevato
        #per la prima volta
        self.aruco_publisher_list=[None] * 250


    def publish_aruco_pose(self):

        #Definire dizionario e parametri
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()


        ids=None

        if self.last_image is not None:
            #Restituisce la lista dei contorni dei marker, i loro id e i contorni dei marker non riconosciuti
            corners, ids, rejected = cv2.aruco.detectMarkers(self.last_image, aruco_dict, parameters=parameters)
            #Mostra l'immagine (avviarla ora, evita rallentamenti al momento della visualizzazione del primo aruco)
            cv2.imshow('Detected Marker with Pose', self.last_image)
            cv2.waitKey(1)

        #Se non sono stati individuati id (ids=None) publish_aruco_pose non fa niente

        if ids is not None:

            for i in range(0,len(ids)): #Esegue le azioni per ogni i-esimo elemento contenuti in ids

                #Pubblica l'id dell'aruco sul topic /aruco_poses
                id=int(ids[i])
                id_published=Int32()
                id_published.data=id
                self.aruco_pose_publisher.publish(id_published)

                #Se vede un aruco con uno specifico id per la prima volta, crea un publisher per la sua posa
                #sul topic /aruco_[id]_pose
                if self.aruco_publisher_list[id] is None:
                    self.aruco_publisher_list[id]=self.create_publisher(PoseStamped, "aruco_"+str(id)+"_pose", 10)
                    self.get_logger().info(f"Aruco {id} identificato; posa disponibile sul topic /aruco_{id}_pose")


                #Stimare la posa del marker --> rvecs e' un vettore di rotazione in formato Rodrigues, tvecs rappresenta la traslazione
                marker_length = 0.06    #Dimensione del marker, in metri
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners[i]], marker_length, self.camera_matrix, self.dist_coeffs)


                #Crea un messaggio PoseStamped per la posa dell'aruco
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()         #Sincronizzazione
                pose_msg.header.frame_id = "head_front_camera_optical_frame"    #Sistema di riferimento (frame) della posa

                #Posizione del marker
                pose_msg.pose.position.x = tvecs[0][0][0]
                pose_msg.pose.position.y = tvecs[0][0][1]
                pose_msg.pose.position.z = tvecs[0][0][2]

                #Orientamento del marker (quaternion da vettori di rotazione)
                rotation_matrix, _ = cv2.Rodrigues(rvecs[0])    # vettore di rotazione rvec in matrice di rotazione
                rot = Rotation.from_matrix(rotation_matrix)     # matrice --> oggetto Rotation
                quat = rot.as_quat()                            # estrai quaternion [x, y, z, w]

                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]


                #Pubblica il messaggio con la posa corrente
                self.aruco_publisher_list[id].publish(pose_msg)


                #Mostrare l'immagine con marker e assi
                cv2.aruco.drawDetectedMarkers(self.last_image, corners, ids)
                cv2.drawFrameAxes(self.last_image, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], marker_length / 2) # disegna un sistema di assi 3D (X, Y, Z) sopra il marker, basato sulla posa stimata

                #Mostra l'immagine
                cv2.imshow('Detected Marker with Pose', self.last_image)
                cv2.waitKey(1)  # Mostra la finestra per breve tempo




    def callback_function(self, msg):
        #Se riceve un immagine, la converte nel formato adatto a ros2
        #NOTA:  le immagini vengono elaborate a colori in quanto si rilevano difficoltà nel riconoscimento dei marker con
        #       l'utilizzo del bianco e nero
        if msg is not None:
            self.last_image=self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        #Quando riceve le info della fotocamera, modifica la matrice della fotocamera e i coefficienti di distorsione
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)



def main(args=None):
    rclpy.init(args=args)
    node=ArUcoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

