import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped,PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
import time
import threading

class TFListener(Node):

    """
    Il nodo pubblica la posa degli aruco rispetto al sistema di riferimento base_footprint a partire dal sistema di riferimento
    head_front_camera_optical_frame.
    Per ogni nuovo aruco osservato pubblica:
        -   una sola volta la media delle pose osservate nei 20 secondi successivi alla visione del primo aruco,
            in PoseTransformed, sul topic /aruco_[id]_pose_tranformed, creato alla prima osservazione dell'aruco con id=[id]
        -   una volta alla creazione, una volta alla pubblicazione su /aruco_[id]_pose_tranformed, l'id dell'aruco in Int32
    Se il nodo non riceve id di aruco dal topic /aruco_poses rimane in attesa.
    """

    def __init__(self):
        super().__init__('aruco_coord_transformation')

        #Crea un thread per il timer di 20 secondi; alla scadenza avverrà la pubblicazione
        self.timer_thread = threading.Thread(target=self.timer)

        self.timer_started=False


        #Subscription a /aruco_poses, dalla quale riceve l'id degli aruco inquadrati
        self.aruco_pose_subscriber=self.create_subscription(Int32,"aruco_poses",self.aruco_pose_found,10)
        #Publisher su cui pubblica, una volta alla creazione, una volta alla pubblicazione su /aruco_[id]_pose_tranformed, l'id dell'aruco
        self.aruco_coord_publisher=self.create_publisher(Int32,"aruco_poses_transformed",10)

        #Nella posizione i-esima dei seguenti array vengono dinamicamente inserite le seguenti informazioni degli aruco con
        #id pari a i (un elemento per ognuno dei 250 aruco 6x6):
        #Array contenete i subscriber delle pose sul topic /aruco_[i]_pose; viene creato alla prima osservazione dell'i-esimo
        #aruco su /aruco_poses
        self.aruco_pose_subscriber=[None] * 250
        #Array contenete i publisher delle pose trasformate e pubblicanti sul topic /aruco_[i]_pose_tranformed; viene creato alla
        #osservazione della prima posa su /aruco_[i]_pose
        self.aruco_already_published=[None] * 250


        #Implementa i PoseTransformed sui quali vengono sommate le posizioni trasformate e poi, prima dell'invio dividendo per
        #aruco_pose_counter, la media
        self.aruco_pose_mean=[None] * 250
        self.aruco_pose_counter=np.zeros(250)

        self.first_aruco=True       #Avvia il timer dopo la ricezione dell'id primo aruco e diventa False
        self.time_to_publish=False  #Quando il timer di 20 secondi scade, diventa True e avviene la pubblicazione delle pose trasformate

        #Listener per ascoltare le trasformazioni tra i frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.id=None




    def lookup_tf(self,msg):

        if self.timer_started is False:

            try:

                #Lookup della trasformazione tra head_front_camera_optical_frame e base_footprint
                transform = self.tf_buffer.lookup_transform('base_footprint', 'head_front_camera_optical_frame', rclpy.time.Time())

                #Posizione e rotazione (quaternione e matrice di rotazione) di head_front_camera_optical_frame rispetto a
                #base_footprint

                pos_head = np.array([[transform.transform.translation.x],
                                    [transform.transform.translation.y],
                                    [transform.transform.translation.z]])

                quat_head = np.array([transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w])

                rot_head = Rotation.from_quat(quat_head).as_matrix()

                #Ottiene i valori della posa non trasformata

                old_pose_traslation=np.zeros((3,1))
                old_pose_rotation=np.zeros(4)

                old_pose_traslation[0]=msg.pose.position.x
                old_pose_traslation[1]=msg.pose.position.y
                old_pose_traslation[2]=msg.pose.position.z

                old_pose_rotation[0]=msg.pose.orientation.x
                old_pose_rotation[1]=msg.pose.orientation.y
                old_pose_rotation[2]=msg.pose.orientation.z
                old_pose_rotation[3]=msg.pose.orientation.w

                old_pose_rotation_rotvet=Rotation.from_quat(old_pose_rotation).as_matrix()

                #Posizione e rotazione del marker rispetto a base_footprint

                new_pose_traslation=pos_head+np.dot(rot_head,old_pose_traslation)

                new_pose_rotation_rotvet=np.dot(rot_head,old_pose_rotation_rotvet)
                new_pose_rotation=Rotation.from_matrix(new_pose_rotation_rotvet).as_quat()

                #Se non ha ancora memorizzato dati per l'aruco con id=self.id esegue le azioni di cui sotto

                if self.aruco_pose_mean[self.id] is None:

                    #Pubblica l'id dell'aruco sul topic /aruco_poses_transformed
                    id_published=Int32()
                    id_published.data=self.id
                    self.aruco_coord_publisher.publish(id_published)

                    #Inizializza un messaggio TransformStamped
                    self.aruco_pose_mean[self.id]=PoseStamped()
                    self.aruco_pose_mean[self.id].header.stamp = self.get_clock().now().to_msg()     #Sincronizzazione
                    self.aruco_pose_mean[self.id].header.frame_id = "base_footprint"                 #Sistema di riferimento (frame) della posa trasformata

                    #Inizializza posizioni e rotazioni con i risultati sopra ottenuti e aggiorna il contatore

                    self.aruco_pose_mean[self.id].pose.position.x = new_pose_traslation[0][0]
                    self.aruco_pose_mean[self.id].pose.position.y = new_pose_traslation[1][0]
                    self.aruco_pose_mean[self.id].pose.position.z = new_pose_traslation[2][0]

                    self.aruco_pose_mean[self.id].pose.orientation.x=new_pose_rotation[0]
                    self.aruco_pose_mean[self.id].pose.orientation.y=new_pose_rotation[1]
                    self.aruco_pose_mean[self.id].pose.orientation.z=new_pose_rotation[2]
                    self.aruco_pose_mean[self.id].pose.orientation.w=new_pose_rotation[3]

                    self.aruco_pose_counter[self.id] += 1

                else:

                    #Altrimenti, somma ai campi del già esistente messaggio posizioni e rotazioni sopra ottenuti e
                    #aggiorna il contatore

                    self.aruco_pose_mean[self.id].pose.position.x += new_pose_traslation[0][0]
                    self.aruco_pose_mean[self.id].pose.position.y += new_pose_traslation[1][0]
                    self.aruco_pose_mean[self.id].pose.position.z += new_pose_traslation[2][0]

                    self.aruco_pose_mean[self.id].pose.orientation.x += new_pose_rotation[0]
                    self.aruco_pose_mean[self.id].pose.orientation.y += new_pose_rotation[1]
                    self.aruco_pose_mean[self.id].pose.orientation.z += new_pose_rotation[2]
                    self.aruco_pose_mean[self.id].pose.orientation.w += new_pose_rotation[3]

                    self.aruco_pose_counter[self.id] += 1

                #Quando self.time_to_publish è True sono trascorsi 20 secondi dalla visualizzazione del primo marker

                if self.time_to_publish is True:

                    #Imposta nuovamente self.time_to_publish a False per evitare invii multipli e, per ogni TransformStamped in
                    #self.aruco_pose_mean, esegue la media dei campi posizione e rotazione e pubblica il risultato

                    self.time_to_publish=False


                    for id_msg in range(0,249):
                        if self.aruco_pose_mean[id_msg] is not None:

                            #Media di posizione e rotazione

                            self.aruco_pose_mean[id_msg].pose.position.x /= self.aruco_pose_counter[id_msg]
                            self.aruco_pose_mean[id_msg].pose.position.y /= self.aruco_pose_counter[id_msg]
                            self.aruco_pose_mean[id_msg].pose.position.z /= self.aruco_pose_counter[id_msg]

                            self.aruco_pose_mean[id_msg].pose.orientation.x /= self.aruco_pose_counter[id_msg]
                            self.aruco_pose_mean[id_msg].pose.orientation.y /= self.aruco_pose_counter[id_msg]
                            self.aruco_pose_mean[id_msg].pose.orientation.z /= self.aruco_pose_counter[id_msg]
                            self.aruco_pose_mean[id_msg].pose.orientation.w /= self.aruco_pose_counter[id_msg]

                            #Pubblicazione della posa trasformata su /aruco_[id_msg]_pose_transformed e dell'id su
                            #/aruco_poses_transformed



                            self.aruco_already_published[id_msg].publish(self.aruco_pose_mean[id_msg])
                            id_published=Int32()
                            id_published.data=id_msg
                            self.aruco_coord_publisher.publish(id_published)

                            self.get_logger().info(f"Posa trasformata dell'aruco {id_msg} pubblicata")

                    self.timer_started=True
                    self.create_timer(1.0,self.timer_event)



            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().warn("Trasformazione non trovata!")

    def timer_event(self):

        if self.timer_started is True:

            for id_msg in range(0,249):

                if self.aruco_pose_mean[id_msg] is not None:

                    if self.aruco_already_published[id_msg] is not None:

                        self.aruco_already_published[id_msg].publish(self.aruco_pose_mean[id_msg])

    def aruco_pose_found(self, msg):

        #Quando entra in questa funzione di callback è stato individuato un aruco: il timer di 20 secondi viene avviato
        if self.first_aruco is True:
            self.get_logger().info("Le pose trasformate verranno pubblicate tra 20 secondi.")
            self.timer_thread.start()
            self.first_aruco=False

        #Prende dal campo data di msg (Int32) l'id dell'aruco individuato. Se self.aruco_already_published[self.id] è
        #impostato in None, è la prima volta che l'aruco con id=self.id viene visualizzato
        self.id=msg.data
        if self.aruco_already_published[self.id] is None:

            #Crea un publisher sul topic /aruco_[self.id]_pose_transformed per la posa trasformata del marker e un subscriber a
            #/aruco_[self.id]_pose per ricavare la posa rispetto a head_front_camera_optical_frame
            self.aruco_already_published[self.id]=self.create_publisher(PoseStamped, "aruco_"+str(self.id)+"_pose_transformed", 10)
            self.aruco_pose_subscriber[self.id]=self.create_subscription(PoseStamped,"aruco_"+str(self.id)+"_pose",self.lookup_tf,10)

            self.get_logger().info(f"Il topic /aruco_{self.id}_pose_transformed, per la visualizzazione della posa traformata di {self.id} è ora disponibile!")

    def timer(self):
        #Timer di 20 secondi: allo scadere la variabile self.time_to_publish diventa True e le pose verranno pubblicate
        time.sleep(20)
        self.time_to_publish=True



def main():

    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

