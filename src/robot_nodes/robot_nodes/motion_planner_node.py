import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from geometry_msgs.msg import PoseStamped
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from spatialmath import SE3
import time


class GazeboTrajectoryPublisher(Node):

    """
    Il nodo GazeboTrajectoryPublisher implementa l'inversione cinematica per il raggiungimento della posa finale elaborata dal
    nodo StateMachine, per la movimentazione del braccio tel robot TIAGo.
    """

    def __init__(self):

        super().__init__('gazebo_trajectory_publisher')

        # Carica il modello del robot da urdf
        urdf_loc = '/home/claudio/Desktop/progetto_ros2/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)

        # Crea un oggetto vuoto PoseStamped per la posa finale da raggiungere
        self.final=PoseStamped()

        # Indici utili all'inversione cinematica
        self.q_traj = None
        self.index = 0

        # Inizializza la posa corrente di braccio e torso a quella di relax
        self.current_arm_pose=[0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
        self.current_torso_pose=[0.35]

        # Crea un publisher e un subscriber per il topic di comunicazione con StateMachine
        self.sm_ik_publisher=self.create_publisher(String,"sm_ik_communication",10)
        self.sm_ik_listener=self.create_subscription(String,"sm_ik_communication",self.next_task_callback,10)

        # Crea un subscriber per il topic delle pose finali comunicate da StateMachine
        self.final_pose=self.create_subscription(PoseStamped,"final_pose",self.final_pose_found,10)

        # Crea un publisher e un subscriber per registrare e pubblicare la posa di braccio e torso
        self.subscriptiona_ = self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.new_arm_pose, 10)
        self.subscriptiont_ = self.create_subscription(JointTrajectory, '/torso_controller/joint_trajectory', self.new_torso_pose, 10)
        self.publishera_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.publishert_ = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)


    # Quando riceve una posa da raggiungere dalla StateMachine, la salva

    def final_pose_found(self,msg):
        self.final=msg

    # Memorizza la posa corrente di braccio e torso, rispettivamente

    def new_arm_pose(self,msg):
       self.current_arm_pose=list(msg.points[0].positions)

    def new_torso_pose(self,msg):
        self.current_torso_pose=list(msg.points[0].positions)

    # Quando la StateMachine invia un nuovo task da compiere, viene eseguita next_task_callback, che implementa l'inversione
    # cinematica

    def next_task_callback(self,msg):

        # Divide il task corrente in corrispondenza dello specificatore per il numero di pose intermedie
        actual_msg_lenght=msg.data.split('_num:')

        # A meno che il task non sia gripper o torso, esegue l'inversione
        if len(actual_msg_lenght)>1 or actual_msg_lenght[0]=='relax':

            # La posa di partenza è data dalle pose correnti di troso e braccio
            q0 = np.array(self.current_torso_pose+self.current_arm_pose)
            T0 = SE3(self.robot.fkine(q0))

            # Se il task non è di relax elabora il messaggio PoseStamped di posa finale
            if msg.data!='relax':

                pos_arm_final = np.array([[self.final.pose.position.x],
                        [self.final.pose.position.y],
                        [self.final.pose.position.z]])

                quat_arm_final = np.array([self.final.pose.orientation.x,
                    self.final.pose.orientation.y,
                    self.final.pose.orientation.z,
                    self.final.pose.orientation.w])

                rot_arm_final=Rotation.from_quat(quat_arm_final).as_matrix()

                # Calcola la posa da raggiungere
                qf=np.vstack((np.hstack((rot_arm_final,pos_arm_final)),[0,0,0,1]))
                Tf = SE3(qf)

                # Specifica il numero di pose intermedie
                N = int(actual_msg_lenght[1])

            else:

                # Calcola la posa da raggiungere in caso di relax
                qf = [0.35, 0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
                Tf = SE3(self.robot.fkine(qf))

                # Preimposta 50 posizioni intermedie
                N=50

            # INVERSIONE CINEMATICA

            # 1. Traiettoria cartesiana tra le due pose (SE3)
            Ts = rtb.ctraj(T0, Tf, N)  # genera N pose interpolate

            # 2. Inverse kinematics per ciascun frame
            q_traj = []
            q_curr = q0
            for T in Ts:
                sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)

                if sol is not None and len(sol) > 0:
                    q_curr = sol[0]
                q_traj.append(q_curr)

            # Traiiettoria da seguire a partire dall'elemento self.index=0
            self.q_traj = np.array(q_traj)
            self.index=0

            #Crea un timer per l'esecuzione dei singoli movimenti
            self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def timer_callback(self):

        # Se i movimenti intermedi sono terminati, distrugge il timer, attende 1 secondo la fine del movimento e invia un
        # messaggio di termine task su sm_ik_communication, per il coordinamento con la StateMachine. Esce poi dalla funzione
        if self.index >= len(self.q_traj):

            self.destroy_timer(self.timer)

            time.sleep(1)

            end_msg=String()
            end_msg.data='terminato'
            self.sm_ik_publisher.publish(end_msg)

            return


        # Pubblica le pose di torso e braccio, rispettivamente, per l'elemento self.index-esimo di self.q_traj
        q = np.array(self.q_traj[self.index]).astype(float)

        msgt = JointTrajectory()
        msgt.joint_names = ['torso_lift_joint']
        torso_point = JointTrajectoryPoint()
        torso_point.positions = [float(q[0])]
        torso_point.time_from_start.sec = 1

        msgt.points.append(torso_point)
        self.publishert_.publish(msgt)

        msga = JointTrajectory()
        msga.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(qi) for qi in q[1:]]
        arm_point.time_from_start.sec = 1

        msga.points.append(arm_point)
        self.publishera_.publish(msga)

        # Aggiorna l'indice per il l'elemento di self.q_traj
        self.index += 1



def main():
    rclpy.init()
    node = GazeboTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

