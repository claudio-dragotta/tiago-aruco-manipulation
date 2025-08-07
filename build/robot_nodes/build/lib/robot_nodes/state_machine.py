from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class State(Enum):
    WAITING_FOR_ARUCO = 1
    MOVE_TO_POSE_1 = 2
    MOVE_TO_POSE_2 = 3
    GRIP_OBJECT = 4
    MOVE_UP_COLA = 5
    MOVE_COLA_TO_FINISH = 6
    RELEASE_OBJECT = 7
    MOVE_TO_HOME = 8
    MOVE_TO_POSE_3 = 9
    MOVE_TO_POSE_4 = 10
    GRIP_OBJECT_2 = 11
    MOVE_UP_PRINGLES = 12
    MOVE_PRINGLES_TO_FINISH = 13
    RELEASE_OBJECT_2 = 14
    MOVE_TO_HOME_2 = 15
    COMPLETED = 16

class RobotStateMachineNode(Node):
    def __init__(self):


        super().__init__('robot_state_machine_node')
        self.get_logger().info("Avvio Robot State Machine...")
        self.stato_corrente = State.WAITING_FOR_ARUCO
        self.ultimo_stato = State.WAITING_FOR_ARUCO
        self.current_target = 1

        self.publisher = self.create_publisher(Int32, '/command_topic', 10)
        self.create_subscription(Int32,'/completed_command_topic', self.completed_command_callback, 10)
        self.create_subscription(Bool, '/all_markers_found', self.all_markers_callback, 10)
        
        self.command_in_progress = False
        self.retry_count = 0
        self.max_retries = 3
        self.timer = self.create_timer(2.0, self.state_manager)

    def all_markers_callback(self, msg: Bool):
        # Quando riceve True, muove la macchina allo stato MOVE_TO_POSE_1
        if msg.data and self.stato_corrente == State.WAITING_FOR_ARUCO:
            self.get_logger().info("Tutti i marker trovati: avvio lo stato MOVE_TO_POSE_1")
            self.stato_corrente = State.MOVE_TO_POSE_1

    def state_manager(self):
        if self.stato_corrente == State.COMPLETED:
            self.get_logger().info("Sistema completato con successo!")
            return
            
        if self.command_in_progress:
            return
            
        if self.stato_corrente != self.ultimo_stato:
            self.get_logger().info(f"⚡ Transizione: {self.ultimo_stato.name} → {self.stato_corrente.name}")
            self.execute_current_state()
            self.ultimo_stato = self.stato_corrente
            
    def execute_current_state(self):
        state_actions = {
            State.WAITING_FOR_ARUCO: self.start_aruco_search,
            State.MOVE_TO_POSE_1: self.move_to_pose_1,
            State.MOVE_TO_POSE_2: self.move_to_pose_2,
            State.GRIP_OBJECT: self.grip_object,
            State.MOVE_UP_COLA: self.move_up_cola,
            State.MOVE_COLA_TO_FINISH: self.move_cola_to_finish,
            State.RELEASE_OBJECT: self.release_object,
            State.MOVE_TO_HOME: self.move_to_home,
            State.MOVE_TO_POSE_3: self.move_to_pose_3,
            State.MOVE_TO_POSE_4: self.move_to_pose_4,
            State.GRIP_OBJECT_2: self.grip_object_2,
            State.MOVE_UP_PRINGLES: self.move_up_pringles,
            State.MOVE_PRINGLES_TO_FINISH: self.move_pringles_to_finish,
            State.RELEASE_OBJECT_2: self.release_object_2,
            State.MOVE_TO_HOME_2: self.move_to_home_2
        }
        
        action = state_actions.get(self.stato_corrente)
        if action:
            self.command_in_progress = True
            action()
        else:
            self.get_logger().warn(f"Stato {self.stato_corrente} non gestito")

    def completed_command_callback(self, msg):
        self.get_logger().info(f"✅ Completato: {State(msg.data).name}")
        self.command_in_progress = False
        self.retry_count = 0
        
        state_transitions = {
            State.MOVE_TO_POSE_1.value: State.MOVE_TO_POSE_2,
            State.MOVE_TO_POSE_2.value: State.GRIP_OBJECT,
            State.GRIP_OBJECT.value: State.MOVE_UP_COLA,
            State.MOVE_UP_COLA.value: State.MOVE_COLA_TO_FINISH,
            State.MOVE_COLA_TO_FINISH.value: State.RELEASE_OBJECT,
            State.RELEASE_OBJECT.value: State.MOVE_TO_HOME,
            State.MOVE_TO_HOME.value: State.MOVE_TO_POSE_3,
            State.MOVE_TO_POSE_3.value: State.MOVE_TO_POSE_4,
            State.MOVE_TO_POSE_4.value: State.GRIP_OBJECT_2,
            State.GRIP_OBJECT_2.value: State.MOVE_UP_PRINGLES,
            State.MOVE_UP_PRINGLES.value: State.MOVE_PRINGLES_TO_FINISH,
            State.MOVE_PRINGLES_TO_FINISH.value: State.RELEASE_OBJECT_2,
            State.RELEASE_OBJECT_2.value: State.MOVE_TO_HOME_2,
            State.MOVE_TO_HOME_2.value: State.COMPLETED
        }
        
        next_state = state_transitions.get(msg.data)
        if next_state:
            self.stato_corrente = next_state
        else:
            self.get_logger().error(f"Transizione non definita per stato {msg.data}")

    def move_to_pose_1(self):
        self.get_logger().info("🎯 Coca-Cola: Movimento verso posa 1")
        self.publish_command(State.MOVE_TO_POSE_1)
        
    def move_to_pose_2(self):
        self.get_logger().info("🎯 Coca-Cola: Avvicinamento per presa")
        self.publish_command(State.MOVE_TO_POSE_2)

    def grip_object(self):
        self.get_logger().info("🤏 Coca-Cola: Chiusura gripper")
        self.publish_command(State.GRIP_OBJECT)

    def move_up_cola(self):
        self.get_logger().info("⬆️  Coca-Cola: Sollevamento")
        self.publish_command(State.MOVE_UP_COLA)

    def move_cola_to_finish(self):
        self.get_logger().info("🎯 Coca-Cola: Trasporto a destinazione")
        self.publish_command(State.MOVE_COLA_TO_FINISH)

    def release_object(self):
        self.get_logger().info("🤲 Coca-Cola: Rilascio")
        self.publish_command(State.RELEASE_OBJECT)

    def move_to_home(self):
        self.get_logger().info("🏠 Ritorno alla posizione iniziale")
        self.publish_command(State.MOVE_TO_HOME)

    def grip_object_2(self):
        self.get_logger().info("🤏 Pringles: Chiusura gripper")
        self.publish_command(State.GRIP_OBJECT_2)

    def release_object_2(self):
        self.get_logger().info("🤲 Pringles: Rilascio")
        self.publish_command(State.RELEASE_OBJECT_2)

    def move_to_pose_3(self):
        self.get_logger().info("🎯 Pringles: Movimento verso posa 3")
        self.publish_command(State.MOVE_TO_POSE_3)

    def move_to_pose_4(self):
        self.get_logger().info("🎯 Pringles: Avvicinamento per presa")
        self.publish_command(State.MOVE_TO_POSE_4)

    def move_up_pringles(self):
        self.get_logger().info("⬆️  Pringles: Sollevamento")
        self.publish_command(State.MOVE_UP_PRINGLES)

    def move_pringles_to_finish(self):
        self.get_logger().info("🎯 Pringles: Trasporto a destinazione")
        self.publish_command(State.MOVE_PRINGLES_TO_FINISH)

    def move_to_home_2(self):
        self.get_logger().info("🏠 Ritorno finale alla posizione iniziale")
        self.publish_command(State.MOVE_TO_HOME_2)
        
    def start_aruco_search(self):
        self.get_logger().info("🔍 Avvio ricerca marker ArUco...")
        # Non pubblica un comando specifico, ma il sistema aspetta il callback
        self.command_in_progress = False  # Non è un comando vero, solo attesa
        
    def publish_command(self, state):
        msg = Int32()
        msg.data = state.value
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()