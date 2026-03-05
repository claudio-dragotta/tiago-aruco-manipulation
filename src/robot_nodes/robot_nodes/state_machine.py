from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class State(Enum):
    WAITING_FOR_ARUCO = 1
    INTERMEDIATE_CONFIG = 2
    OPERATIONAL_CONFIG = 3
    # Oggetto 1 (Pringles): marker 1 -> marker 3 (PRIMA)
    MOVE_TO_OBJECT_1 = 4
    GRIP_OBJECT_1 = 5
    LIFT_OBJECT_1 = 6
    MOVE_TO_DEST_1 = 7
    RELEASE_OBJECT_1 = 8
    RETURN_HOME_1 = 9
    # Oggetto 2 (Coca-Cola): marker 2 -> marker 4 (SECONDA)  
    MOVE_TO_OBJECT_2 = 10
    GRIP_OBJECT_2 = 11
    LIFT_OBJECT_2 = 12
    MOVE_TO_DEST_2 = 13
    RELEASE_OBJECT_2 = 14
    RETURN_HOME_2 = 15
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
        # Quando riceve True (tutti e 4 marker), inizia con configurazione intermedia
        if msg.data and self.stato_corrente == State.WAITING_FOR_ARUCO:
            self.get_logger().info("Tutti e 4 i marker ArUco trovati! Avvio configurazione intermedia...")
            self.stato_corrente = State.INTERMEDIATE_CONFIG

    def state_manager(self):
        if self.stato_corrente == State.COMPLETED:
            if not hasattr(self, 'completed_logged'):
                self.get_logger().info("Sistema completato con successo!")
                self.completed_logged = True
                # Distruggi il timer per evitare ulteriori chiamate
                if hasattr(self, 'timer'):
                    self.timer.cancel()
                    self.destroy_timer(self.timer)
            return

        if self.command_in_progress:
            return
            
        if self.stato_corrente != self.ultimo_stato:
            self.get_logger().info(f"Transizione: {self.ultimo_stato.name} -> {self.stato_corrente.name}")
            self.execute_current_state()
            self.ultimo_stato = self.stato_corrente
        else:
            # Debug: log solo ogni 10 cicli per evitare spam quando aspetta
            if hasattr(self, 'debug_counter'):
                self.debug_counter += 1
            else:
                self.debug_counter = 1
                
            if self.debug_counter % 10 == 0:
                self.get_logger().info(f"In attesa - Stato corrente: {self.stato_corrente.name}, Command in progress: {self.command_in_progress}")
            
    def execute_current_state(self):
        state_actions = {
            State.WAITING_FOR_ARUCO: self.start_aruco_search,
            State.INTERMEDIATE_CONFIG: self.intermediate_config,
            State.OPERATIONAL_CONFIG: self.operational_config,
            # Oggetto 1 (Coca-Cola)
            State.MOVE_TO_OBJECT_1: self.move_to_object_1,
            State.GRIP_OBJECT_1: self.grip_object_1,
            State.LIFT_OBJECT_1: self.lift_object_1,
            State.MOVE_TO_DEST_1: self.move_to_dest_1,
            State.RELEASE_OBJECT_1: self.release_object_1,
            State.RETURN_HOME_1: self.return_home_1,
            # Oggetto 2 (Pringles)
            State.MOVE_TO_OBJECT_2: self.move_to_object_2,
            State.GRIP_OBJECT_2: self.grip_object_2,
            State.LIFT_OBJECT_2: self.lift_object_2,
            State.MOVE_TO_DEST_2: self.move_to_dest_2,
            State.RELEASE_OBJECT_2: self.release_object_2,
            State.RETURN_HOME_2: self.return_home_2
        }
        
        action = state_actions.get(self.stato_corrente)
        if action:
            self.command_in_progress = True
            action()
        else:
            self.get_logger().warn(f"Stato {self.stato_corrente} non gestito")

    def completed_command_callback(self, msg):
        # Ignora completamenti se non stiamo aspettando nessun comando
        if not self.command_in_progress:
            self.get_logger().info(f"Completamento ignorato (non in progress): {msg.data}")
            return

        # Ignora completamenti che non corrispondono allo stato corrente
        if msg.data != self.stato_corrente.value:
            try:
                nome = State(msg.data).name
            except ValueError:
                nome = f"SCONOSCIUTO({msg.data})"
            self.get_logger().warn(f"Completamento scartato: ricevuto {msg.data} ({nome}), atteso {self.stato_corrente.value} ({self.stato_corrente.name})")
            return

        self.get_logger().info(f"Completato: {State(msg.data).name}")
        self.command_in_progress = False
        self.retry_count = 0
        
        state_transitions = {
            State.INTERMEDIATE_CONFIG.value: State.OPERATIONAL_CONFIG,
            State.OPERATIONAL_CONFIG.value: State.MOVE_TO_OBJECT_1,
            # Sequenza Oggetto 1 (Pringles): MARKER 1 -> MARKER 3 (PRIMA)
            State.MOVE_TO_OBJECT_1.value: State.GRIP_OBJECT_1,
            State.GRIP_OBJECT_1.value: State.LIFT_OBJECT_1,
            State.LIFT_OBJECT_1.value: State.MOVE_TO_DEST_1,
            State.MOVE_TO_DEST_1.value: State.RELEASE_OBJECT_1,
            State.RELEASE_OBJECT_1.value: State.RETURN_HOME_1,
            State.RETURN_HOME_1.value: State.MOVE_TO_OBJECT_2,
            # Sequenza Oggetto 2 (Coca-Cola): MARKER 2 -> MARKER 4 (SECONDA)
            State.MOVE_TO_OBJECT_2.value: State.GRIP_OBJECT_2,
            State.GRIP_OBJECT_2.value: State.LIFT_OBJECT_2,
            State.LIFT_OBJECT_2.value: State.MOVE_TO_DEST_2,
            State.MOVE_TO_DEST_2.value: State.RELEASE_OBJECT_2,
            State.RELEASE_OBJECT_2.value: State.RETURN_HOME_2,
            State.RETURN_HOME_2.value: State.COMPLETED
        }
        
        next_state = state_transitions.get(msg.data)
        if next_state:
            self.stato_corrente = next_state
        else:
            self.get_logger().error(f"Transizione non definita per stato {msg.data}")

    # === OGGETTO 1 (PRINGLES): MARKER 1 -> MARKER 3 (PRIMA) ===
    def move_to_object_1(self):
        self.get_logger().info("Pringles: Movimento verso oggetto 1 (ArUco marker 1) - PRIMA")
        self.publish_command(State.MOVE_TO_OBJECT_1)
        
    def grip_object_1(self):
        self.get_logger().info("Pringles: Chiusura gripper per prendere oggetto")
        self.publish_command(State.GRIP_OBJECT_1)

    def lift_object_1(self):
        self.get_logger().info("Pringles: Sollevamento oggetto")
        self.publish_command(State.LIFT_OBJECT_1)

    def move_to_dest_1(self):
        self.get_logger().info("Pringles: Trasporto a destinazione (ArUco marker 3)")
        self.publish_command(State.MOVE_TO_DEST_1)

    def release_object_1(self):
        self.get_logger().info("Pringles: Rilascio oggetto")
        self.publish_command(State.RELEASE_OBJECT_1)

    def return_home_1(self):
        self.get_logger().info("Ritorno finale alla posizione iniziale")
        self.publish_command(State.RETURN_HOME_1)

    # === OGGETTO 2 (COCA-COLA): MARKER 2 -> MARKER 4 (SECONDA) ===
    def move_to_object_2(self):
        self.get_logger().info("Coca-Cola: Movimento verso oggetto 2 (ArUco marker 2) - SECONDA")
        self.publish_command(State.MOVE_TO_OBJECT_2)

    def grip_object_2(self):
        self.get_logger().info("Coca-Cola: Chiusura gripper per prendere oggetto")
        self.publish_command(State.GRIP_OBJECT_2)

    def lift_object_2(self):
        self.get_logger().info("Coca-Cola: Sollevamento oggetto")
        self.publish_command(State.LIFT_OBJECT_2)

    def move_to_dest_2(self):
        self.get_logger().info("Coca-Cola: Trasporto a destinazione (ArUco marker 4)")
        self.publish_command(State.MOVE_TO_DEST_2)

    def release_object_2(self):
        self.get_logger().info("Coca-Cola: Rilascio oggetto")
        self.publish_command(State.RELEASE_OBJECT_2)

    def return_home_2(self):
        self.get_logger().info("Ritorno finale alla posizione iniziale")
        self.publish_command(State.RETURN_HOME_2)
        
    def start_aruco_search(self):
        self.get_logger().info("Avvio ricerca marker ArUco...")
        # Non pubblica un comando specifico, ma il sistema aspetta il callback
        self.command_in_progress = False  # Non è un comando vero, solo attesa

    def intermediate_config(self):
        self.get_logger().info("Invio configurazione intermedia del braccio...")
        self.publish_command(State.INTERMEDIATE_CONFIG)

    def operational_config(self):
        self.get_logger().info("Invio configurazione operativa (torso + braccio + gripper)...")
        self.publish_command(State.OPERATIONAL_CONFIG)
        
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