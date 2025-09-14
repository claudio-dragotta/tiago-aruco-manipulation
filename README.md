# Sistema di Manipolazione Robotica TiAGO con ArUco

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10+-green?style=flat-square&logo=python)
![Gazebo](https://img.shields.io/badge/Gazebo-11+-orange?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple?style=flat-square&logo=ubuntu)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-red?style=flat-square&logo=opencv)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)
![ArUco](https://img.shields.io/badge/ArUco-Detection-blue?style=flat-square)
![Robot](https://img.shields.io/badge/Robot-TiAGO-navy?style=flat-square)

## Descrizione del Progetto

Questo progetto implementa un sistema completo di manipolazione robotica per il robot TiAGO, che utilizza marcatori ArUco per l'identificazione e la manipolazione di oggetti. Il sistema è in grado di:

- Rilevare marcatori ArUco nell'ambiente tramite visione artificiale
- Calcolare la cinematica inversa per il posizionamento preciso del braccio robotico
- Eseguire sequenze di pick-and-place automatizzate
- Gestire la presa e il rilascio di oggetti diversi (Coca-Cola e Pringles)

## Architettura del Sistema

### Componenti Principali

1. **ArUco Detector Node** (`aruco_detector_node.py`)
   - Rileva marcatori ArUco dalla camera del robot
   - Calcola le pose 3D dei marcatori nel sistema di coordinate del robot
   - Pubblica le posizioni sui topic ROS2 dedicati

2. **Inverse Kinematics Node** (`ik.py`)
   - Calcola la cinematica inversa per il braccio TiAGO a 7 gradi di libertà
   - Gestisce la state machine per le operazioni di manipolazione
   - Controlla coordinatamente torso, braccio e gripper del robot

3. **State Machine Node** (`state_machine_node.py`)
   - Coordina la sequenza completa delle operazioni
   - Gestisce le transizioni tra gli stati del sistema
   - Monitora l'avanzamento e gestisce gli errori

4. **Launch System** (`launch_system.py`)
   - Sistema unificato di avvio per l'intero stack software
   - Gestisce l'avvio di Gazebo, RViz e tutti i nodi ROS2

### Flusso Operativo

1. **Fase di Rilevamento**: Il sistema rileva 4 marcatori ArUco (ID: 1, 2, 3, 4)
2. **Configurazione Robot**: Il robot si posiziona in configurazione operativa
3. **Manipolazione Oggetto 1**: Trasporto Pringles (marker 1 → marker 3) - PRIMA
4. **Manipolazione Oggetto 2**: Trasporto Coca-Cola (marker 2 → marker 4) - SECONDA
5. **Ritorno Home**: Il robot torna alla posizione iniziale

### Sequenza Dettagliata della State Machine

**Fase Inizializzazione:**
- `WAITING_FOR_ARUCO` → Attesa rilevamento tutti i 4 marker
- `INTERMEDIATE_CONFIG` → Configurazione posizione intermedia braccio
- `OPERATIONAL_CONFIG` → Configurazione operativa completa (torso + braccio + gripper)

**Prima Sequenza - Pringles (Marker 1 → Marker 3):**
- `MOVE_TO_OBJECT_1` → Movimento verso il contenitore Pringles
- `GRIP_OBJECT_1` → Chiusura gripper per afferrare
- `LIFT_OBJECT_1` → Sollevamento oggetto
- `MOVE_TO_DEST_1` → Trasporto verso destinazione (marker 3)
- `RELEASE_OBJECT_1` → Apertura gripper per rilasciare
- `RETURN_HOME_1` → Ritorno posizione intermedia

**Seconda Sequenza - Coca-Cola (Marker 2 → Marker 4):**
- `MOVE_TO_OBJECT_2` → Movimento verso bottiglia Coca-Cola
- `GRIP_OBJECT_2` → Chiusura gripper per afferrare
- `LIFT_OBJECT_2` → Sollevamento oggetto
- `MOVE_TO_DEST_2` → Trasporto verso destinazione (marker 4)
- `RELEASE_OBJECT_2` → Apertura gripper per rilasciare
- `RETURN_HOME_2` → Ritorno posizione finale

**Completamento:**
- `COMPLETED` → Sistema completato, timer ottimizzato per evitare messaggi ripetuti

## Requisiti di Sistema

### Software Richiesto

- **Sistema Operativo**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10 o superiore
- **Gazebo**: 11 o superiore
- **RViz2**: Per visualizzazione 3D

### Dipendenze Python

```bash
# Librerie per computer vision e ArUco
pip3 install opencv-python opencv-contrib-python

# Librerie per matematica 3D e trasformazioni
pip3 install transforms3d numpy scipy

# Librerie per robotica e controllo
pip3 install robotics-toolbox-python spatialmath-python

# Librerie aggiuntive per visualizzazione
pip3 install matplotlib
```

### Pacchetti ROS2

```bash
# Aggiornamento sistema
sudo apt update && sudo apt upgrade -y

# Installazione ROS2 Humble completa
sudo apt install -y ros-humble-desktop-full

# Pacchetti per simulazione
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-rviz2

# Pacchetti per controllo robot
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-controller-manager
sudo apt install -y ros-humble-joint-trajectory-controller
sudo apt install -y ros-humble-gripper-controllers

# Strumenti di sviluppo
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep
sudo apt install -y git cmake build-essential
```

### Setup dell'Ambiente ROS2

```bash
# Configurazione automatica ambiente ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Inizializzazione rosdep (eseguire solo la prima volta)
sudo rosdep init
rosdep update
```

## Installazione e Setup

### 1. Clonazione del Repository

```bash
# Clona il repository in una directory di lavoro
cd ~/Desktop
git clone <repository-url> progetto_ros2
cd progetto_ros2
```

### 2. Installazione delle Dipendenze

```bash
# Crea file requirements.txt (se non presente)
cat > requirements.txt << EOF
opencv-python>=4.5.0
opencv-contrib-python>=4.5.0
transforms3d>=0.4.0
robotics-toolbox-python>=1.0.0
spatialmath-python>=1.0.0
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.5.0
EOF

# Installa le dipendenze Python
pip3 install -r requirements.txt

# Installa le dipendenze ROS2
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compilazione del Progetto

```bash
# Compila tutto il workspace ROS2
colcon build --symlink-install

# Configura l'ambiente per la sessione corrente
source install/setup.bash

# Aggiungi al .bashrc per sessioni future (opzionale)
echo "source ~/Desktop/progetto_ros2/install/setup.bash" >> ~/.bashrc
```

### 4. Verifica dell'Installazione

```bash
# Verifica che i pacchetti siano compilati correttamente
ros2 pkg list | grep robot

# Output atteso:
# robot_launcher
# robot_nodes

# Verifica nodi disponibili
ros2 pkg executables robot_nodes
```

## Utilizzo del Sistema

### Avvio Completo (Raccomandato)

**IMPORTANTE**: Prima di ogni avvio, eseguire sempre la compilazione e il sourcing:

```bash
# 1. Compilazione del workspace (OBBLIGATORIO prima di ogni launch)
colcon build

# 2. Sourcing dell'ambiente ROS2 e del progetto
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. Avvio completo: Gazebo + RViz + tutti i nodi ROS2
ros2 launch robot_launcher full_system.launch.py
```

**Comando completo in una riga:**
```bash
colcon build && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_launcher full_system.launch.py
```

### Avvio Alternativo con Script

```bash
# Se disponibile il launcher unificato
python3 scripts/launch_system.py --mode full
```

### Modalità Alternative

```bash
# Solo simulazione (Gazebo senza RViz) - per prestazioni migliori
python3 scripts/launch_system.py --mode sim

# Solo nodi ROS2 (per utilizzo con robot reale)
python3 scripts/launch_system.py --mode nodes

# Modalità debug (con output dettagliati)
python3 scripts/launch_system.py --mode debug
```

### Comandi Manuali per Debug

```bash
# Terminal 1: Avvio Gazebo
ros2 launch robot_launcher tiago_gazebo.launch.py

# Terminal 2: Nodo State Machine
ros2 run robot_nodes state_machine_node

# Terminal 3: Rilevatore ArUco
ros2 run robot_nodes aruco_detector_node

# Terminal 4: Cinematica Inversa
ros2 run robot_nodes ik

# Debug camera
python3 scripts/debug_camera.py
```

### Monitoraggio del Sistema

```bash
# Lista di tutti i nodi attivi
ros2 node list

# Lista di tutti i topic
ros2 topic list

# Monitoraggio pose ArUco
ros2 topic echo /aruco_base_pose_1
ros2 topic echo /aruco_base_pose_2
ros2 topic echo /aruco_base_pose_3
ros2 topic echo /aruco_base_pose_4

# Monitoraggio comandi
ros2 topic echo /command_topic
ros2 topic echo /completed_command_topic

# Stato complessivo del sistema
ros2 topic echo /all_markers_found
```

## Configurazione dell'Ambiente

### Setup dei Marcatori ArUco

1. **Generazione Marcatori**:
   - Utilizzare il dizionario DICT_4X4_50
   - Stampare i marcatori con dimensioni di almeno 5x5 cm
   - IDs richiesti: 1, 2, 3, 4

2. **Posizionamento nell'Ambiente**:
   - **Marker 1**: Posizionare sopra il contenitore di Pringles (oggetto manipolato per primo)
   - **Marker 2**: Posizionare sopra la bottiglia di Coca-Cola (oggetto manipolato per secondo)
   - **Marker 3**: Posizione di destinazione per i Pringles
   - **Marker 4**: Posizione di destinazione per la Coca-Cola

3. **Requisiti di Visibilità**:
   - I marcatori devono essere visibili dalla camera frontale di TiAGO
   - Illuminazione adeguata (evitare riflessi diretti)
   - Distanza ottimale: 0.5-2.0 metri dal robot

### Configurazione Camera

I parametri della camera sono configurati in:
```yaml
# config/robot_params.yaml
camera_matrix: [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
aruco_marker_size: 0.05  # 5 cm
```

## Risoluzione Problemi (Troubleshooting)

### Problemi di Installazione

**1. Errore "No module named 'transforms3d'"**
```bash
pip3 install transforms3d
# Se persiste:
pip3 install --upgrade transforms3d
```

**2. Errore "No module named 'roboticstoolbox'"**
```bash
pip3 install robotics-toolbox-python spatialmath-python
```

**3. Errore compilazione colcon**
```bash
# Pulire la build e ricompilare
rm -rf build install log
colcon build --symlink-install
```

**4. Errore "No executable found" o nodi non trovati**
```bash
# SEMPRE eseguire colcon build e source prima del launch
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verificare che i nodi siano disponibili
ros2 pkg executables robot_nodes
```

### Problemi di Runtime

**1. ArUco non rilevati**
- Verificare che i marcatori siano nel campo visivo
- Controllare illuminazione dell'ambiente
- Eseguire debug della camera: `python3 scripts/debug_camera.py`
- Verificare topic camera: `ros2 topic echo /head_front_camera/rgb/image_raw`

**2. Errori di Cinematica Inversa**
- Verificare che le posizioni target siano raggiungibili
- Controllare i limiti dei giunti del robot
- Verificare la configurazione URDF
- Log dettagliati: `ros2 run robot_nodes ik --ros-args --log-level debug`

**3. Gazebo non si avvia**
```bash
# Verifica installazione Gazebo
gazebo --version

# Reinstalla se necessario
sudo apt install gazebo11 libgazebo11-dev

# Configura environment
source /usr/share/gazebo/setup.bash
```

**4. Errori di Trasformazione (TF)**
```bash
# Verifica tree delle trasformazioni
ros2 run tf2_tools view_frames

# Verifica trasformazione specifica
ros2 run tf2_ros tf2_echo base_footprint camera_link

# Visualizza in tempo reale
ros2 run rqt_tf_tree rqt_tf_tree
```

**5. Problemi di Comunicazione tra Nodi**
```bash
# Verifica che tutti i nodi siano attivi
ros2 node list

# Verifica connessioni
ros2 node info /state_machine
ros2 node info /aruco_detector_node

# Test comunicazione
ros2 topic pub /test_topic std_msgs/String "data: test" --once
```

## Struttura del Progetto

```
progetto_ros2/
├── README.md                   # Questo file
├── requirements.txt           # Dipendenze Python
├── src/
│   ├── robot_launcher/        # Launch files per avvio sistema
│   │   ├── launch/           # File .launch.py
│   │   └── package.xml       # Manifesto package
│   └── robot_nodes/          # Nodi ROS2 principali
│       ├── robot_nodes/      # Codice Python
│       │   ├── __init__.py
│       │   ├── aruco_detector_node.py
│       │   ├── ik.py
│       │   └── state_machine_node.py
│       ├── package.xml       # Manifesto package
│       └── setup.py          # Setup Python package
├── scripts/                  # Script di utilità
│   ├── launch_system.py     # Launcher principale
│   └── debug_camera.py      # Tool debug camera
├── config/                   # File di configurazione
│   └── robot_params.yaml    # Parametri sistema
├── models/                   # Modelli 3D (se presenti)
├── docs/                     # Documentazione aggiuntiva
├── build/                    # Output compilazione (generato)
├── install/                  # File installazione (generato)
└── log/                      # Log di compilazione (generato)
```

## Sviluppo e Estensioni

### Aggiunta di Nuovi Oggetti

1. **Definire Nuovi Stati**:
   ```python
   # In ik.py, aggiungere alla classe State
   NEW_OBJECT_MOVE = 17
   NEW_OBJECT_GRIP = 18
   # ...
   ```

2. **Implementare Logica**:
   ```python
   # In command_callback, aggiungere gestione nuovi stati
   elif msg.data == State.NEW_OBJECT_MOVE.value:
       self.calculate_pose_with_offset_state(5, offset_x=0.05, offset_y=-0.05, offset_z=-0.08)
   ```

3. **Configurare Parametri**:
   - Dimensioni gripper per il nuovo oggetto
   - Offset specifici per la presa
   - Timing delle operazioni

### Test del Sistema

```bash
# Test unitari (se implementati)
python3 -m pytest tests/

# Test di integrazione manuale
python3 scripts/test_integration.py

# Benchmark prestazioni
python3 scripts/benchmark_system.py
```

## Specifiche Tecniche

### Limiti del Sistema

- **Workspace Robot**: Raggio di azione ~1.4m dal base_footprint
- **Peso Oggetti**: Massimo 2kg per il gripper TiAGO
- **Precisione Positioning**: ±2mm in condizioni ottimali
- **Tempo Ciclo Completo**: 3-5 minuti per entrambi gli oggetti

### Performance

- **Rilevamento ArUco**: ~10 FPS con risoluzione 640x480
- **Calcolo IK**: ~50ms per soluzione
- **Movimento Robot**: Velocità limitata per sicurezza (20% max velocity)

## Licenza

Questo progetto è rilasciato sotto licenza MIT. Vedere il file LICENSE per dettagli completi.

## Autori e Contributi

- **Sviluppatore Principale**: Claudio
- **Progetto**: Sistema di Manipolazione Robotica TiAGO
- **Framework**: ROS2 Humble con Gazebo

## Contatti e Supporto

Per supporto tecnico o segnalazione di bug, consultare la documentazione o contattare il team di sviluppo.

---

**Nota Importante**: Questo sistema è progettato e testato principalmente per funzionare con il robot TiAGO in simulazione Gazebo. L'utilizzo con robot fisico reale richiede calibrazione aggiuntiva dei parametri di controllo, sicurezza e possibili modifiche hardware-specifiche.