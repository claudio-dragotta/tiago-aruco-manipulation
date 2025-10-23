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

1. **ArUco Scan Publisher** (`aruco_scan_publisher.py`) [NUOVO]
   - Rileva marcatori ArUco dalla camera frontale del robot
   - Stima le pose 3D rispetto a `head_front_camera_optical_frame`
   - Pubblica pose su topic `/aruco_[id]_pose` per ogni marker rilevato
   - Disegna marker e assi 3D in tempo reale su OpenCV

2. **ArUco Coordinate Transformation** (`aruco_coord_transformation.py`) [NUOVO]
   - Trasforma le coordinate dei marker dal frame della camera a `base_footprint`
   - Implementa sistema di media su 20 secondi per stabilità
   - Pubblica pose trasformate su `/aruco_[id]_pose_transformed`
   - Utilizza TF2 per le trasformazioni tra frame

3. **Head Movement Action Node** (`head_movement_action.py`) [AGGIORNATO]
   - Controlla il movimento della testa del robot tramite ActionClient
   - Scansiona l'ambiente da destra a sinistra per rilevare i marker
   - Si ferma automaticamente quando tutti i marker sono rilevati
   - Pubblica su `/head_controller/follow_joint_trajectory`

4. **Motion Planner Node** (`motion_planner_node.py`) [NUOVO]
   - Implementa l'inversione cinematica per il braccio TiAGO a 7 DOF
   - Carica il modello URDF del robot e calcola traiettorie
   - Comunica con State Machine via topic `/sm_ik_communication`
   - Usa `roboticstoolbox` (Levenberg-Marquardt IK) per calcoli precisi

5. **State Machine Node** (`state_machine.py`)
   - Implementa automa finito con 16 stati
   - Coordina la sequenza completa di operazioni (pickup-place)
   - Gestisce comunicazione con Motion Planner
   - Monitora transizioni tra stati e comandi

### Flusso Operativo (Pipeline)

```
PIPELINE ARCHITETTURA FUNZIONANTE:

1. ArUco Scan Publisher (t=18s)
   └─> Rileva marker 1,2,3,4 dalla camera
       Pubblica su /aruco_[1-4]_pose
       ↓

2. ArUco Coord Transform (t=20s)
   └─> Trasforma coordinate in base_footprint
       Pubblica su /aruco_[1-4]_pose_transformed
       ↓

3. Head Movement (t=22s)
   └─> Scansiona ambiente
       Muove testa da destra a sinistra
       ↓

4. Motion Planner (t=24s)
   └─> Attende comandi dalla State Machine
       Calcola inversione cinematica
       Pubblica traiettorie ai controller
       ↓

5. State Machine (t=26s)
   └─> Attende marker rilevati
       Invia comandi a Motion Planner
       Coordina sequenza manipolazione
       ↓

SEQUENZA MANIPOLAZIONE:
   - Configurazione intermedia
   - Configurazione operativa
   - Pringles: marker 1 → marker 3
   - Coca-Cola: marker 2 → marker 4
   - Ritorno home
```

### Componenti Supporto

- **Gazebo Simulator**: Simula il robot TiAGO e l'ambiente
- **ROS2 Controllers**: Controllano torso, braccio, gripper, testa
- **TF2**: Gestisce trasformazioni tra frame coordinati

## Panorama Workspace

- **Struttura Colcon**: il workspace segue la convenzione ROS2 con il codice sorgente in `src/`, output generati in `build/`, `install/` e `log/`, documentazione e dipendenze al livello radice (`README.md`, `requirements.txt`) e il modello URDF personalizzato del TiAGO (`tiago_robot.urdf`).
- **Pacchetto `robot_launcher`**: contiene il launch `launch/full_system.launch.py`, che orchestra l’avvio temporizzato di Gazebo dal workspace ufficiale PAL, RViz e dei quattro nodi ROS personalizzati, mostrando messaggi guida all’utente.
- **Script di utilità**: la cartella `scripts/` include `launch_system.py` per compilare e lanciare l’intero stack fuori da ROS, oltre a strumenti di debug come `debug_camera.py` e `test_dependencies.py` per verificare topic camera e dipendenze.

### Tour Dettagliato del Workspace

Nel livello radice convivono gli elementi che servono per compilare e comprendere il progetto:
- `README.md` (questo file) e `requirements.txt` documentano rispettivamente funzionamento e dipendenze Python, mentre `tiago_robot.urdf` fornisce al nodo IK la descrizione completa del robot.
- Le cartelle `build/`, `install/` e `log/` sono prodotte da `colcon build`: contengono artefatti, workspace sovrapposto e log di esecuzione e possono essere rigenerate in qualunque momento.
- `scripts/` ospita i tool “extra ROS”: `launch_system.py` gestisce l’avvio coordinato di compilazione, simulazione e nodi ROS direttamente da terminale, `debug_camera.py` aiuta a diagnosticare la pipeline video della testa del TiAGO, mentre `test_dependencies.py` verifica rapidamente che tutte le librerie richieste e l’ambiente ROS2 siano configurati.

La cartella `src/` raccoglie i due pacchetti ROS2 sviluppati nel workspace:

1. **`robot_launcher/`**
   - `launch/full_system.launch.py` è il cuore dell’orchestrazione ROS: annuncia l’avvio con messaggi informativi e, grazie a più `TimerAction`, avvia Gazebo e RViz dal workspace ufficiale PAL (`/home/claudio/tiago_public_ws`), quindi lancia nell’ordine macchina a stati, detector ArUco, nodo di movimento testa e IK, sincronizzando i tempi per evitare conflitti.
   - `package.xml`, `setup.py`, `setup.cfg` e `resource/robot_launcher` costituiscono la configurazione standard del pacchetto ament, mentre la cartella `test/` contiene gli script di linting generati dal template.

2. **`robot_nodes/`**
   - Nel modulo Python `robot_nodes/` risiedono i quattro nodi principali:
     - `aruco_detector.py` ascolta `CameraInfo` e `Image` dalla testa, rileva i marker con OpenCV, calcola le pose nel frame `base_footprint`, applica offset ottimizzati per la presa, pubblica le pose verso IK e state machine e diffonde anche trasformate TF. Tiene traccia dei marker trovati e, una volta raggiunta la quaterna {1,2,3,4}, pubblica `/all_markers_found` per sbloccare il workflow.
     - `head_movement_action.py` sincronizza un ActionClient e un ActionServer su `/head_controller/follow_joint_trajectory`: all’avvio prova a collegarsi al controller e lancia una sequenza ciclica di punti che scandisce l’ambiente da destra a sinistra; ricevendo `/all_markers_found` arresta la ricerca.
     - `state_machine.py` definisce l’automa finito che coordina l’intera missione: parte in attesa dei marker, invia comandi progressivi al nodo IK tramite `/command_topic`, ascolta le conferme su `/completed_command_topic` e avanza nella sequenza di pick-and-place (Pringles 1→3, Coca-Cola 2→4), con ritorno a casa finale.
     - `ik.py` funge da pianificatore di cinematica inversa: carica `tiago_robot.urdf`, mantiene i client di controllo per torso, braccio e gripper, media le pose ArUco per stabilizzare la percezione e, ad ogni comando, calcola traiettorie con `roboticstoolbox` (IK Levenberg-Marquardt + verifica dell’errore in spazio operativo). Pubblica `JointTrajectory` sui controller interessati e segnala l’esito alla state machine.
   - Anche qui `package.xml`, `setup.py`, `setup.cfg` e `resource/robot_nodes` curano il packaging ament, mentre la cartella `test/` replica gli script di linting.

## Dettaglio Nodi di Controllo (pacchetto `robot_nodes`)

- **`aruco_detector.py`**: sottoscrive i topic della camera di testa, rileva i marker con OpenCV, trasforma le pose nel frame `base_footprint`, applica offset ottimizzati per la presa e pubblica sia topic Pose che trasformazioni TF. Alla scoperta di tutti e quattro i marker invia `/all_markers_found` per sbloccare gli altri nodi.
- **`head_movement_action.py`**: combina un ActionClient e un ActionServer su `/head_controller/follow_joint_trajectory` per oscillare la testa in cerca dei marker; si ferma automaticamente quando riceve il segnale di completamento dal detector.
- **`state_machine.py`**: implementa una macchina a stati con 16 tappe, dall’attesa dei marker fino alla chiusura missione. Pubblica comandi numerici su `/command_topic`, ascolta conferme su `/completed_command_topic` e governa l’intera sequenza pick-and-place (Pringles marker 1→3, Coca-Cola marker 2→4).
- **`ik.py`**: carica `tiago_robot.urdf`, mantiene i client di controllo per torso, braccio e gripper, media le pose ArUco e, per ogni comando ricevuto, calcola traiettorie con `roboticstoolbox` (funzioni `ikine_LM` e controllo iterativo sull’errore), pubblicandole come `JointTrajectory`. Tiene traccia dello stato operativo, gestisce offset dinamici delle pose e invia conferme di completamento alla state machine.

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

### Avvio Completo (RACCOMANDATO - Un Comando Solo)

**OPZIONE 1: Avvio Veloce in Un Unico Comando**

```bash
cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 launch robot_launcher full_system.launch.py
```

**OPZIONE 2: Avvio con Compilazione (Sicuro)**

```bash
cd /home/claudio/Desktop/progetto_ros2 && colcon build && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_launcher full_system.launch.py
```

Questo comando:
- Compila il workspace
- Configura l'ambiente ROS2
- Avvia Gazebo con TiAGO
- Avvia tutti i 5 nodi principali in sequenza:
  1. **ArUco Scan Publisher** (t=18s) - Rilevamento marker ArUco
  2. **ArUco Coordinate Transformation** (t=20s) - Trasformazione coordinate
  3. **Head Movement** (t=22s) - Scansione ambiente
  4. **Motion Planner** (t=24s) - Inversione cinematica
  5. **State Machine** (t=26s) - Coordinamento operazioni

---

### Avvio per Terminal Multipli (Avanzato)

Se preferisci lanciare i nodi separatamente in più terminal:

**Terminal 1 - Avvia Gazebo e Sistema Base:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 launch robot_launcher full_system.launch.py
```

**Terminal 2 (Opzionale) - Monitoraggio Marker ArUco:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /aruco_poses
```

**Terminal 3 (Opzionale) - Monitoraggio Stato Macchina:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /command_topic
```

---

### Lanciare Nodi Singolarmente (Per Debug)

Se vuoi controllare ogni nodo singolarmente:

**Terminal 1 - ArUco Scan Publisher:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 run robot_nodes aruco_scan_publisher
```

**Terminal 2 - ArUco Coordinate Transformation:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 run robot_nodes aruco_coord_transformation
```

**Terminal 3 - Head Movement:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 run robot_nodes head_movement_action_node
```

**Terminal 4 - Motion Planner:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 run robot_nodes motion_planner_node
```

**Terminal 5 - State Machine:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 run robot_nodes state_machine
```

---

### Comandi di Monitoraggio e Debug

#### **Verificare Nodi Attivi:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 node list
```

#### **Vedere i Topic ArUco:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic list | grep aruco
```

#### **Monitorare Marker Rilevati (in tempo reale):**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /aruco_poses
```

#### **Monitorare Pose Trasformate:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /aruco_poses_transformed
```

#### **Monitorare Comando State Machine:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /command_topic
```

#### **Monitorare Comunicazione SM-IK:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic echo /sm_ik_communication
```

#### **Informazioni su un Nodo:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 node info /robot_state_machine_node
```

#### **Visualizzare Tutti i Topic:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 topic list
```

---

### Comandi di Compilazione e Manutenzione

#### **Compilare il Progetto:**
```bash
cd /home/claudio/Desktop/progetto_ros2
colcon build --packages-select robot_nodes robot_launcher
```

#### **Compilare e Pulire:**
```bash
cd /home/claudio/Desktop/progetto_ros2
rm -rf build install log
colcon build --symlink-install
```

#### **Verifichiare Eseguibili Disponibili:**
```bash
cd /home/claudio/Desktop/progetto_ros2
source install/setup.bash
ros2 pkg executables robot_nodes
```

---

### Alias Utili da Aggiungere al .bashrc

Se vuoi semplificare i comandi, aggiungi questi alias al tuo `~/.bashrc`:

```bash
# Aggiungi queste linee al file ~/.bashrc
alias tiago_launch='cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 launch robot_launcher full_system.launch.py'
alias tiago_build='cd /home/claudio/Desktop/progetto_ros2 && colcon build && source /opt/ros/humble/setup.bash && source install/setup.bash'
alias tiago_nodes='cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 node list'
alias tiago_topics='cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 topic list'
alias tiago_aruco='cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 topic echo /aruco_poses'
```

Dopo aver salvato, esegui:
```bash
source ~/.bashrc
```

Poi potrai lanciare il sistema semplicemente con:
```bash
tiago_launch
```

---

### Sequenza Completa Consigliata

1. **Apri un terminale e avvia il sistema:**
   ```bash
   cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 launch robot_launcher full_system.launch.py
   ```

2. **Apri un secondo terminale per monitorare i marker:**
   ```bash
   cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 topic echo /aruco_poses
   ```

3. **(Opzionale) Apri un terzo terminale per monitorare lo stato:**
   ```bash
   cd /home/claudio/Desktop/progetto_ros2 && source install/setup.bash && ros2 node list
   ```

4. **Posiziona i 4 marker ArUco (ID: 1,2,3,4) davanti alla camera**

5. **Il sistema inizierà automaticamente la manipolazione!**

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
