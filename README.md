# TiAGO ArUco Manipulation System

Sistema di manipolazione robotica autonoma che utilizza il robot TiAGO per rilevare marker ArUco e movimentare oggetti tra posizioni predefinite.

## Funzionalità

Il sistema implementa una pipeline completa di manipolazione robotica:

1. **Rilevamento ArUco**: Scansiona l'ambiente con la camera della testa per identificare 4 marker ArUco (ID: 1, 2, 3, 4)
2. **Cinematica inversa**: Calcola le traiettorie necessarie per raggiungere gli oggetti
3. **Manipolazione coordinata**: Esegue sequenze di presa e rilascio tra le posizioni dei marker
4. **Controllo stato**: Coordina tutte le operazioni attraverso una macchina a stati centralizzata

## Architettura Sistema

### Nodi ROS2

- **state_machine**: Coordinatore centrale che gestisce la sequenza completa di operazioni
- **aruco_detector_node**: Rileva marker ArUco e pubblica le pose nel frame `base_footprint`
- **head_movement_action_node**: Controlla i movimenti della testa per la scansione
- **ik**: Calcola la cinematica inversa per la pianificazione delle traiettorie

### Topic Principali

- `/aruco_base_pose_1` a `/aruco_base_pose_4`: Pose dei marker ArUco rilevati
- `/command_topic`: Comandi inviati dalla macchina stati ai nodi esecutori
- `/completed_command_topic`: Notifiche di completamento delle operazioni
- `/all_markers_found`: Segnale che indica il completamento della fase di rilevamento
- `/marker_discovered`: Notifica della scoperta di un nuovo marker

## Installazione e Setup

### Prerequisiti

- ROS2 Humble
- Workspace TiAGO configurato in `/home/claudio/tiago_public_ws`
- Pacchetti Python richiesti:

```bash
pip3 install opencv-python transforms3d roboticstoolbox-python spatialmath-python
```

### Compilazione

```bash
cd /home/claudio/Desktop/progetto_ros2
colcon build --symlink-install
```

## Utilizzo

### Avvio Sistema Completo (Raccomandato)

```bash
# Sistema completo con Gazebo
python3 scripts/launch_system.py --mode full
```

### Modalità Debug

```bash
# Per sviluppo e test con output dettagliati
python3 scripts/launch_system.py --mode debug
```

### Solo Nodi (senza Gazebo)

```bash
# Solo i nodi del sistema
python3 scripts/launch_system.py --mode nodes
```

### Verifica Compilazione

```bash
# Test compilazione senza avvio
python3 scripts/launch_system.py --build-only
```

## Preparazione Ambiente

1. **Setup marker ArUco**: Posizionare 4 marker ArUco (ID: 1, 2, 3, 4) nel campo visivo della camera TiAGO
2. **Avvio Gazebo**: Il sistema avvierà automaticamente Gazebo con TiAGO (modalità `full`)
3. **Monitoraggio**: Osservare i log per verificare il rilevamento dei marker

## Sequenza Operativa

1. **Fase Inizializzazione**: Avvio coordinato di tutti i nodi (8-15 secondi)
2. **Fase Scansione**: Movimento della testa per ricerca marker ArUco
3. **Fase Rilevamento**: Identificazione e localizzazione dei 4 marker
4. **Fase Manipolazione**: Esecuzione sequenze di presa e rilascio
5. **Completamento**: Ritorno alla posizione home

## Monitoraggio e Debug

### Verifica Stato Sistema

```bash
# Stato marker rilevati
ros2 topic echo /all_markers_found

# Pose marker individuali
ros2 topic echo /aruco_base_pose_1

# Comandi in esecuzione
ros2 topic echo /command_topic
```

### Log e Diagnostica

I log del sistema sono visualizzati nel terminale durante l'esecuzione con informazioni dettagliate su:
- Stato rilevamento marker
- Calcoli cinematica inversa  
- Progressione macchina stati
- Errori e avvertimenti

## Struttura Workspace

```
progetto_ros2/
├── README.md
├── scripts/
│   ├── launch_system.py      # Launcher principale
│   └── debug_camera.py       # Tool debug camera
├── src/
│   ├── robot_nodes/          # Pacchetto nodi sistema
│   └── robot_launcher/       # Pacchetto launch files
├── build/                    # Build output (generato)
├── install/                  # Installazione (generato)
└── log/                      # Log build (generato)
```

## Troubleshooting

### Problema: Marker non rilevati
- Verificare illuminazione ambiente
- Controllare che i marker siano nel campo visivo della camera
- Verificare topic camera: `ros2 topic echo /head_front_camera/rgb/image_raw`

### Problema: Errori comunicazione
- Verificare che tutti i nodi siano attivi: `ros2 node list`
- Controllare topic disponibili: `ros2 topic list`

### Problema: Errori cinematica
- Verificare che le pose dei marker siano raggiungibili
- Controllare i log del nodo `ik` per errori specifici

## Autore

Claudio - Progetto Robotica Avanzata TiAGO