#!/usr/bin/env python3

"""
Sistema di lancio semplificato per TiAGO ArUco Manipulation
"""

import subprocess
import os
import sys
import time
import signal
from pathlib import Path
import argparse

class TiAGoLauncher:
    def __init__(self):
        self.project_ws = "/home/claudio/Desktop/progetto_ros2"
        self.tiago_ws = "/home/claudio/tiago_public_ws"
        self.processes = []
        self.running = True
        
        if not Path(self.project_ws).exists():
            raise FileNotFoundError(f"Workspace progetto non trovato: {self.project_ws}")
        
        self.tiago_available = Path(self.tiago_ws).exists()
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        print('\nSpegnimento sistema in corso...')
        self.running = False
        
        for process in self.processes:
            if process and process.poll() is None:
                process.terminate()
                
        time.sleep(2)
        
        for process in self.processes:
            if process and process.poll() is None:
                process.kill()
                
        print("Processi terminati.")
        sys.exit(0)
    
    def build_workspace(self):
        print("Compilazione workspace...")
        result = subprocess.run(
            "colcon build --symlink-install",
            shell=True, cwd=self.project_ws
        )
        return result.returncode == 0
    
    def launch_gazebo_and_rviz(self):
        """Avvia Gazebo e RViz dal workspace TiAGO"""
        print("Avvio Gazebo con TiAGO...")
        
        # Environment per TiAGO workspace
        tiago_env = os.environ.copy()
        
        # Comando per Gazebo
        gazebo_cmd = f'bash -c "source {self.tiago_ws}/install/setup.bash && ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True"'
        
        gazebo_process = subprocess.Popen(
            gazebo_cmd, shell=True, env=tiago_env, executable='/bin/bash'
        )
        self.processes.append(gazebo_process)
        
        print("Gazebo avviato. Avvio RViz...")
        time.sleep(10)  # Aspetta un po' prima di avviare RViz
        
        # Comando per RViz
        rviz_cmd = f'bash -c "source {self.tiago_ws}/install/setup.bash && ros2 run rviz2 rviz2"'
        
        rviz_process = subprocess.Popen(
            rviz_cmd, shell=True, env=tiago_env, executable='/bin/bash'
        )
        self.processes.append(rviz_process)
        
        print("RViz avviato. Sistema grafico completo attivo.")
    
    def run_system(self, mode="full"):
        if not self.build_workspace():
            return False
            
        print(f"Avvio sistema modalità: {mode}")
        
        if mode == "full":
            launch_file = "full_system.launch.py"
        elif mode == "debug": 
            launch_file = "debug_system.launch.py"
        else:
            launch_file = "main_system.launch.py"
        
        # Avvio Gazebo e RViz se modalità full e workspace TiAGO disponibile
        if mode == "full" and self.tiago_available:
            self.launch_gazebo_and_rviz()
        
        env = os.environ.copy()
        
        # Setup environment
        if self.tiago_available:
            subprocess.run(f'bash -c "source {self.tiago_ws}/install/setup.bash"', 
                         shell=True, env=env)
        
        subprocess.run(f'bash -c "source {self.project_ws}/install/setup.bash"',
                      shell=True, env=env)
        
        # Aspetta che Gazebo si carichi prima di avviare i nodi ArUco
        if mode == "full" and self.tiago_available:
            print("Aspetto che Gazebo si carichi completamente... (45 secondi)")
            time.sleep(45)
        
        cmd = f"source install/setup.bash && ros2 launch robot_launcher {launch_file}"
        
        process = subprocess.Popen(
            cmd, shell=True, cwd=self.project_ws, env=env,
            executable='/bin/bash'
        )
        
        self.processes.append(process)
        
        print(f"Sistema completo avviato: Gazebo + RViz + Nodi ArUco")
        print("Aspetta che tutti i componenti si carichino, poi posiziona i 4 marker ArUco.")
        print("Premere Ctrl+C per arrestare tutto.")
        
        try:
            while self.running:
                time.sleep(1)
                if process.poll() is not None:
                    break
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)
            
        return True

def main():
    parser = argparse.ArgumentParser(description='TiAGO ArUco System Launcher')
    parser.add_argument('--mode', choices=['full', 'debug', 'nodes'], 
                       default='full', help='Modalità di lancio')
    parser.add_argument('--build-only', action='store_true', 
                       help='Solo compila')
    
    args = parser.parse_args()
    
    try:
        launcher = TiAGoLauncher()
        
        if args.build_only:
            return launcher.build_workspace()
        
        return launcher.run_system(args.mode)
        
    except Exception as e:
        print(f"ERRORE: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
