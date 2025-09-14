#!/usr/bin/env python3

"""
Script per debug della camera e topic ROS2
"""

import subprocess
import sys
import time

def run_command(cmd):
    """Esegue comando e ritorna output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.stdout.strip()
    except:
        return "ERRORE nell'esecuzione comando"

def main():
    print("=== DEBUG CAMERA TIAGO ===\n")
    
    print("1. Verifica topic disponibili...")
    topics = run_command("ros2 topic list")
    print("Topic disponibili:")
    for topic in topics.split('\n'):
        if 'camera' in topic.lower() or 'image' in topic.lower():
            print(f"  {topic}")
    
    print("\n2. Verifica topic camera specifici...")
    camera_topics = [
        "/head_front_camera/rgb/image_raw",
        "/head_front_camera/rgb/camera_info", 
        "/head_front_camera/color/image_raw",
        "/head_front_camera/color/camera_info",
        "/xtion/rgb/image_raw",
        "/xtion/rgb/camera_info"
    ]
    
    for topic in camera_topics:
        output = run_command(f"ros2 topic info {topic}")
        if "not found" not in output.lower() and "error" not in output.lower():
            print(f"  {topic} - DISPONIBILE")
            # Verifica frequenza
            hz_output = run_command(f"timeout 3 ros2 topic hz {topic}")
            if "average rate" in hz_output:
                hz = hz_output.split("average rate: ")[1].split("\n")[0]
                print(f"     Frequenza: {hz}")
        else:
            print(f"  {topic} - NON DISPONIBILE")
    
    print("\n3. Verifica nodi camera...")
    nodes = run_command("ros2 node list")
    print("Nodi attivi:")
    for node in nodes.split('\n'):
        if any(keyword in node.lower() for keyword in ['camera', 'gazebo', 'tiago']):
            print(f"  {node}")
    
    print("\n4. Comandi utili per debug:")
    print("  # Visualizza immagini camera:")
    print("  ros2 run rqt_image_view rqt_image_view")
    print("  ")
    print("  # Test specifico topic:")
    print("  ros2 topic echo /head_front_camera/rgb/image_raw --once")
    print("  ")
    print("  # Frequenza topic:")
    print("  ros2 topic hz /head_front_camera/rgb/image_raw")

if __name__ == "__main__":
    main()