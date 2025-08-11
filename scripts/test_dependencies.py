#!/usr/bin/env python3
"""
Script di test per verificare che tutte le dipendenze siano installate correttamente.
"""

import sys

def test_imports():
    """Testa l'importazione di tutte le librerie richieste."""
    
    required_packages = {
        'cv2': 'opencv-python',
        'transforms3d': 'transforms3d', 
        'numpy': 'numpy',
        'scipy': 'scipy',
        'roboticstoolbox': 'robotics-toolbox-python',
        'spatialmath': 'spatialmath-python',
        'matplotlib': 'matplotlib'
    }
    
    print("=== TEST DIPENDENZE PYTHON ===")
    print()
    
    failed_imports = []
    
    for package, pip_name in required_packages.items():
        try:
            __import__(package)
            print(f"OK    - {package} ({pip_name})")
        except ImportError as e:
            print(f"FAIL  - {package} ({pip_name}) - {e}")
            failed_imports.append((package, pip_name))
    
    print()
    
    if failed_imports:
        print("ERRORE: Le seguenti dipendenze mancano:")
        for package, pip_name in failed_imports:
            print(f"  pip3 install {pip_name}")
        print()
        print("Eseguire:")
        print("pip3 install -r requirements.txt")
        return False
    else:
        print("SUCCESSO: Tutte le dipendenze Python sono installate!")
        return True

def test_ros2():
    """Testa se ROS2 è configurato correttamente."""
    
    print("=== TEST AMBIENTE ROS2 ===")
    print()
    
    import os
    
    # Verifica variabili ambiente ROS2
    ros_distro = os.environ.get('ROS_DISTRO', None)
    if ros_distro:
        print(f"OK    - ROS_DISTRO: {ros_distro}")
    else:
        print("WARN  - ROS_DISTRO non trovata")
        print("       Eseguire: source /opt/ros/humble/setup.bash")
        return False
    
    # Testa import rclpy
    try:
        import rclpy
        print("OK    - rclpy importato correttamente")
    except ImportError:
        print("FAIL  - rclpy non trovato")
        print("       Installare: sudo apt install ros-humble-desktop-full")
        return False
    
    print()
    print("SUCCESSO: Ambiente ROS2 configurato!")
    return True

def main():
    """Funzione principale di test."""
    
    print("VERIFICA SISTEMA - TiAGO ArUco Manipulation")
    print("=" * 50)
    print()
    
    # Test dipendenze Python
    python_ok = test_imports()
    print()
    
    # Test ROS2
    ros2_ok = test_ros2()
    print()
    
    # Risultato finale
    print("=== RISULTATO FINALE ===")
    if python_ok and ros2_ok:
        print("SUCCESSO: Sistema pronto per l'utilizzo!")
        print()
        print("Prossimi passi:")
        print("1. Compilare: colcon build --symlink-install")
        print("2. Configurare: source install/setup.bash") 
        print("3. Avviare: python3 scripts/launch_system.py --mode full")
        sys.exit(0)
    else:
        print("ERRORE: Sistema non pronto. Risolvere i problemi sopra indicati.")
        sys.exit(1)

if __name__ == "__main__":
    main()
