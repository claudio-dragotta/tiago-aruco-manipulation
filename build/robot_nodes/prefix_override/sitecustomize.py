import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/claudio/Desktop/progetto_ros2/install/robot_nodes'
