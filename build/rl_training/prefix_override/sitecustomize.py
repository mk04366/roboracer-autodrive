import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/autroboracer/ros2_ws/src/roboracer-autodrive/install/rl_training'
