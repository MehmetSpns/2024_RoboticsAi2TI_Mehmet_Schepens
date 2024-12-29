import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dogan/ros2_ws/src/cartographer_pkg/install/cartographer_pkg'
