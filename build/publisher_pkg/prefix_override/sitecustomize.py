import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dogan/ros2_ws/2024_RoboticsAi2TI_Mehmet_Schepens/install/publisher_pkg'
