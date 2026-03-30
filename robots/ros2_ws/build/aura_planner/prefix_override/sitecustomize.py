import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/sf_aura_shared/ros2_ws/install/aura_planner'
