import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/russe/Autonomous_Robot/src/install/mobile_robot'
