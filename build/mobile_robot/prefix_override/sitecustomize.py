import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/benoitrouchy/Autonomous_Robot-master/install/mobile_robot'
