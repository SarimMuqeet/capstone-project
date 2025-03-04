import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/homey/Desktop/capstone-project/install/sipeed_tof'
