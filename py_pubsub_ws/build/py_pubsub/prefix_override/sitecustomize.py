import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaewoo/ros2/py_pubsub_ws/install/py_pubsub'
