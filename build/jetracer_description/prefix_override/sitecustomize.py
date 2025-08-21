import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetracer/ros2_ws/src/jetracer_description/install/jetracer_description'
