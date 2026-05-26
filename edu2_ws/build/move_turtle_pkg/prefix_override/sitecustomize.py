import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yoo/ROS2_education/edu2_ws/install/move_turtle_pkg'
