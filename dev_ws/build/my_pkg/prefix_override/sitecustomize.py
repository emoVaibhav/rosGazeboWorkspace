import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vd/ros_gazebo_workspace/dev_ws/install/my_pkg'
