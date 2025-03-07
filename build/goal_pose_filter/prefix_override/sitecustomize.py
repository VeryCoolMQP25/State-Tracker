import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/suki/ros2_ws/src/goal_pose_filter/install/goal_pose_filter'
