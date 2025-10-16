std_msgs/Header header

string[] names

# Floating base
geometry_msgs/Pose[] base_pose
geometry_msgs/Twist[] base_twist
geometry_msgs/Twist[] base_accel

# Joint
string[] joint_names
float64[] joint_pos
float64[] joint_vel

# Phase
float64 phase