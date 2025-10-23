std_msgs/Header header

# List of the index
int32[] index

# List of the geometric properties of each module
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist

# Value of the joint of each module
float64[] joint_motor
float64[] joint_paddle

# Force applied to its COM for each module
geometry_msgs/Vector3[] force

# List of the phase of each module
float64[] phase
