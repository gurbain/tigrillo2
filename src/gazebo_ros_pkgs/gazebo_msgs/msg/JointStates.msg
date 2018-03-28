# broadcast all joint states in world frame
string[] name                 # joint names
# joint states
float64[] position
float64[] rate
geometry_msgs/Vector3[] axes
geometry_msgs/Vector3[] body1Forces
geometry_msgs/Vector3[] body2Forces
geometry_msgs/Vector3[] body1Torques
geometry_msgs/Vector3[] body2Torques
