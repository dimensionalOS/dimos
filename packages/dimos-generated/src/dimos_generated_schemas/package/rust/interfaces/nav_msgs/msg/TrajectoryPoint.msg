# Trajectory point state

# Absolute time and frame of reference of the point along a trajectory
std_msgs/Header header

# Pose of the trajectory sample.
geometry_msgs/Pose pose

# Velocity of the trajectory sample.
geometry_msgs/Twist velocity

# Acceleration of the trajectory (optional, linear.x component is NaN if not set).
geometry_msgs/Accel acceleration

# Force/Torque to apply at trajectory sample (optional, force.x component is NaN if not set).
geometry_msgs/Wrench effort
