# An array of trajectory points that represents a trajectory for a robot to follow.

# Indicates the frame_id in which the planner ran and timestamp represents when the trajectory was generated.
std_msgs/Header header

# Array of trajectory points to follow.
TrajectoryPoint[] points
