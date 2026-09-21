# An array of navigation goals


# This header will store the time at which the poses were computed (not to be confused with the stamps of the poses themselves)
# In the case that individual poses do not have their frame_id set or their timetamp set they will use the default value here.
std_msgs/Header header

# An array of goals to for navigation to achieve.
# The goals should be executed in the order of the array.
# The header and stamp are intended to be used for computing the position of the goals.
# They may vary to support cases of goals that are moving with respect to the robot.
geometry_msgs/PoseStamped[] goals
