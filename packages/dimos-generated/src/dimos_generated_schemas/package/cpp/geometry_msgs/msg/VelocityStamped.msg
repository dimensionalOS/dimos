# This expresses the timestamped velocity vector of a frame 'body_frame_id' in the reference frame 'reference_frame_id' expressed from arbitrary observation frame 'header.frame_id'.
# - If the 'body_frame_id' and 'header.frame_id' are identical, the velocity is observed and defined in the local coordinates system of the body
#   which is the usual use-case in mobile robotics and is also known as a body twist.

std_msgs/Header header
string body_frame_id
string reference_frame_id
Twist velocity
