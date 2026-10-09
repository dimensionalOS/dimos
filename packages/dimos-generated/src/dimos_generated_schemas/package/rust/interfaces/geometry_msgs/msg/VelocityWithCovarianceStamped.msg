# A timestamped velocity of a body whose frame is 'body_frame_id', measured
# relative to the reference frame 'reference_frame_id', with the velocity and
# covariance both expressed in the basis of the observation frame
# 'header.frame_id'.
#
# - If 'body_frame_id' and 'header.frame_id' are identical, the velocity and
#   covariance are expressed in the body's own basis. This is functionally
#   equivalent to the body-twist convention used by
#   'geometry_msgs/TwistStamped'.
#
# This message is the covariance-bearing analogue of
# 'geometry_msgs/VelocityStamped'.

std_msgs/Header header
string body_frame_id
string reference_frame_id
TwistWithCovariance velocity
