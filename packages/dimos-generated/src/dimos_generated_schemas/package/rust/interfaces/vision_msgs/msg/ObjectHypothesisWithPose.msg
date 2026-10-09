# An object hypothesis that contains pose information.
# If you would like to define an array of ObjectHypothesisWithPose messages,
#   please see the Detection2D or Detection3D message types.

# The object hypothesis (ID and score).
ObjectHypothesis hypothesis

# The 6D pose of the object hypothesis. This pose should be
#   defined as the pose of some fixed reference point on the object, such as
#   the geometric center of the bounding box, the center of mass of the
#   object or the origin of a reference mesh of the object.
# Note that this pose is not stamped; frame information can be defined by
#   parent messages.
# Also note that different classes predicted for the same input data may have
#   different predicted 6D poses.
geometry_msgs/PoseWithCovariance pose
