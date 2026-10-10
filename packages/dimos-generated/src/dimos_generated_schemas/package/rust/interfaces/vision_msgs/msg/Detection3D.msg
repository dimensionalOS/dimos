# Defines a 3D detection result.
#
# This extends a basic 3D classification by including the pose of the
# detected object.

std_msgs/Header header

# Class probabilities. Does not have to include hypotheses for all possible
#   object ids, the scores for any ids not listed are assumed to be 0.
ObjectHypothesisWithPose[] results

# 3D bounding box surrounding the object.
BoundingBox3D bbox

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id

# Source data that generated this classification are not a part of the message.
# If you need to access them, use an exact or approximate time synchronizer in
# your code, as this message's header should match the header of the source
# data.
