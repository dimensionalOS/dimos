# Defines a 2D detection result.
#
# This is similar to a 2D classification, but includes position information,
#   allowing a classification result for a specific crop or image point to
#   to be located in the larger image.

std_msgs/Header header

# Class probabilities
ObjectHypothesisWithPose[] results

# 2D bounding box surrounding the object.
BoundingBox2D bbox

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id

# Source data that generated this detection are not a part of the message.
# If you need to access them, use an exact or approximate time synchronizer in
# your code, as this message's header should match the header of the source
# data.
