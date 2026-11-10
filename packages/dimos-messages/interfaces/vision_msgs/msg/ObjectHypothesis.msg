# An object hypothesis that contains no pose information.
# If you would like to define an array of ObjectHypothesis messages,
#   please see the Classification message type.

# The unique ID of the object class. To get additional information about
#   this ID, such as its human-readable class name, listeners should perform a
#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
string class_id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score
