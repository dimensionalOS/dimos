# A list of 2D detections, for a multi-object 2D detector.

std_msgs/Header header

# A list of the detected proposals. A multi-proposal detector might generate
#   this list with many candidate detections generated from a single input.
Detection2D[] detections
