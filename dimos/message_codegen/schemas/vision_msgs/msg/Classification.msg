# Defines a classification result.
#
# This result does not contain any position information. It is designed for
#   classifiers, which simply provide class probabilities given an instance of
#   source data (e.g., an image or a point cloud).

std_msgs/Header header

# A list of class probabilities. This list need not provide a probability for
#   every possible class, just ones that are nonzero, or above some
#   user-defined threshold.
ObjectHypothesis[] results

# Source data that generated this classification are not a part of the message.
# If you need to access them, use an exact or approximate time synchronizer in
# your code, as this message's header should match the header of the source
# data.
