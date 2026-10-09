# A key value pair that maps an integer class_id to a string class label
#   in computer vision systems.

# The int value that identifies the class.
# Elements identified with 65535, the maximum uint16 value are assumed
#   to belong to the "UNLABELED" class. For vision pipelines using less
#   than 255 classes the "UNLABELED" is the maximum value in the uint8
#   range.
uint16 class_id

# The name of the class represented by the class_id
string class_name
