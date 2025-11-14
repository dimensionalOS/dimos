# Timestamp (in seconds since epoch)
float64 stamp

# Frame ID for the coordinate frame
string frame_id

# Resolution of the voxel map
float64 resolution

# Origin of the voxel map (x, y, z)
float64[3] origin

# Width of the voxel map in each dimension (x, y, z)
int16[3] width

# Source size of the compressed data
uint64 src_size

# Compressed voxel data
uint8[] data

