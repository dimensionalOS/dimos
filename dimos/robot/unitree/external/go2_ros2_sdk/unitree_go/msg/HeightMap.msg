# Header
float64 stamp         # timestamp
string frame_id      # world frame id

# Map info
float32 resolution     # The map resolution [m/cell]
uint32 width  # Map width along x-axis [cells]
uint32 height # Map height alonge y-axis [cells]
float32[2] origin      # Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame

# Map data, in x-major order, starting with [0,0], ending with [width, height]
# For a cell whose 2d-array-index is [ix, iy]，
#    its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]
#    its cell value is: data[width * iy + ix]
float32[] data