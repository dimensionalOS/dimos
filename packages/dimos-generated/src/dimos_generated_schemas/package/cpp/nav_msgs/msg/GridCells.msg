# An array of cells in a 2D grid

std_msgs/Header header

# Width of each cell
float32 cell_width

# Height of each cell
float32 cell_height

# Each cell is represented by the Point at the center of the cell
geometry_msgs/Point[] cells
