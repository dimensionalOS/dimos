# A keyed mesh chunk. Flat arrays contain consecutive xyz vertices, normals, and triangles.
std_msgs/Header header
int32[3] key
float32[] vertices
float32[] normals
uint32[] faces
